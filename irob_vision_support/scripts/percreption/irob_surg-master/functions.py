# -*- coding: utf-8 -*-
import numpy as np
import torch
import torch.nn.functional as F
import torchvision.transforms.functional as tvf

def img_to_tensor(im, normalize=None):
    tensor = torch.from_numpy(np.moveaxis((im / 255), -1, 0).astype(np.float32))
    if normalize is not None:
        return tvf.normalize(tensor, **normalize)
    return tensor

def rebuild_from_disparity(img, disp):
    batch_size, _, height, width = img.size()

# Original coordinates of pixels
    x_base = torch.linspace(0, 1, width).repeat(batch_size,
                           height, 1).type_as(img)
    y_base = torch.linspace(0, 1, height).repeat(batch_size,
                           width, 1).transpose(1, 2).type_as(img)

    # Apply shift in X direction
    x_shifts = disp[:, 0, :, :]  # Disparity is passed in NCHW format with 1 channel
    flow_field = torch.stack((x_base + x_shifts, y_base), dim=3)
        # In grid_sample coordinates are assumed to be between -1 and 1
    output = F.grid_sample(img, 2*flow_field - 1, mode='bilinear',
                           padding_mode='zeros')

    return output

def jaccard(y_true, y_pred):
    intersection = (y_true * y_pred)
    union = y_true + y_pred - intersection
    return (intersection + 1e-15) / (union + 1e-15)
def recon_from_disp_cv(disparity, img_opp):
    h, w = disparity.shape
    im_rec = np.zeros_like(disparity)
    for y in range(h):    
        for x in range(w):
           intense = disparity[y,x]
           x_range = x + intense
           if x_range >= 0 and x_range < w:
               im_rec[y, x] = img_opp[y, x_range]
    return im_rec

def calc_loss_img_diff_cv(img_recon, img_expected, weight = 0.5):
    h, w = img_expected.shape
    avg_val = 1/(h * w)
    diff = 0
    for y in range(h):    
        for x in range(w):
            diff = diff + (np.abs((img_expected[y, x] - img_recon[y, x])))            
    return weight * (avg_val * diff)

def calc_loss_img_disp_cv(disp_est_l, disp_est_r, weight = 1.0):
    h, w = disp_est_l.shape
    avg_val = 1/(h * w)
    diff = 0
    for i in range(h):
        for j in range(w):
            diff = diff + (np.abs(disp_est_l[i,j] - disp_est_r[(i + disp_est_r[i,j]), j]))
    return  weight * (avg_val * diff)

def recon_from_disp_pil(disparity, img_opp):
    _, h, w, _ = disparity.shape
    recon = torch.zeros_like(img_opp)
    for y in range(h):    
        for x in range(w):
           intense = disparity[0, y, x]
           intense = (intense.detach().cpu().numpy()[0])
           x_range = x + np.int32(intense)
           if x_range >= 0 and x_range < w:
               #print(img_opp[0, y, x_range])
               recon[0, y, x] = img_opp[0, y, x_range]
    return recon

def calc_loss_img_diff(disp_im, img_from_rec, img_expected, weight = 0.5):
    b, h, w, d = disp_im.size()
    im_rec = torch.zeros_like(img_expected, requires_grad=True)
    loss_batch = 0
    for batch in range(b):
        for y in range(h):
            for x in range(w):
                disp_im_idx = disp_im[batch][y][x].detach().numpy() + x
                im_rec[batch][y][x][0] = 0
                im_rec[batch][y][x][1] = 0
                im_rec[batch][y][x][2] = 0
                if(int(disp_im_idx) < w):
                    im_rec[batch][y][x][0] = img_from_rec[batch][y][int(disp_im_idx)][0]
                    im_rec[batch][y][x][1] = img_from_rec[batch][y][int(disp_im_idx)][1]
                    im_rec[batch][y][x][2] = img_from_rec[batch][y][int(disp_im_idx)][2]
        loss = 1/(h*w) * (torch.sum(img_expected - im_rec).abs())
        loss_batch = loss_batch + loss
    return loss_batch * weight



def calc_loss_img_disp(disp_est_l, disp_est_r, weight = 1.0):
    b, h, w, d = disp_est_l.size()
    dep_r = torch.zeros((b, h, w, d))
    loss_batch = 0
    for batch in range(b):
        for y in range(h):
            for x in range(w):
                disp_r_idx = dep_r[batch][y][x].detach().numpy() + x
                dep_r[batch][y][x] = 0
                if(int(disp_r_idx) < w):
                    dep_r[batch][y][x] = disp_est_r[batch][y][int(disp_r_idx)]
        loss = 1/(h*w) * (torch.sum(disp_est_l - dep_r).abs())
        loss_batch = loss_batch + loss
    return loss_batch * weight

def loss_fn(dep_l, dep_r, im_l, im_r, device):
    dep_l = dep_l.permute(0, 3, 2, 1).abs() * 255
    dep_r = dep_r.permute(0, 3, 2, 1).abs() * 255
    im_l = im_l.permute(0, 3, 2, 1)
    im_r = im_r.permute(0, 3, 2, 1)
    dep_diff = calc_loss_img_disp(dep_l, dep_r)
    rec_diff_l = calc_loss_img_diff(dep_l, im_r, im_l)
    rec_diff_r = calc_loss_img_diff(dep_r, im_l, im_r)
    return rec_diff_l + rec_diff_r + dep_diff

def apply_disparity(input_images, x_offset, wrap_mode='border', tensor_type='torch.FloatTensor'):
        num_batch, num_channels, width, height = input_images.size()

        # Handle both texture border types
        edge_size = 0
        if wrap_mode == 'border':
            edge_size = 1
            # Pad last and second-to-last dimensions by 1 from both sides
            input_images = F.pad(input_images, (1, 1, 1, 1))
        elif wrap_mode == 'edge':
            edge_size = 0
        else:
            return None

        # Put channels to slowest dimension and flatten batch with respect to others
        input_images = input_images.permute(1, 0, 2, 3).contiguous()
        im_flat = input_images.view(num_channels, -1)

        # Create meshgrid for pixel indicies (PyTorch doesn't have dedicated
        # meshgrid function)
        x = torch.linspace(0, width - 1, width).repeat(height, 1).type(tensor_type)
        y = torch.linspace(0, height - 1, height).repeat(width, 1).transpose(0, 1).type(tensor_type)
        # Take padding into account
        x = x + edge_size
        # y = y + edge_size
        # Flatten and repeat for each image in the batch
        x = x.view(-1).repeat(1, num_batch)
        y = y.contiguous().view(-1).repeat(1, num_batch)

        # Now we want to sample pixels with indicies shifted by disparity in X direction
        # For that we convert disparity from % to pixels and add to X indicies
        x = x + x_offset.contiguous().view(-1) * width
        # Make sure we don't go outside of image
        x = torch.clamp(x, 0.0, width - 1 + 2 * edge_size)
        # Round disparity to sample from integer-valued pixel grid
        y0 = torch.floor(y)
        # In X direction round both down and up to apply linear interpolation
        # between them later
        x0 = torch.floor(x)
        x1 = x0 + 1
        # After rounding up we might go outside the image boundaries again
        x1 = x1.clamp(max=(width - 1 + 2 * edge_size))

        # Calculate indices to draw from flattened version of image batch
        dim2 = (width + 2 * edge_size)
        dim1 = (width + 2 * edge_size) * (height + 2 * edge_size)
        # Set offsets for each image in the batch
        base = dim1 * torch.arange(num_batch).type(tensor_type)
        base = base.view(-1, 1).repeat(1, height * width).view(-1)
        # One pixel shift in Y  direction equals dim2 shift in flattened array
        base_y0 = base + y0 * dim2
        # Add two versions of shifts in X direction separately
        idx_l = base_y0 + x0
        idx_r = base_y0 + x1

        # Sample pixels from images
        pix_l = im_flat.gather(1, idx_l.repeat(num_channels, 1).long())
        pix_r = im_flat.gather(1, idx_r.repeat(num_channels, 1).long())

        # Apply linear interpolation to account for fractional offsets
        weight_l = x1 - x
        weight_r = x - x0
        output = weight_l * pix_l + weight_r * pix_r

        # Reshape back into image batch and permute back to (N,C,H,W) shape
        output = output.view(num_channels, num_batch, height, width).permute(1, 0, 2, 3)

        return output
    
    
    