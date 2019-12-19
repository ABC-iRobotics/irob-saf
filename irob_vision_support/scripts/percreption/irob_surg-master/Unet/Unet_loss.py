import torch

def dice_loss(y_pred, y_true):
    numerator = 2 * torch.sum(y_true * y_pred)
    denomiator = torch.sum(y_true + y_pred)
    return 1 - (numerator +1) / (denomiator + 1)