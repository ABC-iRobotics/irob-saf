% in mm
model_3d_corners = [0, 0, 0; ...
                    0, 100, 0; ...
                    100, 100, 0; ...
                    100, 0, 0];
               
model_3d_targets = [20, 20, 0; ...
                    20, 40, 0; ...
                    40, 20, 0];
                
target_h = 30.0;

grasp_h = 20.0;

model_3d_approaches = model_3d_targets + repmat([0, 0, target_h], size(model_3d_targets, 1), 1);

model_3d_grasps = model_3d_targets + repmat([0, 0, grasp_h], size(model_3d_targets, 1), 1);

target_d = 6.0;

save pnp_phantom_model.mat model_3d_corners model_3d_targets model_3d_approaches model_3d_grasps target_h grasp_h target_d