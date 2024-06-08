function R_updated = update_rotation_matrix(R, w, dt)
    % Function to update rotation matrix R using angular velocity vector w and timestep dt.
    
    % Calculate rotation angle
    theta = norm(w) * dt;
    
    % Check for small angles to avoid division by zero
    if theta == 0
        R_updated = R;
        return;
    end
    
    % Normalize angular velocity vector
    w_unit = w / norm(w);
    
    % Skew-symmetric matrix from angular velocity vector
    skew_w = [0, -w_unit(3), w_unit(2);
              w_unit(3), 0, -w_unit(1);
              -w_unit(2), w_unit(1), 0];
    
    % Rodrigues' rotation formula
    delta_R = eye(3) + sin(theta) * skew_w + (1 - cos(theta)) * skew_w^2;
    
    % Update rotation matrix
    R_updated = normalize(R * delta_R,"norm",2);
end


% % Example usage:
% % Initial rotation matrix
% R = eye(3);
% % Angular velocity vector (for example)
% w = [0.1; 0.2; 0.3];
% % Timestep
% dt = 0.01;
% % Update rotation matrix
% R = update_rotation_matrix(R, w, dt);


