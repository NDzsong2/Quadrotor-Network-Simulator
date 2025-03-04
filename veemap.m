function [ R_vee ] = veemap( R )
% quatCross takes estimated quaternion to the [quat_est_cric_cross] to be used later 

R_vee3 = R(1,2);
R_vee2 = R(1,3);
R_vee1 = R(2,3);

R_vee = [-R_vee1 ; R_vee2 ; -R_vee3];

end







