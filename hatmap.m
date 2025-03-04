function [ skew_matrix ] = hatmap(Omega)
% quatCross takes estimated quaternion to the [quat_est_cric_cross] to be used later 

    Omega1 = Omega(1);
    Omega2 = Omega(2);
    Omega3 = Omega(3);

    skew_matrix = [0 -Omega3 Omega2; Omega3 0 -Omega1; -Omega2 Omega1 0];

end







