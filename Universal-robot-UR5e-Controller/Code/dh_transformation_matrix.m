function T = dh_transformation_matrix(theta, dh_parameters)
    a = dh_parameters(2);
    d = dh_parameters(3);
    alpha = dh_parameters(4);
    cos_t = cos(theta);
    sin_t = sin(theta); 
    cos_a = cos(alpha);
    sin_a = sin(alpha);
    % Compute transformation matrix using DH convention
    T = [cos_t -sin_t*cos_a  sin_t*sin_a  a*cos_t;
         sin_t  cos_t*cos_a -cos_t*sin_a  a*sin_t;
         0            sin_a        cos_a        d;
         0                0            0        1];
end