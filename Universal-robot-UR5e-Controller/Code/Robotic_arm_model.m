function params = Robotic_arm_model()
    % UR5e DH parameters [theta(rad) a(m) d(m) alpha(rad)]
    params.dh = [
        0       0.1625  0       pi/2;
        0       0       -0.425  0;
        0       0       -0.3922 0;
        0       0.1333  0       pi/2;
        0       0.0997  0       -pi/2;
        0       0.0996  0       0
    ];
    
    % Link masses (kg)
    params.masses = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879];
    
    % Link COM positions (m)
    params.com = [
        0       -0.02561  0.00193;
        0.2125  0         0.11336;
        0.15    0         0.0265;
        0       -0.0018   0.01634;
        0       0.0018    0.01634;
        0       0         -0.001159
    ];
    
    % Inertia tensors (kg*m^2)
    params.inertia = {
        diag([0.0067, 0.0067, 0.0067]),
        diag([0.0149, 0.0149, 0.0149]),
        diag([0.0025, 0.0025, 0.0025]),
        diag([0.0013, 0.0013, 0.0013]),
        diag([0.0013, 0.0013, 0.0013]),
        diag([0.0001, 0.0001, 0.0001])
    };
    
    % Gravity vector
    params.g = [0; 0; -9.81];
end
