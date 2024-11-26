function [End_Effector_Pos] = PD_Control()
    % X-Coordinate Tracking
    syms s t
    Kd1 = 10;
    Kp1 = 10;
    T_Total = 10;
    T = 0:0.5:T_Total;
    X_Trajectory = 35+10*cos(2*pi*t/T_Total);
    
    desired_laplace = laplace(diff(diff(X_Trajectory)) + Kp1*diff(X_Trajectory) + Kd1*X_Trajectory);
    tracking_laplace = (desired_laplace)/(s^2 + Kd1*s + Kp1);
    X_tracking = ilaplace(tracking_laplace);
    
    
    syms s t
    Kd2 = 10;
    Kp2 = 10;
    
    Y_Trajectory = 35+10*sin(2*pi*t/T_Total);
    
    desired_laplace = laplace(diff(diff(Y_Trajectory)) + Kp2*diff(Y_Trajectory) + Kd2*Y_Trajectory);
    tracking_laplace = (desired_laplace)/(s^2 + Kd2*s + Kp2);
    Y_tracking = ilaplace(tracking_laplace);
    
    
    syms s t
    Kd3 = 10;
    Kp3 = 10;
    Z_Trajectory = 25;
    desired_laplace = Z_Trajectory*Kp3/s;
    tracking_laplace = (desired_laplace)/(s^2 + Kd3*s + Kp3);
    Z_tracking = ilaplace(tracking_laplace);
    
    End_Effector_Pos = [X_tracking; Y_tracking; Z_tracking];
end