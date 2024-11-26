function [End_Effector_Pos] = PD_Control_New()
    % X-Coordinate Tracking
   
    Kd1 = 10;
    Kp1 = 10;
    T_Total = 10;
    T = 0:0.5:T_Total;
    X_Trajectory = 35+10*cos(2*pi*T/T_Total);
    Y_Trajectory = 35+10*sin(2*pi*T/T_Total);
    Z_Trajectory = 25;
    a1 = 45/100; a2 = 45/100; a3 = 45/100;
    Q_r=[];
    for i=1:1:length(T)
    Q_r=[Q_r;(Inverse_Dynamics([X_Trajectory(i)/100,Y_Trajectory(i)/100,25/100],a1, a2, a3))'];
    end
    
    % desired_laplace = laplace(diff(diff(Y_Trajectory)) + Kp2*diff(Y_Trajectory) + Kd2*Y_Trajectory);
    % tracking_laplace = (desired_laplace)/(s^2 + Kd2*s + Kp2);
    % Y_tracking = ilaplace(tracking_laplace);
    % 
    % 
    % syms s t
    % Kd3 = 10;
    % Kp3 = 10;
    % Z_Trajectory = 25;
    % desired_laplace = Z_Trajectory*Kp3/s;
    % tracking_laplace = (desired_laplace)/(s^2 + Kd3*s + Kp3);
    % Z_tracking = ilaplace(tracking_laplace);

    % End_Effector_Pos = [X_tracking; Y_tracking; Z_tracking];
end