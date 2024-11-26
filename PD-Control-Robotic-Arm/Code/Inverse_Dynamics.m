function angles = Inverse_Dynamics(position, a1, a2, a3)
    x = position(1);
    y = position(2);
    z = position(3);
    theta1 = atan2(y ,x);
    theta3 = acos((x^2 + y^2 + (z-a1)^2 - a2^2 - a3^2) / (2 * a2 * a3));
    k = sqrt(x^2 + y^2 + (z-a1)^2);
    theta2 =  atan2(z-a1, sqrt(x^2 + y^2)) - acos((a2^2 + k^2 - a3^2) / (2 * a2 * k));
    angles = [theta1; theta2; theta3];
end