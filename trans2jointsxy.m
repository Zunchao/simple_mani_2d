function trand_2_joints_xy = trans2jointsxy(L, Theta1, Theta2)
% 2nd joint endpoints
trand_2_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1);
    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) ];
end

