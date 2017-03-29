function trans_3_joints_xy = trans3jointsxy(L, Theta1, Theta2, Theta3)
% 3rd joint endpoints
%trans_3_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1) + L*cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + L*sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2));
%    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) + L*cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - L*sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) ];
trans_3_joints_xy = [ L*cos(Theta1) + L*cos(Theta1+Theta2) + L*cos(Theta1+Theta2+Theta3);
   L*sin(Theta1) + L*sin(Theta1+Theta2) + L*sin(Theta1+Theta2+Theta3) ];
end

