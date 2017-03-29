function trans_1_joints_xy = trans1jointsxy(L, Theta1)
% 1st joint endpoint
%trans_1_joints_xy = [ L*sin(Theta1) L*cos(Theta1) ];
trans_1_joints_xy = [ L*cos(Theta1) L*sin(Theta1) ];
end

