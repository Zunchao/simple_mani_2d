function trans_3_joints_xy_jacobian_mat = trans3jointsxy_jacobian_full(L, Theta1, Theta2, Theta3)
% 3rd joint endpoints
%{
syms Theta1 Theta2 Theta3
L=L;
f1= L*cos(Theta1) + L*cos(Theta1+Theta2) + L*cos(Theta1+Theta2+Theta3);
f2= L*sin(Theta1) + L*sin(Theta1+Theta2) + L*sin(Theta1+Theta2+Theta3);

f_jacobian=jacobian([f1, f2], [Theta1, Theta2, Theta3])
trans_3_joints_xy_jacobian=eval(subs(f_jacobian, {Theta1, Theta2, Theta3}, {theta1, theta2, theta3}));
%}

trans_3_joints_xy_jacobian_mat=[ - L*sin(Theta1 + Theta2) - L*sin(Theta1) - L*sin(Theta1 + Theta2 + Theta3), - L*sin(Theta1 + Theta2) - L*sin(Theta1 + Theta2 + Theta3), -L*sin(Theta1 + Theta2 + Theta3);
    L*cos(Theta1 + Theta2) + L*cos(Theta1) + L*cos(Theta1 + Theta2 + Theta3),   L*cos(Theta1 + Theta2) + L*cos(Theta1 + Theta2 + Theta3),  L*cos(Theta1 + Theta2 + Theta3);
    0 0 0;
    0 0 0;
    0 0 0;
    1 1 1];

end