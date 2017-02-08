function  tnum = trans_of_multijoints()
% output trans matrics of i-joints in a string format
syms Theta1 Theta2 Theta3 L
T1 = [cos(Theta1) sin(Theta1) L*sin(Theta1);
    -sin(Theta1) cos(Theta1) L*cos(Theta1);
    0 0 1];

T2 = [cos(Theta2) sin(Theta2) L*sin(Theta2);
    -sin(Theta2) cos(Theta2) L*cos(Theta2);
    0 0 1];

T3 = [cos(Theta3) sin(Theta3) L*sin(Theta3);
    -sin(Theta3) cos(Theta3) L*cos(Theta3);
    0 0 1];

trand_2_joints = T1*T2
trand_3_joints = T1*T2*T3

%{
trand_2_joints = [ cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2), cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1), L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1);
    - cos(Theta1)*sin(Theta2) - cos(Theta2)*sin(Theta1), cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2), L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2);
    0, 0, 1]

 
trand_3_joints = [ cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)), cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)), L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1) + L*cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + L*sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2));
    - cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) - sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)), cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)), L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) + L*cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - L*sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1));
    0, 0, 1]

trand_2_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1);
    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) ]; 

trand_3_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1) + L*cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + L*sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2));
    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) + L*cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - L*sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) ];

%}

end












