function  tnum = trans_of_multijoints()
% output trans matrics of i-joints in a string format
syms Theta1 Theta2 Theta3 L1 L2 L3
T1 = [cos(Theta1) -sin(Theta1) 0;
    sin(Theta1) cos(Theta1) 0;
    0 0 1];

T2 = [cos(Theta2) -sin(Theta2) L1;
    sin(Theta2) cos(Theta2) 0;
    0 0 1];

T3 = [cos(Theta3) sin(Theta3) L2;
    -sin(Theta3) cos(Theta3) 0;
    0 0 1];

T4 = [1 0 L3;
    0 1 0;
    0 0 1];


trand_3_joints = T1*T2*T3*T4

%{
trand_3_joints =
 
[ cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) + sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)), sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)), L1*cos(Theta1) + L3*(cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) + sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1))) + L2*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2));
 cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) - sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)), cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) + sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)), L1*sin(Theta1) + L3*(cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) - sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2))) + L2*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1));
     0,  0,  1]

%}

end












