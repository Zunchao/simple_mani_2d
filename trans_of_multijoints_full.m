function  trand_3_joints = trans_of_multijoints_full(q)
% output trans matrics of i-joints in a string format

Theta1=q(1);
Theta2=q(2);
Theta3=q(3);

L1=10; 
L2=10; 
L3=10;

T1 = [cos(Theta1) sin(Theta1) 0 L1*cos(Theta1);
    -sin(Theta1) cos(Theta1) 0 L1*sin(Theta1);
    0 0 1 0;
    0 0 0 1];

T2 = [cos(Theta2) sin(Theta2) 0 L2*cos(Theta2);
    -sin(Theta2) cos(Theta2) 0 L2*sin(Theta2);
    0 0 1 0;
    0 0 0 1];

T3 = [cos(Theta3) sin(Theta3) 0 L3*cos(Theta3);
    -sin(Theta3) cos(Theta3) 0 L3*sin(Theta3);
    0 0 1 0;
    0 0 0 1];

trand_2_joints = T1*T2;
trand_3_joints = T1*T2*T3;

%{
trand_2_joints =
 
[   cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2), cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1), 0, L1*sin(Theta1) + L2*cos(Theta1)*sin(Theta2) + L2*cos(Theta2)*sin(Theta1);
 - cos(Theta1)*sin(Theta2) - cos(Theta2)*sin(Theta1), cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2), 0, L1*cos(Theta1) + L2*cos(Theta1)*cos(Theta2) - L2*sin(Theta1)*sin(Theta2);
  0, 0, 1, 0;
  0, 0, 0, 1];
 
 
trand_3_joints =
 
[   cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)), cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)), 0, L1*sin(Theta1) + L2*cos(Theta1)*sin(Theta2) + L2*cos(Theta2)*sin(Theta1) + L3*cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + L3*sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2));
 - cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) - sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)), cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)), 0, L1*cos(Theta1) + L2*cos(Theta1)*cos(Theta2) - L2*sin(Theta1)*sin(Theta2) + L3*cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - L3*sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1));
 0, 0, 1, 0;
 0, 0, 0, 1];
 

trand_2_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1);
    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) ]; 

trand_3_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1) + L*cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + L*sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2));
    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) + L*cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - L*sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) ];

%}

end












