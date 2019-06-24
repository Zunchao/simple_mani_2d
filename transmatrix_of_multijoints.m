function  trand_3_joints_mat = transmatrix_of_multijoints(armconfig_vec)
% output trans matrics of i-joints in a string format
Theta1=armconfig_vec(1,1);
Theta2=armconfig_vec(1,2);
Theta3=armconfig_vec(1,3);
L=10;
T1 = [cos(Theta1) -sin(Theta1) 0;
    sin(Theta1) cos(Theta1) 0;
    0 0 1];

T2 = [cos(Theta2) -sin(Theta2) L;
    sin(Theta2) cos(Theta2) 0;
    0 0 1];

T3 = [cos(Theta3) -sin(Theta3) L;
    sin(Theta3) cos(Theta3) 0;
    0 0 1];

T4 = [1 0 L;
    0 1 0;
    0 0 1];
%T1*T2;
%T1*T2*T3;
%trand_2_joints = T1*T2*T3;
trand_3_joints_mat = T1*T2*T3*T4;

end
