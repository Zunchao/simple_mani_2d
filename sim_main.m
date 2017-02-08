function [ output_args ] = sim_main(  )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

trans_1_joints_xy = [L*sin(Theta1);
    L*cos(Theta1)];

trans_2_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1);
    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) ]; 

trans_3_joints_xy = [ L*sin(Theta1) + L*cos(Theta1)*sin(Theta2) + L*cos(Theta2)*sin(Theta1) + L*cos(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) + L*sin(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2));
    L*cos(Theta1) + L*cos(Theta1)*cos(Theta2) - L*sin(Theta1)*sin(Theta2) + L*cos(Theta3)*(cos(Theta1)*cos(Theta2) - sin(Theta1)*sin(Theta2)) - L*sin(Theta3)*(cos(Theta1)*sin(Theta2) + cos(Theta2)*sin(Theta1)) ];


end

