function  fik_3_joints = ik_3joints_mani(p)
% inverse kinematics for 3R manipulator
% input position and orientation of goal
% output 2 solutions of theta1 theta2 theta2

eex=p(1);
eey=p(2);
eeo=p(3);

L1=10;
L2=10; 
L3=10;

%eex = L1*cos(Theta1)+L2*cos(Theta1+Theta2)+L3*cos(Theta1+Theta2+Theta3);
%eey = L1*sin(Theta1)+L2*sin(Theta1+Theta2)+L3*sin(Theta1+Theta2+Theta3);
%eeo = Theta1+Theta2+Theta3;

xw = eex-L3*cos(eeo);
yw = eey-L3*sin(eeo);

theta1 = atan2(yw,xw)-acos((xw*xw+yw*yw+L1*L1-L2*L2)/(2*L1*sqrt(xw*xw+yw*yw)));
theta2 = pi-acos((L1*L1+L2*L2-xw*xw-yw*yw)/(2*L1*L2));
theta3 = eeo-theta1-theta2;

%eex = L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)
%eey = L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)

fik_3_joints=[theta1,theta2,theta3];

theta1 = theta1+2*acos((xw*xw+yw*yw+L1*L1-L2*L2)/(2*L1*sqrt(xw*xw+yw*yw)));
theta2 = -theta2;
theta3 = eeo-theta1-theta2;

%eex = L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)
%eey = L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)

fik_3_joints=[fik_3_joints;theta1,theta2,theta3];
%sum(fik_3_joints')

end












