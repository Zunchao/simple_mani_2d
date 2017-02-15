function [] = sim_main()
%
%   Detail_joint_ed expl_joint_anation goes here
cla

num = 25;
Theta1 = linspace(-pi/2, pi/2, num);
Theta2 = linspace(-pi/2, pi/2, num);
Theta3 = linspace(-pi/2, pi/2, num);

l_joint_ = 10;
init_xy = [0, 0;
    0, l_joint_;
    0, l_joint_*2;
    0, l_joint_*3];

plot(init_xy(:,1), init_xy(:,2), 'b.-')
axis([-l_joint_*4 l_joint_*4 -l_joint_*2 l_joint_*4])
hold on

for i = 1:num
    trans_1_joints_xy = [l_joint_*sin(Theta1(i));
        l_joint_*cos(Theta1(i))];
    
    for j = 1:num
        trans_2_joints_xy = [ l_joint_*sin(Theta1(i)) + l_joint_*cos(Theta1(i))*sin(Theta2(j)) + l_joint_*cos(Theta2(j))*sin(Theta1(i));
            l_joint_*cos(Theta1(i)) + l_joint_*cos(Theta1(i))*cos(Theta2(j)) - l_joint_*sin(Theta1(i))*sin(Theta2(j)) ];
        
        for k = 1:num
            trans_3_joints_xy = [ l_joint_*sin(Theta1(i)) + l_joint_*cos(Theta1(i))*sin(Theta2(j)) + l_joint_*cos(Theta2(j))*sin(Theta1(i)) + l_joint_*cos(Theta3(k))*(cos(Theta1(i))*sin(Theta2(j)) + cos(Theta2(j))*sin(Theta1(i))) + l_joint_*sin(Theta3(k))*(cos(Theta1(i))*cos(Theta2(j)) - sin(Theta1(i))*sin(Theta2(j)));
                l_joint_*cos(Theta1(i)) + l_joint_*cos(Theta1(i))*cos(Theta2(j)) - l_joint_*sin(Theta1(i))*sin(Theta2(j)) + l_joint_*cos(Theta3(k))*(cos(Theta1(i))*cos(Theta2(j)) - sin(Theta1(i))*sin(Theta2(j))) - l_joint_*sin(Theta3(k))*(cos(Theta1(i))*sin(Theta2(j)) + cos(Theta2(j))*sin(Theta1(i))) ];
            
            conveyor_xy =[-l_joint_*4 l_joint_*1.5;
                l_joint_*4 l_joint_*1.5];
            
            plot_xy = [ 0, 0;
                trans_1_joints_xy(1), trans_1_joints_xy(2);
                trans_2_joints_xy(1), trans_2_joints_xy(2);
                trans_3_joints_xy(1), trans_3_joints_xy(2)];
            
            plot(plot_xy(:,1), plot_xy(:,2), 'r.-', conveyor_xy(:,1), conveyor_xy(:,2), 'k--')
            axis([-l_joint_*4 l_joint_*4 -l_joint_*2 l_joint_*4])
            %plot_simple_conveyor(l_joint_)
            hold off
            drawnow
            
        end
    end
end


end



















