function [] = sim_main_cspace_search()
% search the c-space and corresponding work space
%
clf

theta_range_ = pi/2;
l_joint_ = 10;

Theta1 = 0;
Theta2 = 0;
Theta3 = 0;

Q_init_ = [Theta1, Theta2, Theta3];
start_xy_mat = arm_vertex_mat(l_joint_, Q_init_);

subplot(1,2,1)
plot(start_xy_mat(:,1), start_xy_mat(:,2), 'r.-')
axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
hold on
title('Work space')

subplot(1,2,2)
plot3(Q_init_(1,1), Q_init_(1,2), Q_init_(1,3), 'bo')
axis([0 pi -theta_range_ theta_range_ -theta_range_ theta_range_])
hold on
title('C-space')

Q_tree_ = Q_init_;
step_angle_ = pi/100;
iteration = 1;
tree_index_ = 1;

while (iteration < 1000)
    %random_angles_ = unifrnd(-theta_range_,theta_range_,1,3);
    random_angles_1 = unifrnd(theta_range_-pi/2,theta_range_+pi/2);
    random_angles_2 = unifrnd(-theta_range_,theta_range_);
    random_angles_3 = unifrnd(-theta_range_,theta_range_);
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    for j = 1:iteration
        dis_(j) = sum((random_angles_ - Q_tree_(j,:)).^2);
    end
    [min_dis_,n] = min(dis_);
    Q_near_ = Q_tree_(n,:);
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_);
    
    plot_xy_mat = arm_vertex_mat(l_joint_, Q_new_);
    subplot(1,2,1)
    plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r')
    axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
    
    subplot(1,2,2)
    qnew_ = [Q_near_;Q_new_];
    plot3(qnew_(:,1), qnew_(:,2), qnew_(:,3), 'r-')
    axis([theta_range_-pi/2 theta_range_+pi/2 -theta_range_ theta_range_ -theta_range_ theta_range_])
    box on
    drawnow
    
    Q_tree_ = [Q_tree_; Q_new_];
    qtree_mat_(n, tree_index_+1) = 1;
    
    iteration = iteration + 1;
    tree_index_ = tree_index_+1;
end


%{
while (iteration < 10000)%iteration<=n_iteration)
    random_angles_ = unifrnd(-pi,pi,1,3);
    random_xy_mat = arm_vertex_mat(l_joint_, random_angles_);
    
    for j = 1:iteration
        dis_(j) = sqrt(sum((random_xy_mat(4,:) - P_goal_arm(j,:)).^2));
    end
    [x,n] = min(dis_);
    Q_near_ = Q_tree_(n,:);
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_);
    P_goal_new = arm_vertex_mat(l_joint_, Q_new_);
    
    
    
    plot(P_goal_new(:,1), P_goal_new(:,2), 'r.-')
    
    axis([-l_joint_*4 l_joint_*4 -l_joint_*2 l_joint_*4])
    drawnow
    %pause(0.1)
    Q_tree_ = [Q_tree_; Q_new_];
    P_goal_arm = [P_goal_arm; P_goal_new];
    iteration = iteration + 1;
end

%}

%{

Q_init_ = [Theta1, Theta2, Theta3];
plot_xy_mat = arm_vertex_mat(l_joint_, Theta1, Theta2, Theta3);

plot(plot_xy_mat(:,1), plot_xy_mat(:,2), 'r.-', ...
    conveyor_xy(:,1), conveyor_xy(:,2), 'k--', ...
    P_goal_conveyor(1), P_goal_conveyor(2), 'bo')
axis([-l_joint_*4 l_joint_*4 -l_joint_*2 l_joint_*4])

P_goal_arm = plot_xy_mat(4,:);
dis_end = sqrt(sum((P_goal_arm-P_goal_conveyor).^2));

step_ = 0.2;
step_angle_ = pi/50;

while (dis_end > step_)%iteration<=n_iteration)
    random_angles_ = unifrnd(-pi,pi,1,3);




for i = 1:num
    trans_1_joints_xy = [l_joint_*sin(Theta1(i));
        l_joint_*cos(Theta1(i))];
    
    for j = 1:num
        trans_2_joints_xy = [ l_joint_*sin(Theta1(i)) + l_joint_*cos(Theta1(i))*sin(Theta2(j)) + l_joint_*cos(Theta2(j))*sin(Theta1(i));
            l_joint_*cos(Theta1(i)) + l_joint_*cos(Theta1(i))*cos(Theta2(j)) - l_joint_*sin(Theta1(i))*sin(Theta2(j)) ];
        
        for k = 1:num
            trans_3_joints_xy = [ l_joint_*sin(Theta1(i)) + l_joint_*cos(Theta1(i))*sin(Theta2(j)) + l_joint_*cos(Theta2(j))*sin(Theta1(i)) + l_joint_*cos(Theta3(k))*(cos(Theta1(i))*sin(Theta2(j)) + cos(Theta2(j))*sin(Theta1(i))) + l_joint_*sin(Theta3(k))*(cos(Theta1(i))*cos(Theta2(j)) - sin(Theta1(i))*sin(Theta2(j)));
                l_joint_*cos(Theta1(i)) + l_joint_*cos(Theta1(i))*cos(Theta2(j)) - l_joint_*sin(Theta1(i))*sin(Theta2(j)) + l_joint_*cos(Theta3(k))*(cos(Theta1(i))*cos(Theta2(j)) - sin(Theta1(i))*sin(Theta2(j))) - l_joint_*sin(Theta3(k))*(cos(Theta1(i))*sin(Theta2(j)) + cos(Theta2(j))*sin(Theta1(i))) ];
            
            
            
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
%}

end








