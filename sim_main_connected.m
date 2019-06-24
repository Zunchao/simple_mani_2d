function [] = sim_main_connected()
% search the space and connect to the goal
% output the paths both ws and Cs
% naive and slow
% just search all the space

% 20170216

clf

theta_range_ = pi/2;
l_joint_ = 10;

conveyor_xy =[-l_joint_*4 l_joint_*1.8;
    l_joint_*4 l_joint_*1.8];

P_goal_conveyor =[l_joint_*unifrnd(-2,2) l_joint_*1.8];

Theta1 = 0;
Theta2 = 0;
Theta3 = 0;

Q_init_ = [Theta1, Theta2, Theta3];
start_xy_mat = arm_vertex_mat(l_joint_, Q_init_);

subplot(1,2,1)
plot(start_xy_mat(:,1), start_xy_mat(:,2), 'r.-')
axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
hold on
plot(conveyor_xy(:,1), conveyor_xy(:,2),'k--',P_goal_conveyor(1),P_goal_conveyor(2),'b*')
title('Work space')

subplot(1,2,2)
plot3(Q_init_(1,1), Q_init_(1,2), Q_init_(1,3), 'bo')
axis([-theta_range_ theta_range_ -theta_range_ theta_range_ -theta_range_ theta_range_])
hold on
title('C-space')

P_goal_arm = start_xy_mat(4,:);
dis_end = sqrt(sum((P_goal_arm-P_goal_conveyor).^2));

Q_tree_ = Q_init_;
step_angle_ = pi/20;
iteration = 1;
tree_index_ = 1;

plot_xy_mat = arm_vertex_mat(l_joint_, Q_init_);
plotf_xy_mat = arm_vertex_mat(l_joint_, Q_init_+step_angle_);
%dis_circle_=sqrt(sum((plot_xy_mat(4,:)-plotf_xy_mat(4,:)).^2))

while (iteration < 10000)
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
    %plotf_xy_mat = arm_vertex_mat(l_joint_, Q_near_);
    %dis_circle_=sqrt(sum((plot_xy_mat(4,:)-plotf_xy_mat(4,:)).^2))
    
    subplot(1,2,1)
    plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r')
    axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
    
    subplot(1,2,2)
    qnew_ = [Q_near_;Q_new_];
    plot3(qnew_(:,1), qnew_(:,2), qnew_(:,3), 'r-')
    axis([0 pi -theta_range_ theta_range_ -theta_range_ theta_range_])
    box on
    drawnow
    
    Q_tree_ = [Q_tree_; Q_new_];
    qtree_mat_(n, tree_index_+1) = 1;
    
    dis_end = sqrt(sum((plot_xy_mat(4,:)-P_goal_conveyor).^2));
    
    if dis_end < 2
        [q_trees_, n_start] = find_each_arm(qtree_mat_,tree_index_+1,Q_tree_)
        
        for k=1:n_start
            % draw the final arm path
            subplot(1,2,1)
            plot_xy_mat = arm_vertex_mat(l_joint_, q_trees_(n_start-k+1,:));
            plot(plot_xy_mat(:,1),plot_xy_mat(:,2),'g.-');
            drawnow 
            pause(0.2)
            
            % draw the final jointstate path
            subplot(1,2,2)
            if k < n_start
                qtree_2points = [q_trees_(n_start-k+1,:); q_trees_(n_start-k,:)];
                plot3(qtree_2points(:,1), qtree_2points(:,2), qtree_2points(:,3), 'g.-')
            end
            
        end
        
        break
    end
    
    iteration = iteration + 1;
    tree_index_ = tree_index_+1;
end

end



