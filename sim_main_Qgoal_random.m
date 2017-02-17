function [] = sim_main_Qgoal_random()
% input: configuration of goal and start
% 20170217
clf

theta_range_ = pi/2;
l_joint_ = 10;

Q_start_ = [-pi/2, 0, 0];
start_xy_mat = arm_vertex_mat(l_joint_, Q_start_);

Q_goal_ = unifrnd(-theta_range_,theta_range_,1,3);
goal_xy_mat = arm_vertex_mat(l_joint_, Q_goal_);

subplot(1,2,1)
plot(start_xy_mat(:,1), start_xy_mat(:,2), 'r.-', goal_xy_mat(:,1), goal_xy_mat(:,2), 'b.-')
axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
hold on
title('Work space')

subplot(1,2,2)
plot3(Q_start_(1), Q_start_(2), Q_start_(3), 'ro', Q_goal_(1), Q_goal_(2), Q_goal_(3), 'bo')
axis([-theta_range_ theta_range_ -theta_range_ theta_range_ -theta_range_ theta_range_])
hold on
title('C-space')

Q_start_tree_ = Q_start_;
step_angle_ = pi/100;
iteration = 1;
tree_index_ = 1;

range_angles_1 = [Q_start_(1), Q_goal_(1)];
range_angles_2 = [Q_start_(2), Q_goal_(2)];
range_angles_3 = [Q_start_(3), Q_goal_(3)];

random_angles_1 = unifrnd(min(range_angles_1)-step_angle_, max(range_angles_1)+step_angle_);
random_angles_2 = unifrnd(min(range_angles_2)-step_angle_, max(range_angles_2)+step_angle_);
random_angles_3 = unifrnd(min(range_angles_3)-step_angle_, max(range_angles_3)+step_angle_);

theta_range_box = theta_range_*2;

while (iteration < 10000)
    
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    
    for j = 1:iteration
        dis_(j) = sum((random_angles_ - Q_start_tree_(j,:)).^2);
    end
    [min_dis_,n] = min(dis_);
    Q_near_ = Q_start_tree_(n,:);
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_);
    plot_xy_mat = arm_vertex_mat(l_joint_, Q_new_);
    
    subplot(1,2,1)
    plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r')
    axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
    drawnow
    
    subplot(1,2,2)
    qnew_ = [Q_near_;Q_new_];
    plot3(qnew_(:,1), qnew_(:,2), qnew_(:,3), 'r-')
    axis([-theta_range_box theta_range_box -theta_range_box ...
        theta_range_box -theta_range_box theta_range_box])
    box on
    
    Q_start_tree_ = [Q_start_tree_; Q_new_];
    qtree_mat_(n, tree_index_+1) = 1;
    
    dis_Qend = sqrt(sum((Q_goal_-Q_new_).^2));
    
    if dis_Qend < 0.05
        [q_trees_, n_start] = find_each_arm(qtree_mat_,tree_index_+1,Q_start_tree_)
        size(q_trees_)
        for k=1:n_start
            % draw the final arm path
            subplot(1,2,1)
            plot_xy_mat = arm_vertex_mat(l_joint_, q_trees_(n_start-k+1,:));
            plot(plot_xy_mat(:,1),plot_xy_mat(:,2),'g.-');
            axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
            drawnow
            pause(0.2)
            %hold off
            
            % draw the final jointstate path
            subplot(1,2,2)
            if k < n_start
                qtree_2points = [q_trees_(n_start-k+1,:); q_trees_(n_start-k,:)];
                plot3(qtree_2points(:,1), qtree_2points(:,2), qtree_2points(:,3), 'g.-')
            end
            
        end
        
        break
    end
    
    range_angles_1 = [Q_new_(1), Q_goal_(1)];
    range_angles_2 = [Q_new_(2), Q_goal_(2)];
    range_angles_3 = [Q_new_(3), Q_goal_(3)];
    
    random_angles_1 = unifrnd(min(range_angles_1)-step_angle_, max(range_angles_1)+step_angle_);
    random_angles_2 = unifrnd(min(range_angles_2)-step_angle_, max(range_angles_2)+step_angle_);
    random_angles_3 = unifrnd(min(range_angles_3)-step_angle_, max(range_angles_3)+step_angle_);
    
    iteration = iteration + 1;
    tree_index_ = tree_index_+1;
    
end
iteration
end


