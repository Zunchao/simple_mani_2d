function [] = sim_main_Qgoal_merge()
% input configuration of gaol and start
% merge from both sides
% tree biuld from both gaol and start

% 20170217
clf

theta_range_ = pi/2;
l_joint_ = 10;

Q_start_ = [0, 0, 0];
start_xy_mat = arm_vertex_mat(l_joint_, Q_start_);

Q_goal_ = unifrnd(0,pi,1,3);
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

P_start_ = start_xy_mat(4,:);
P_goal_ = goal_xy_mat(4,:);

Q_start_tree_ = Q_start_;
Q_goal_tree_ = Q_goal_;
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
        dis_start_(j) = sum((random_angles_ - Q_start_tree_(j,:)).^2);
        dis_goal_(j) = sum((random_angles_ - Q_goal_tree_(j,:)).^2);
    end
    [min_dis_start,n] = min(dis_start_);
    Q_near_start = Q_start_tree_(n,:);
    Q_new_start = cal_new_(Q_near_start, random_angles_, step_angle_);
    splot_xy_mat = arm_vertex_mat(l_joint_, Q_new_start);
    qnew_start = [Q_near_start;Q_new_start];
    
    [min_dis_goal,m] = min(dis_goal_);
    Q_near_goal = Q_goal_tree_(n,:);
    Q_new_goal = cal_new_(Q_near_goal, random_angles_, step_angle_);
    gplot_xy_mat = arm_vertex_mat(l_joint_, Q_new_goal);
    qnew_goal = [Q_near_goal;Q_new_goal];
    
    subplot(1,2,1)
    plot(splot_xy_mat(4,1), splot_xy_mat(4,2), 'r', gplot_xy_mat(4,1), gplot_xy_mat(4,2), 'b')
    axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
    drawnow
    
    subplot(1,2,2)
    plot3(qnew_start(:,1), qnew_start(:,2), qnew_start(:,3), 'r-', ...
        qnew_goal(:,1), qnew_goal(:,2), qnew_goal(:,3), 'b-')
    axis([-theta_range_box theta_range_box -theta_range_box ...
        theta_range_box -theta_range_box theta_range_box])
    box on
    
    Q_start_tree_ = [Q_start_tree_; Q_new_start];
    qtree_mat_start(n, tree_index_+1) = 1;
    
    Q_goal_tree_ = [Q_goal_tree_; Q_new_goal];
    qtree_mat_goal(n, tree_index_+1) = 1;
    
    dis_Qend = sqrt(sum((Q_new_goal-Q_new_start).^2));
    
    if dis_Qend < 0.05
        [q_trees_start, n_start] = find_each_arm(qtree_mat_start,tree_index_+1,Q_start_tree_)
        size(q_trees_start)
        [q_trees_goal, n_goal] = find_each_arm(qtree_mat_goal,tree_index_+1,Q_goal_tree_)
        size(q_trees_goal)
        
        for k=1:n_start
            % draw the final arm path
            subplot(1,2,1)
            splot_xy_mat = arm_vertex_mat(l_joint_, q_trees_start(n_start-k+1,:));
            gplot_xy_mat = arm_vertex_mat(l_joint_, q_trees_goal(n_start-k+1,:));
            plot(splot_xy_mat(:,1),splot_xy_mat(:,2),'g.-', gplot_xy_mat(:,1),gplot_xy_mat(:,2),'g.-');
            axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
            drawnow
            pause(0.2)
            %hold off
            
            % draw the final jointstate path
            subplot(1,2,2)
            if k < n_start
                qtree_2points_start = [q_trees_start(n_start-k+1,:); q_trees_start(n_start-k,:)];
                qtree_2points_goal = [q_trees_goal(n_start-k+1,:); q_trees_goal(n_start-k,:)];
                plot3(qtree_2points_start(:,1), qtree_2points_start(:,2), qtree_2points_start(:,3), 'g.-', ...
                    qtree_2points_goal(:,1), qtree_2points_goal(:,2), qtree_2points_goal(:,3), 'g.-')
            end
            
        end
        
        break
    end

    range_angles_1 = [Q_new_start(1), Q_new_goal(1)];
    range_angles_2 = [Q_new_start(2), Q_new_goal(2)];
    range_angles_3 = [Q_new_start(3), Q_new_goal(3)];
    
    random_angles_1 = unifrnd(min(range_angles_1)-step_angle_, max(range_angles_1)+step_angle_);
    random_angles_2 = unifrnd(min(range_angles_2)-step_angle_, max(range_angles_2)+step_angle_);
    random_angles_3 = unifrnd(min(range_angles_3)-step_angle_, max(range_angles_3)+step_angle_);
    
    iteration = iteration + 1;
    tree_index_ = tree_index_+1;
    
end
iteration
end


