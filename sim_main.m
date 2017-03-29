function [] = sim_main()
% reach the goal on the conveyor, in a faster way
% more efficient, search space optimized by
% comparing the distance of new node and 
% min-distance of the tree without new node
% ignore the joint range

% 20170216
%
clf

theta_range_ = pi/2;
l_joint_ = 10;

conveyor_xy =[-l_joint_*4 l_joint_*1.5;
    l_joint_*4 l_joint_*1.5];

P_goal_conveyor =[l_joint_*unifrnd(-2,2) l_joint_*1.5];

Theta1 = -pi/2;
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
dis_end_mat(1) = dis_end;

Q_tree_ = Q_init_;
step_angle_ = pi/50;
iteration = 1;
tree_index_ = 1;

plot_xy_mat = arm_vertex_mat(l_joint_, Q_init_);
plotf_xy_mat = arm_vertex_mat(l_joint_, Q_init_+step_angle_);
%dis_circle_=sqrt(sum((plot_xy_mat(4,:)-plotf_xy_mat(4,:)).^2))

random_angles_1 = unifrnd(-theta_range_-pi/2,theta_range_-pi/2);
random_angles_2 = unifrnd(-theta_range_,theta_range_);
random_angles_3 = unifrnd(-theta_range_,theta_range_);

theta_range_box = theta_range_*2;

while (iteration < 10000)
    
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    
    for j = 1:iteration
        dis_(j) = sum((random_angles_ - Q_tree_(j,:)).^2);
    end
    [min_dis_,n] = min(dis_);
    Q_near_ = Q_tree_(n,:);
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_);
    
    plot_xy_mat_near = arm_vertex_mat(l_joint_, Q_near_);
    plot_xy_mat = arm_vertex_mat(l_joint_, Q_new_);
    
    %dis_end_second = sqrt(sum((plot_xy_mat_near(end,:)-P_goal_conveyor).^2));
    dis_end = sqrt(sum((plot_xy_mat(end,:)-P_goal_conveyor).^2));
    
    dis_end_mat(iteration+1) = dis_end;
    [min_dis_end,m] = min(dis_end_mat(1:end-1));
    
    % set the search space not out of each joint range
    % WARNING: 
    %          actually useless in the following condition if
    f_min_new = (Q_new_(1)-step_angle_ > -theta_range_-pi/2) && ...
        (Q_new_(2)-step_angle_*2 > -theta_range_) && ...
        (Q_new_(3)-step_angle_*3 > -theta_range_);
    f_max_new = (Q_new_(1)+step_angle_ < theta_range_-pi/2) && ...
        (Q_new_(2)+step_angle_*2 < theta_range_) && ...
        (Q_new_(3)+step_angle_*3 < theta_range_);
    
    f_min_near = (Q_tree_(m,1)-step_angle_ > -theta_range_-pi/2) && ...
        (Q_tree_(m,2)-step_angle_*2 > -theta_range_) && ...
        (Q_tree_(m,3)-step_angle_*3 > -theta_range_);
    f_max_near = (Q_tree_(m,1)+step_angle_ < theta_range_-pi/2) && ...
        (Q_tree_(m,2)+step_angle_*2 < theta_range_) && ...
        (Q_tree_(m,3)+step_angle_*3 < theta_range_);
    
    f_min_max = f_min_new && f_max_new && f_min_near && f_max_near;
    
    f_dis = (dis_end < min_dis_end);
    
    if  f_dis %&& f_min_max
        random_angles_1 = unifrnd(Q_new_(1)-step_angle_, Q_new_(1)+step_angle_);
        random_angles_2 = unifrnd(Q_new_(2)-step_angle_*2, Q_new_(2)+step_angle_*2);
        random_angles_3 = unifrnd(Q_new_(3)-step_angle_*3, Q_new_(3)+step_angle_*3);
    else
        random_angles_1 = unifrnd(Q_tree_(m,1)-step_angle_, Q_tree_(m,1)+step_angle_);
        random_angles_2 = unifrnd(Q_tree_(m,2)-step_angle_*2, Q_tree_(m,2)+step_angle_*2);
        random_angles_3 = unifrnd(Q_tree_(m,3)-step_angle_*3, Q_tree_(m,3)+step_angle_*3);
    end
    
    if (plot_xy_mat(:,2)<l_joint_*1.45)
        
        subplot(1,2,1)
        plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r')
        axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
        
        subplot(1,2,2)
        qnew_ = [Q_near_;Q_new_];
        plot3(qnew_(:,1), qnew_(:,2), qnew_(:,3), 'r-')
        axis([-theta_range_box theta_range_box -theta_range_box ...
            theta_range_box -theta_range_box theta_range_box])
        box on
        drawnow
        
        Q_tree_ = [Q_tree_; Q_new_];
        qtree_mat_(n, tree_index_+1) = 1;
        
        if dis_end < 1
            [q_trees_, n_start] = find_each_arm(qtree_mat_,tree_index_+1,Q_tree_)
            size(q_trees_)
            for k=1:n_start
                % draw the final arm path
                subplot(1,2,1)
                plot_xy_mat = arm_vertex_mat(l_joint_, q_trees_(n_start-k+1,:));
                plot(plot_xy_mat(:,1),plot_xy_mat(:,2),'g.-');
                axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
                drawnow
                %pause(0.2)
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
        
        iteration = iteration + 1;
        tree_index_ = tree_index_+1;
        
    end
    
end
iteration

end


