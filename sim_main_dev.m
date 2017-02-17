function [] = sim_main_dev()
% dev manuscript
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

P_obstacles_ = [(P_goal_conveyor(1)-30)/2, P_goal_conveyor(2)/2];

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
    
    f_end_ = (plot_xy_mat(end,1)<(P_obstacles_(1)+1)) && ...
        (plot_xy_mat(end,1)>(P_obstacles_(1)-1)) && ...
        (plot_xy_mat(end,2)>(P_obstacles_(2)-1))&& ...
        (plot_xy_mat(end,2)>(P_obstacles_(2)-1)); 
        
    f_end_ = ~f_end_;
    
    if  f_dis && f_end_
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
        plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r', P_obstacles_(1), P_obstacles_(2), 'ko')
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
                plot(plot_xy_mat(:,1),plot_xy_mat(:,2),'g.-', P_obstacles_(1), P_obstacles_(2), 'ko');
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
        
        iteration = iteration + 1;
        tree_index_ = tree_index_+1;
        
    end
    
end
iteration

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





function [theta1, theta2, theta3] = random_angles_joint(delta_th, theta11, theta21, theta31)
% joint 1 state search range
if theta11 - delta_th < 0
    theta11 = 0;
else
    theta11 = theta11 - delta_th;
end

if theta11 + delta_th > pi
    theta12 = pi;
else
    theta12 = theta11 + delta_th;
end

% joint 2 state search range
if theta21 - delta_th*2 < -pi/2
    theta21 = -pi/2;
else
    theta21 = theta21 - delta_th*2;
end

if theta21 + delta_th*2 > pi/2
    theta22 = pi/2;
else
    theta22 = theta21 + delta_th*2;
end

% joint 3 state search range
if theta31 - delta_th*3 < -pi/2
    theta31 = -pi/2;
else
    theta31 = theta31 - delta_th*3;
end

if theta31 + delta_th*3 > pi/2
    theta32 = pi/2;
else
    theta32 = theta31 + delta_th*3;
end

theta1 = unifrnd(theta11, theta12);
theta2 = unifrnd(theta21, theta22);
theta3 = unifrnd(theta31, theta32);

end





