function [] = sim_main_1ob_dev()
% 1 obstacle planning
%
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
dis_end_mat(1) = dis_end;

Q_tree_ = Q_init_;
step_angle_ = pi/100;
iteration = 1;
tree_index_ = 1;

plot_xy_mat = arm_vertex_mat(l_joint_, Q_init_);
plotf_xy_mat = arm_vertex_mat(l_joint_, Q_init_+step_angle_);
%dis_circle_=sqrt(sum((plot_xy_mat(4,:)-plotf_xy_mat(4,:)).^2))

random_angles_1 = unifrnd(theta_range_-pi/2,theta_range_+pi/2);
random_angles_2 = unifrnd(-theta_range_,theta_range_);
random_angles_3 = unifrnd(-theta_range_,theta_range_);

theta_range_box = theta_range_*2;

%P_obstacles_ = [(P_goal_conveyor(1)-30)/2, P_goal_conveyor(2)/2];
P_obstacles_ = [(30-P_goal_conveyor(1))*rand(1)+P_goal_conveyor(1)+2, P_goal_conveyor(2)*rand(1)];

while (iteration < 10000)
    
    if (plot_xy_mat(end,1) > (P_obstacles_(1)-2))%%&&(plot_xy_mat(end,1) > (P_obstacles_(1)-1))
        P_goal_ = [P_obstacles_(1)-2 P_obstacles_(2)-0.5];
        nx=1;
    else
        P_goal_ = P_goal_conveyor;
        nx=2;
    end
    
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    
    for j = 1:iteration
        dis_(j) = sum((random_angles_ - Q_tree_(j,:)).^2);
    end
    [min_dis_,n] = min(dis_);
    Q_near_ = Q_tree_(n,:);
    
    %step_angle_= rand(1)*step_angle_*2;
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_);
    
    plot_xy_mat_near = arm_vertex_mat(l_joint_, Q_near_);
    plot_xy_mat = arm_vertex_mat(l_joint_, Q_new_);
    
    %dis_end_second = sqrt(sum((plot_xy_mat_near(end,:)-P_goal_conveyor).^2));
    dis_end = sqrt(sum((plot_xy_mat(end,:)-P_goal_conveyor).^2));
    
    dis_end_mat(iteration+1) = dis_end;
    [min_dis_end,m] = min(dis_end_mat(1:end-1));
    
    f_dis = (dis_end < min_dis_end);
    
    f_end_ = (plot_xy_mat(end,1)<(P_obstacles_(1)+1)) && ...
        (plot_xy_mat(end,1)>(P_obstacles_(1)-1)) && ...
        (plot_xy_mat(end,2)>(P_obstacles_(2)-1)) && ...
        (plot_xy_mat(end,2)>(P_obstacles_(2)-1));
    
    f_end_ = ~f_end_;
    mi= ceil(iteration/2);
    
    
    f_min_new = (Q_new_(1)-step_angle_ > theta_range_-pi/2) && ...
        (Q_new_(2)-step_angle_*6 > -theta_range_*2) && ...
        (Q_new_(3)-step_angle_*9 > -theta_range_*2);
    f_max_new = (Q_new_(1)+step_angle_ < theta_range_+pi/2) && ...
        (Q_new_(2)+step_angle_*6 < theta_range_*2) && ...
        (Q_new_(3)+step_angle_*9 < theta_range_*2);
    
    f_min_near = (Q_tree_(m,1)-step_angle_ > theta_range_-pi/2) && ...
        (Q_tree_(m,2)-step_angle_*6 > -theta_range_*2) && ...
        (Q_tree_(m,3)-step_angle_*9 > -theta_range_*2);
    f_max_near = (Q_tree_(m,1)+step_angle_ < theta_range_+pi/2) && ...
        (Q_tree_(m,2)+step_angle_*6 < theta_range_*2) && ...
        (Q_tree_(m,3)+step_angle_*9 < theta_range_*2);
    
    f_min_max = f_min_new && f_max_new && f_min_near && f_max_near;
  

        if  f_dis
            random_angles_1 = unifrnd(Q_new_(1)-step_angle_, Q_new_(1)+step_angle_);
            random_angles_2 = unifrnd(Q_new_(2)-step_angle_*2, Q_new_(2)+step_angle_*2);
            random_angles_3 = unifrnd(Q_new_(3)-step_angle_*3, Q_new_(3)+step_angle_*3);
        else
            random_angles_1 = unifrnd(Q_tree_(m,1)-step_angle_, Q_tree_(m,1)+step_angle_);
            random_angles_2 = unifrnd(Q_tree_(m,2)-step_angle_*6, Q_tree_(m,2)+step_angle_*6);
            random_angles_3 = unifrnd(Q_tree_(m,3)-step_angle_*9, Q_tree_(m,3)+step_angle_*9);
        end

    
    ob_threshold = 0.5;%<1 >0.1
    if (plot_xy_mat(end,1) > (P_obstacles_(1) - ob_threshold))%%&&(plot_xy_mat(end,1) > (P_obstacles_(1)-1))
        f_endpoint = P_obstacles_(2) - ob_threshold;
    else
        f_endpoint = P_goal_conveyor(2) - ob_threshold;
    end
    
    f_endpoint = P_goal_(2) - ob_threshold;
    dis_3 = dot2_lines(plot_xy_mat, P_obstacles_);
    f_dis_3 = sum(dis_3 >= (ob_threshold+0.5));
    
    f_endplot = sum(plot_xy_mat(:,2) < f_endpoint);
    
    if (f_endplot==4)&&(f_dis_3==3)
        
        subplot(1,2,1)
        plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r', P_obstacles_(1), P_obstacles_(2), 'ko')
        axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
        
        subplot(1,2,2)
        qnew_ = [Q_near_;Q_new_];
        plot3(qnew_(:,1), qnew_(:,2), qnew_(:,3), 'r-')
        axis([-theta_range_box theta_range_box -theta_range_box ...
            theta_range_box -theta_range_box theta_range_box])
        box on
        drawnow;
        
        Q_tree_ = [Q_tree_; Q_new_];
        qtree_mat_(n, tree_index_+1) = 1;
        
        if (dis_end < 1)&&(nx==2)
            [q_trees_, n_start] = find_each_arm(qtree_mat_,tree_index_+1,Q_tree_);
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

%%
function dis_dl = dot2_lines(points4mat, dot)
xf=ones(1,4)*dot(1)>=min(points4mat(:,1))-0.5;
xff=ones(1,4)*dot(1)<=max(points4mat(:,1))+0.5;
yf=ones(1,4)*dot(2)>=min(points4mat(:,2))-0.5;
yff=ones(1,4)*dot(2)<=max(points4mat(:,2))+0.5;
for i=1:3
    a = points4mat(i,2)-points4mat(i+1,2);
    b = points4mat(i+1,1)-points4mat(i,1);
    c = points4mat(i,1)*points4mat(i+1,2)-points4mat(i+1,1)*points4mat(i,2);
    d = abs(a*dot(1)+b*dot(2)+c);
    d = d/sqrt(a*a+b*b);
    if ~b
        ydot = dot(2);
    else
        ydot = -a/b*(dot(1)-points4mat(i,1))+points4mat(i,2);
    end
    
    if (sum(xf)==4)&&(sum(xff)==4)&&(sum(yf)==4)&&(sum(yff)==4)
        if ydot<=dot(2)
            dis_dl(i) = d;
        else
            dis_dl(i) = -1000;
        end
    else
        dis_dl(i)=1000;
    end
end
end










