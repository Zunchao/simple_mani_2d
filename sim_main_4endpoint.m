function [] = sim_main_4endpoint()
% rrt for endpoingt and plan for every point on the endpoint path
%
clf


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
plot(conveyor_xy(:,1), conveyor_xy(:,2),'k--',P_goal_conveyor(1),P_goal_conveyor(2),'k*')
title('Work space')

subplot(1,2,2)
plot3(Q_init_(1,1), Q_init_(1,2), Q_init_(1,3), 'bo')
%axis([-theta_range_ theta_range_ -theta_range_ theta_range_ -theta_range_ theta_range_])
hold on
title('C-space')

P_goal_arm = start_xy_mat(4,:);


P_obstacles_ = [P_goal_conveyor(1)+(30-P_goal_conveyor(1))*rand(1), P_goal_conveyor(2)*rand(1)];

%pgoal_1= [P_obstacles_(1,1)-2,(P_obstacles_(1,2)+P_goal_conveyor(2))/2];
%[q_trees_, n_start]=step_start_goal(P_goal_arm, pgoal_1, P_obstacles_, Q_init_, l_joint_);

%Q_init_=q_trees_(end,:);
[Q_trees_,D_start_new] = endpoint_rrt_merge(P_goal_arm, P_goal_conveyor, P_obstacles_);
%size(Q_trees_)
%Q_trees_=[Q_trees_(1,:);Q_trees_(9,:);Q_trees_(end,:)];
%step_start_goal(P_goal_arm, P_goal_conveyor, P_obstacles_, Q_init_, l_joint_);
Qstart=Q_init_;
qtree_init=Q_init_;

for itree= 1:size(Q_trees_,1)-1
    plot_a_tree(Q_trees_)
    [q_trees_, n_start]=step_start_goal(Q_trees_(itree,:), Q_trees_(itree+1,:), P_obstacles_, Qstart, l_joint_);
    Qstart=q_trees_(1,:);
    qtree_init=[q_trees_(1:n_start-1,:);qtree_init];
end

qtree_init=[q_trees_(end,:);qtree_init];
n_fin=size(qtree_init,1);

for km=1:n_fin
    % draw the final arm path
    
    subplot(1,2,1)
    plot_xy_mat = arm_vertex_mat(l_joint_, qtree_init(n_fin-km+1,:));
    plot(plot_xy_mat(:,1),plot_xy_mat(:,2),'g.-', P_obstacles_(1), P_obstacles_(2), 'ko');
    axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
    drawnow
    hold on
    
    % draw the final jointstate path
    subplot(1,2,2)
    if km < n_fin
        qtree_2points = [qtree_init(n_fin-km+1,:); qtree_init(n_fin-km,:)];
        plot3(qtree_2points(:,1), qtree_2points(:,2), qtree_2points(:,3), 'g.-')
    end
    
end

end

function [q_trees_, n_start]=step_start_goal(pstart, pgoal, pobs, qstart, l_joint)

P_goal_=pgoal;
P_obstacles_=pobs;
Q_tree_ = qstart;

step_angle_ = pi/100;
iteration = 1;
tree_index_ = 1;

theta_range_ = pi/2;
random_angles_1 = unifrnd(0,pi);
random_angles_2 = unifrnd(-pi/2,pi/2);
random_angles_3 = unifrnd(-pi/2,pi/2);

random_angles_1 = unifrnd(qstart(1,1)-step_angle_, qstart(1,1)+step_angle_);
random_angles_2 = unifrnd(qstart(1,2)-step_angle_, qstart(1,2)+step_angle_);
random_angles_3 = unifrnd(qstart(1,3)-step_angle_, qstart(1,3)+step_angle_);

theta_range_box = theta_range_*2;

dis_end = sqrt(sum((pstart-pgoal).^2));
dis_end_mat(1)=dis_end;

while (dis_end > 0.3)
    
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    start_xy_mat = arm_vertex_mat(l_joint, random_angles_);
    
    for j = 1:iteration
        dis_(j) = sum((random_angles_ - Q_tree_(j,:)).^2);
    end
    [min_dis_,n] = min(dis_);
    Q_near_ = Q_tree_(n,:);
    
    %plot_xy_matnear = arm_vertex_mat(l_joint, Q_near_);
    %pointnear=plot_xy_matnear(end,:);
    %xdata=(pointnear-pgoal)/sqrt(sum((pointnear-pgoal).^2));
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_);
    
    plot_xy_mat = arm_vertex_mat(l_joint, Q_new_);
    
    dis_end = sqrt(sum((plot_xy_mat(end,:)-P_goal_).^2));
    
    dis_end_mat(iteration+1) = dis_end;
    [min_dis_end,m] = min(dis_end_mat(1:end-1));
    
    f_dis = (dis_end < min_dis_end);
    %{
    fiteration = mod(iteration,3);
    if  f_dis && (fiteration==0)
        random_angles_1 = unifrnd(Q_new_(1)-step_angle_, Q_new_(1)+step_angle_);
    else
        random_angles_1 = unifrnd(Q_tree_(m,1)-step_angle_, Q_tree_(m,1)+step_angle_);
    end
    
    if  f_dis && (fiteration==1)
        random_angles_2 = unifrnd(Q_new_(2)-step_angle_*3, Q_new_(2)+step_angle_*3);
    else
        random_angles_2 = unifrnd(Q_tree_(m,2)-step_angle_*3, Q_tree_(m,2)+step_angle_*3);
    end
    
    if  f_dis && (fiteration==2)
        random_angles_3 = unifrnd(Q_new_(3)-step_angle_*6, Q_new_(3)+step_angle_*6);
    else
        random_angles_3 = unifrnd(Q_tree_(m,3)-step_angle_*6, Q_tree_(m,3)+step_angle_*6);
    end
    %}
    f_min_new = (Q_new_(1)-step_angle_ > theta_range_-pi/2) && ...
        (Q_new_(2)-step_angle_*6 > -theta_range_) && ...
        (Q_new_(3)-step_angle_*9 > -theta_range_);
    f_max_new = (Q_new_(1)+step_angle_ < theta_range_+pi/2) && ...
        (Q_new_(2)+step_angle_*6 < theta_range_) && ...
        (Q_new_(3)+step_angle_*9 < theta_range_);
    
    f_min_near = (Q_tree_(m,1)-step_angle_ > theta_range_-pi/2) && ...
        (Q_tree_(m,2)-step_angle_*6 > -theta_range_) && ...
        (Q_tree_(m,3)-step_angle_*9 > -theta_range_);
    f_max_near = (Q_tree_(m,1)+step_angle_ < theta_range_+pi/2) && ...
        (Q_tree_(m,2)+step_angle_*6 < theta_range_) && ...
        (Q_tree_(m,3)+step_angle_*9 < theta_range_);
    
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

    
    dis_3 = dot2_lines(plot_xy_mat, P_obstacles_);
    f_dis_3 = sum(dis_3 >= 1);
    
    f_endplot = sum(plot_xy_mat(:,2) < 17.8);
    
    if (f_endplot==4)&&(f_dis_3==3)
        
        subplot(1,2,1)
        plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r', P_obstacles_(1), P_obstacles_(2), 'ko')
        axis([-l_joint*4 l_joint*4 -l_joint*4 l_joint*4])
        
        subplot(1,2,2)
        qnew_ = [Q_near_;Q_new_];
        plot3(qnew_(:,1), qnew_(:,2), qnew_(:,3), 'r-')
        axis([-theta_range_box theta_range_box -theta_range_box ...
            theta_range_box -theta_range_box theta_range_box])
        box on
        drawnow;
        
        Q_tree_ = [Q_tree_; Q_new_];
        qtree_mat_(n, tree_index_+1) = 1;
        
        iteration = iteration + 1;
        tree_index_ = tree_index_+1;
        
    end
    
end
iteration;
[q_trees_, n_start] = find_each_arm(qtree_mat_,tree_index_,Q_tree_);
size(q_trees_);
end


%%
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










