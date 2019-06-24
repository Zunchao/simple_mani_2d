function [] = sim_main_4endpoint_ik()
% rrt for endpoingt and plan for every point on the endpoint path
%
clf


l_joint_ = 10;

conveyor_xy =[-l_joint_*4 l_joint_*1.8;
    l_joint_*4 l_joint_*1.8];

P_goal_conveyor =[0.9*l_joint_*unifrnd(-2,2) l_joint_*1.8];

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
%random_angles_1 = unifrnd(0,pi);
%random_angles_2 = unifrnd(-pi/2,pi/2);
%random_angles_3 = unifrnd(-pi/2,pi/2);

random_angles_1 = unifrnd(qstart(1,1)-step_angle_, qstart(1,1)+step_angle_);
random_angles_2 = unifrnd(qstart(1,2)-step_angle_, qstart(1,2)+step_angle_);
random_angles_3 = unifrnd(qstart(1,3)-step_angle_, qstart(1,3)+step_angle_);

theta_range_box = theta_range_*2;

dis_end = sqrt(sum((pstart-pgoal).^2));
dis_end_mat(1)=dis_end;

stop_iteration = 200;

        qtree_mat_(1, 1) = 1;
while (dis_end > 0.5)
    
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
    %}
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
    f_dis_3 = sum(dis_3 >= 1.5);
    
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
    if iteration > stop_iteration
        break
    end
end

if iteration > stop_iteration
    disp('enter ik loop!')
    [ q ] = possible_solution_3joint_posigoal_forRRT( pgoal ,qstart(3));
    for mi=1:2
        dis_q(mi) = sum((q(mi+1,:) - qstart)).^2;
    end
    [minq,mj] = min(dis_q);
    [q_trees_, n_start] = twoq_merge(qstart,q(2,:), pobs);
else
    iteration;
    [q_trees_, n_start] = find_each_arm(qtree_mat_,tree_index_,Q_tree_);
    size(q_trees_);
end

end


%%
function [qtrees_, nstarts] = twoq_merge(Q_start_,Q_goal_, P_obstacles_)

l_joint_ = 10;

start_xy_mat = arm_vertex_mat(l_joint_, Q_start_);

goal_xy_mat = arm_vertex_mat(l_joint_, Q_goal_);

P_start_ = start_xy_mat(end,:);
P_goal_ = goal_xy_mat(end,:);

Q_start_tree_ = Q_start_;
Q_goal_tree_ = Q_goal_;

q_trees_goals = Q_start_;
q_trees_start = Q_goal_;
qtrees_= [q_trees_goals;q_trees_start];
nstarts = 2;

step_angle_ = pi/50;
iteration = 1;
stree_index_ = 1;
gtree_index_ = 1;

range_angles_1 = [Q_start_(1), Q_goal_(1)];
range_angles_2 = [Q_start_(2), Q_goal_(2)];
range_angles_3 = [Q_start_(3), Q_goal_(3)];

random_angles_1 = unifrnd(min(range_angles_1)-step_angle_, max(range_angles_1)+step_angle_);
random_angles_2 = unifrnd(min(range_angles_2)-step_angle_, max(range_angles_2)+step_angle_);
random_angles_3 = unifrnd(min(range_angles_3)-step_angle_, max(range_angles_3)+step_angle_);


f_angles_1 = -(Q_start_(1)-Q_goal_(1))/abs(Q_start_(1)-Q_goal_(1));
f_angles_2 = -(Q_start_(2)-Q_goal_(2))/abs(Q_start_(2)-Q_goal_(2));
f_angles_3 = -(Q_start_(3)-Q_goal_(3))/abs(Q_start_(3)-Q_goal_(3));

qtree_mat_goal(1,1)=1;
qtree_mat_start(1,1)=1;

while (iteration < 20000)
    
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    
    for js = 1:stree_index_
        dis_start_(js) = sum((random_angles_ - Q_start_tree_(js,:)).^2);
    end
    
    for jg = 1:gtree_index_
        dis_goal_(jg) = sum((random_angles_ - Q_goal_tree_(jg,:)).^2);
    end
    
    [min_dis_start,n] = min(dis_start_);
    Q_near_start = Q_start_tree_(n,:);
    Q_new_start = cal_new_(Q_near_start, random_angles_, step_angle_);
    splot_xy_mat = arm_vertex_mat(l_joint_, Q_new_start);
    qnew_start = [Q_near_start;Q_new_start];
    
    [min_dis_goal,m] = min(dis_goal_);
    Q_near_goal = Q_goal_tree_(m,:);
    Q_new_goal = cal_new_(Q_near_goal, random_angles_, step_angle_);
    gplot_xy_mat = arm_vertex_mat(l_joint_, Q_new_goal);
    qnew_goal = [Q_near_goal;Q_new_goal];
    
    
    dis_3 = dot2_lines(splot_xy_mat, P_obstacles_);
    f_dis_s = sum(dis_3 >= 1.5);
    dis_3 = dot2_lines(gplot_xy_mat, P_obstacles_);
    f_dis_g = sum(dis_3 >= 1.5);
    
    f_endplots = sum(splot_xy_mat(:,2) < 17.8);
    f_endplotg = sum(gplot_xy_mat(:,2) < 17.8);
    
    if (f_endplots==4)&&(f_dis_s==3)
        
        subplot(1,2,1)
        plot(splot_xy_mat(4,1), splot_xy_mat(4,2), 'r')
        
        subplot(1,2,2)
        plot3(qnew_start(:,1), qnew_start(:,2), qnew_start(:,3), 'r-')
        Q_start_tree_ = [Q_start_tree_; Q_new_start];
        qtree_mat_start(n, stree_index_+1) = 1;
        stree_index_ = stree_index_+1;
    end
    
    if (f_endplotg==4)&&(f_dis_g==3)
        
        subplot(1,2,1)
        plot(gplot_xy_mat(4,1), gplot_xy_mat(4,2), 'b')
        
        subplot(1,2,2)
        plot3(qnew_goal(:,1), qnew_goal(:,2), qnew_goal(:,3), 'b-')
        Q_goal_tree_ = [Q_goal_tree_; Q_new_goal];
        qtree_mat_goal(m, gtree_index_+1) = 1;
        gtree_index_ = gtree_index_+1;
    end
    
    dis_Qend = sqrt(sum((Q_new_goal-Q_new_start).^2));
    %dis_Qend = sqrt(sum((gplot_xy_mat(end,:)-splot_xy_mat(end,:)).^2))
    if dis_Qend < step_angle_*3
        [q_trees_start, n_start] = find_each_arm(qtree_mat_start,stree_index_,Q_start_tree_);
        size(q_trees_start);
        [q_trees_goal, n_goal] = find_each_arm(qtree_mat_goal,gtree_index_,Q_goal_tree_);
        nk=size(q_trees_goal,1);
        for mk=1:nk
            q_trees_goals(mk,:) = q_trees_goal(nk-mk+1,:);
        end
        q_trees_=[q_trees_goals;q_trees_start];
        n_starts = n_start+n_goal;
        iteration;
        break
    end
    
    range_angles_1 = [Q_start_(1), Q_goal_(1)];
    range_angles_2 = [Q_start_(2), Q_goal_(2)];
    range_angles_3 = [Q_start_(3), Q_goal_(3)];
    
    random_angles_1 = unifrnd(min(range_angles_1)-step_angle_*2, max(range_angles_1)+step_angle_*2);
    random_angles_2 = unifrnd(min(range_angles_2)-step_angle_*5, max(range_angles_2)+step_angle_*5);
    random_angles_3 = unifrnd(min(range_angles_3)-step_angle_*5, max(range_angles_3)+step_angle_*5);

    random_angles_1 = unifrnd(0,pi);
random_angles_2 = unifrnd(-pi,pi);
random_angles_3 = unifrnd(-pi,pi);

    iteration = iteration + 1;
    
    %
end
qtrees_= q_trees_;
nstarts = n_starts;
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
    if b==0
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










