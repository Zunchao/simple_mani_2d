function [] = sim_main_rrt_inTSR()
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

conveyor_xy =[-l_joint_*4 l_joint_*1.8;
    l_joint_*4 l_joint_*1.8];

P_goal_conveyor =[l_joint_*unifrnd(-2,2)*0.75 l_joint_*1.8];
P_goal_conveyor =[-10 18];
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
axis([0 pi -pi pi -pi pi])
hold on
title('C-space')

step_angle_ = pi/50;

theta_range_box = theta_range_*2;
halfpathTree_ = halfpath_to_TSR_dev(Q_init_,P_goal_conveyor,step_angle_,l_joint_);

TSRstart_= halfpathTree_(1,:);

TSRstart_xy_mat = arm_vertex_mat(l_joint_, TSRstart_);

P_goal_arm = TSRstart_xy_mat(4,:);
dis_end = sqrt(sum((P_goal_arm-P_goal_conveyor).^2));
dis_end_mat(1) = dis_end;

Q_tree_2 = TSRstart_;
iteration = 1;
tree_index_ = 1;

random_angles_1 = unifrnd(Q_tree_2(1,1)-step_angle_, Q_tree_2(1,1)+step_angle_);
random_angles_2 = unifrnd(Q_tree_2(1,2)-step_angle_*2, Q_tree_2(1,2)+step_angle_*2);
random_angles_3 = unifrnd(Q_tree_2(1,3)-step_angle_*3, Q_tree_2(1,3)+step_angle_*3);

iterationc=1;

f_dis_end=0.2;

while (dis_end>f_dis_end/2)
    
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    
    for j = 1:iterationc
        dis_(j) = sum((random_angles_ - Q_tree_2(j,:)).^2);
    end
    
    [min_dis_,n] = min(dis_);
    Q_near_ = Q_tree_2(n,:);
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_)
    
    Q_new_ = constrainConfig(Q_near_,Q_new_, P_goal_conveyor,step_angle_)
    
    if ~isempty(Q_new_)
        plot_xy_mat = arm_vertex_mat(l_joint_, Q_new_);
        dis_end = sqrt(sum((plot_xy_mat(end,:)-P_goal_conveyor).^2));
        
        if (plot_xy_mat(:,2)<P_goal_conveyor(2))
            
            subplot(1,2,1)
            plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r')
            axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
            
            subplot(1,2,2)
            qnew_ = [Q_near_;Q_new_];
            plot3(qnew_(:,1), qnew_(:,2), qnew_(:,3), 'r-')
            axis([0 pi -pi pi -pi pi])
            box on
            drawnow
            
            Q_tree_2 = [Q_tree_2; Q_new_];
            qtree_mat_2(n, tree_index_+1) = 1;
            iterationc=iterationc+1;
            tree_index_ = tree_index_+1;            
            end
            if dis_end < f_dis_end
                [q_trees_, n_start] = find_each_arm(qtree_mat_2,tree_index_,Q_tree_2);
                size(q_trees_)
                for k=1:n_start
                    % draw the final arm path
                    subplot(1,2,1)
                    path_xy_mat = arm_vertex_mat(l_joint_, q_trees_(n_start-k+1,:));
                    plot(path_xy_mat(:,1),path_xy_mat(:,2),'g.-');
                    %axis([-l_joint_*4 l_joint_*4 -l_joint_*4 l_joint_*4])
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
        
    end
    random_angles_1 = unifrnd(0,pi);
    random_angles_2 = unifrnd(-pi,pi);
    random_angles_3 = unifrnd(-pi,pi);
    
    iteration = iteration + 1;
end
%}
iteration;
dis_end
end

%%
function [tree_halfpath_] = halfpath_to_TSR_dev(qstart,pgoal,step_a, ljoint_)
%

start_xy_mat = arm_vertex_mat(ljoint_, qstart);

dis_end = sqrt(sum((start_xy_mat(4,:)-pgoal).^2));
dis_end_mat(1) = dis_end;

qtree_ = qstart;
iteration = 1;
tree_index_ = 1;

random_angles_1 = unifrnd(0, pi);
random_angles_2 = unifrnd(-pi/2, pi/2);
random_angles_3 = unifrnd(-pi/2, pi/2);

while (dis_end>0.1)
    
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    
    for j = 1:iteration
        dis_(j) = sum((random_angles_ - qtree_(j,:)).^2);
    end
    
    [min_dis_,n] = min(dis_);
    qnear_ = qtree_(n,:);
    
    qnew_ = cal_new_(qnear_, random_angles_, step_a);
    
    plot_xy_mat_near = arm_vertex_mat(ljoint_, qnear_);
    plot_xy_mat = arm_vertex_mat(ljoint_, qnew_);
    
    dis_end = abs(plot_xy_mat(end,2)-pgoal(2));
    
    dis_end_mat(iteration+1) = dis_end;
    [min_dis_end,m] = min(dis_end_mat(1:end-1));
    
    f_dis = (dis_end < min_dis_end);
    
    if  f_dis
        random_angles_1 = unifrnd(qnew_(1)-step_a, qnew_(1)+step_a);
        random_angles_2 = unifrnd(qnew_(2)-step_a*2, qnew_(2)+step_a*2);
        random_angles_3 = unifrnd(qnew_(3)-step_a*3, qnew_(3)+step_a*3);
    else
        random_angles_1 = unifrnd(qtree_(m,1)-step_a, qtree_(m,1)+step_a);
        random_angles_2 = unifrnd(qtree_(m,2)-step_a*2, qtree_(m,2)+step_a*2);
        random_angles_3 = unifrnd(qtree_(m,3)-step_a*3, qtree_(m,3)+step_a*3);
    end
    
    if (plot_xy_mat(:,2)<pgoal(2))
        
        subplot(1,2,1)
        plot(plot_xy_mat(4,1), plot_xy_mat(4,2), 'r')
        %axis([-ljoint_*4 ljoint_*4 -ljoint_*4 ljoint_*4])
        
        subplot(1,2,2)
        qnew_s = [qnear_;qnew_];
        plot3(qnew_s(:,1), qnew_s(:,2), qnew_s(:,3), 'r-')
        axis([-pi pi -pi pi -pi pi])
        box on
        drawnow
        
        qtree_ = [qtree_; qnew_];
        qtree_mat_(n, tree_index_+1) = 1;
        
        iteration = iteration + 1;
        tree_index_ = tree_index_+1;
        
    end
    
end

[qtrees_, n_start] = find_each_arm(qtree_mat_,tree_index_,qtree_);

for k=1:n_start
    % draw the final arm path
    subplot(1,2,1)
    plot_xy_mat = arm_vertex_mat(ljoint_, qtrees_(n_start-k+1,:));
    plot(plot_xy_mat(:,1),plot_xy_mat(:,2),'g.-');
    drawnow
    
    % draw the final jointstate path
    subplot(1,2,2)
    if k < n_start
        qtree_2points = [qtrees_(n_start-k+1,:); qtrees_(n_start-k,:)];
        plot3(qtree_2points(:,1), qtree_2points(:,2), qtree_2points(:,3), 'g.-')
    end
    
end

tree_halfpath_=qtrees_;

end

%%
function Q_new_ = extent_new_(tnear_, prand_, ori_rand)
i=1;
pi(i,:)=tnear_(1,4:5);
deltapi=[0,1];
v=[1,0];
pd=prand_;

delta_pdi=prand_-abs(dot((prand_-pi(i,:)),deltapi))*deltapi
vi=delta_pdi-pi(i,:)
v=vi/norm(vi)
pi(i+1,:)=pi(i,:)+v*0.01

end




