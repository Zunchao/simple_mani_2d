function [] = sim_main_TSR_stilman()
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

P_goal_conveyor =[l_joint_*unifrnd(-2,2) l_joint_*1.8];

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
    random_angles_1 = unifrnd(-theta_range_,theta_range_);
    random_angles_2 = unifrnd(-theta_range_,theta_range_);
    random_angles_3 = unifrnd(-theta_range_,theta_range_);
    random_angles_ = [random_angles_1, random_angles_2, random_angles_3];
    
    for j = 1:iteration
        dis_(j) = sum((random_angles_ - Q_tree_(j,:)).^2);
    end
    [min_dis_,n] = min(dis_);
    Q_near_ = Q_tree_(n,:);
    
    Q_new_ = cal_new_(Q_near_, random_angles_, step_angle_);
    
    [flag_new,qsnew] = fr_new_config(Q_new_,Q_near_,P_goal_conveyor)
    Q_new_=qsnew;
    if flag_new
        Q_tree_ = [Q_tree_; Q_new_];
        qtree_mat_(n, tree_index_+1) = 1;
    end
    
    plot_xy_mat_near = arm_vertex_mat(l_joint_, Q_near_);
    plot_xy_mat = arm_vertex_mat(l_joint_, Q_new_);
    
    %dis_end_second = sqrt(sum((plot_xy_mat_near(end,:)-P_goal_conveyor).^2));
    dis_end = sqrt(sum((plot_xy_mat(end,:)-P_goal_conveyor).^2));
    
    dis_end_mat(iteration+1) = dis_end;
    [min_dis_end,m] = min(dis_end_mat(1:end-1));
    
    
    if flag_new==1%(plot_xy_mat(:,2)<l_joint_*1.75)
        
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



function [flag_,qs2] = fr_new_config(qs,qnear,pgoal)
qr=qs;
delta_xerror = computertaskerror(qs,pgoal);
norm(delta_xerror)
flag_ = 1;
while norm(delta_xerror)>0.00001
    qs = trtractconfig(qs,delta_xerror);
    if norm(qs-qr)>norm(qr-qnear)
        flag_ = 0;
        break;
    end
    delta_xerror = computertaskerror(qs,pgoal);    
end

qs2=qs;
end

function delta_xerror = computertaskerror(qs,pgoal)
C=eye(6);

C(1,1) = 0;
C(2,2) = 1;
C(6,6) = 0;

Tt0=eye(4);
Tt0(1,4) = pgoal(1);
Tt0(2,4) = pgoal(2);

Te0 = trans_of_multijoints_full(qs);

T0t=inv(Tt0);
Tet=T0t*Te0;
orient_angle=sum(qs);

delta_x=[Tet(1,end) Tet(2,end) Tet(3,end) atan2(Tet(3,2),Tet(3,3)) -asin(Tet(3,1)) atan2(Tet(2,1),Tet(1,1))]'
delta_xerror=C*delta_x;
delta_xerror=delta_xerror';

end

function qs1=trtractconfig(qs,delta_xerror)
Theta1=qs(1);
Theta2=qs(2);
Theta3=qs(3);
J0 = trans3jointsxy_jacobian_full(10,Theta1, Theta2, Theta3)
Jt=J0;
%orientation of task frame is yaw =0, pitch=0, roll=0;
Erpy=eye(6);
J=Erpy*Jt;
%Jp=J'*inv(J*J');
Jp=pinv(J);
delta_qerr=Jp*delta_xerror';
qs1=qs-delta_qerr';
end


