function new_angles_ = cal_new_(current_angles_, random_angels_, step_angle_)
% create new node on the tree
inum = size(random_angels_,2);

delta_angle = random_angels_ - current_angles_;
%trans_3_joints_xy_jacobian = trans3jointsxy_jacobian(10, delta_angle(1),delta_angle(2),delta_angle(3))
%xdata=xdata'
%trans_3_joints_xy_jacobian=trans_3_joints_xy_jacobian'
%norm_del = trans_3_joints_xy_jacobian*xdata
norm_del = delta_angle/norm(delta_angle);
%minval=min([step_angle_,norm(delta_angle)]);
new_angles_ = current_angles_ + min([step_angle_,norm(delta_angle)])*norm_del;

%{
for i = 1:inum
    if (random_angels_(i) < current_angles_(i))
        new_angles_(i) = current_angles_(i) - step_angle_;
    else
        new_angles_(i) = current_angles_(i) + step_angle_;
    end
end
%}
end