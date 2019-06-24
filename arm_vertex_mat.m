function plot_xy_mat = arm_vertex_mat(L, Theta_vec)
% 3-joint-arm joint endpoints
trans_1_joints_xy = trans1jointsxy(L, Theta_vec(1));
trans_2_joints_xy = trans2jointsxy(L, Theta_vec(1), Theta_vec(2));
trans_3_joints_xy = trans3jointsxy(L, Theta_vec(1), Theta_vec(2), Theta_vec(3));

plot_xy_mat = [ 0, 0;
    trans_1_joints_xy(1), trans_1_joints_xy(2);
    trans_2_joints_xy(1), trans_2_joints_xy(2);
    trans_3_joints_xy(1), trans_3_joints_xy(2)];
end
