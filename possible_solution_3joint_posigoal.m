function [ q ] = possible_solution_3joint_posigoal( positiongoal )
%UNTITLED6 Summary of this function goes here
ni=10001;
theta = linspace(0,pi,ni);
for i = 2:ni-1
    p = [positiongoal,theta(i)];
    fik_3_joints = ik_3joints_mani(p);
    if isreal(fik_3_joints)
        break
    end
end
q=[positiongoal,theta(i);fik_3_joints];
i;

for j =2:3
    plot_xy_mat = arm_vertex_mat(10, q(j,:));
    plot(plot_xy_mat(:,1),plot_xy_mat(:,2),'g.-');
    hold on
end

plot(q(1,1),q(1,2),'ko');
axis([-40 40 -40 40])
%{
theta = 0;
p = [positiongoal,theta];
fik_3_joints = ik_3joints_mani(p);
while ~isreal(fik_3_joints(1,1))
    theta = unifrnd(-pi/2,pi/2)
    fik_3_joints = ik_3joints_mani(p);
    isreal(fik_3_joints(1,1))
end

q=fik_3_joints;
%}
end