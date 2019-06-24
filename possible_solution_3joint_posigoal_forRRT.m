function [ q ] = possible_solution_3joint_posigoal_forRRT( positiongoal ,qcurrent)
%UNTITLED6 Summary of this function goes here
ni=1001;

theta3 = linspace(qcurrent-pi/18,qcurrent+pi/18,20);
for j=1:10
    theta = linspace(0-theta3(j),pi-theta3(j),ni);
    for i = 2:ni-1
        p = [positiongoal,theta(i)+theta3(j)];
        fik_3_joints = ik_3joints_mani(p);
        if isreal(fik_3_joints)
            break
        end
    end
    q=[positiongoal,theta(i);fik_3_joints];
    i;
    j;
end
end