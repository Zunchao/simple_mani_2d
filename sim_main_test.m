function [ output_args ] = sim_main_test()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
ni = 7;
nj = 9;
nk = 19;
l_joint = 10;

theta_1_joint = linspace(0,pi,ni);
theta_2_joint = linspace(-pi/2,pi/2,nj);
theta_3_joint = linspace(-pi/2,pi/2,nk);

cla

for i = 1:ni
    T1 = trans_Joint( theta_1_joint(i), l_joint );
    px1 = [0, T1(1,end)];
    py1 = [0, T1(2,end)];
    plot(T1(1,end), T1(2,end), 'bo')
    axis([-l_joint*3 l_joint*3 -l_joint*3 l_joint*3])
    hold on
    for j = 1:nj
        T12 = trans2jointsxy(l_joint, theta_1_joint(i), theta_2_joint(j))%trans_Joint( theta_2_joint(j), l_joint );
        %T2 = T1*T12;
        %px2 = [T1(1,end), T2(1,end)];
        %py2 = [T1(2,end), T2(2,end)];
        plot(T12(1), T12(2), 'r.')
        
        for k = 1:nk
            T13 = trans3jointsxy(l_joint, theta_1_joint(i), theta_2_joint(j), theta_3_joint(k))%trans_Joint( theta_3_joint(k), l_joint );
            %T3 = T2*T13;
            %px2 = [T2(1,end), T3(1,end)];
            %py2 = [T2(2,end), T3(2,end)];
            plot(T13(1), T13(2), 'g.')
            drawnow
        end
    end
end
end

