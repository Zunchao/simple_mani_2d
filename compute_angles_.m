%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算角度分程序：
function Angle = compute_angles_(Ppath,Pgoa_obs)
%Angle是斥力与x轴+的角度向量,Pstart是起点坐标，Pgoa_obs是目标或者障碍的坐标向量
n = size(Pgoa_obs,1);
for i=1:n%n是障碍数目
    deltaX(i)=Pgoa_obs(i,1)-Ppath(1);
    deltaY(i)=Pgoa_obs(i,2)-Ppath(2);
    r(i)=sqrt(deltaX(i)^2+deltaY(i)^2);
    if deltaY(i)>0
        Xtheta = acos(deltaX(i)/r(i));
    else
        Xtheta = -acos(deltaX(i)/r(i));
    end
    %Xtheta = acos(deltaX(i)/r(i));
    Angle(i)=Xtheta;
    %保存每个角度在Angle向量里面，表示与目标的角度或者与障碍的角度
end