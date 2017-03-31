function qc = constrainConfig(qs_old,qs, pgoal,step)
%qs=[0.221136391637154   0.525958654414750   0.245893898061861];
%pgoal=[0 18];
while 1
    
    C=[-1 35;
        -1 0;
        0 pi];

    Tc0=eye(3);
    Tc0(1,3) = pgoal(1);
    Tc0(2,3) = pgoal(2);
    
    Te0 = transmatrix_of_multijoints(qs);
    
    T0c=inv(Tc0);
    Tec=T0c*Te0;
    orient_angle=sum(qs);
    
    dc = [Tec(1,end) Tec(2,end) orient_angle];
    
    for i = 1:3
        if dc(i)>max(C(i,:))
            deltax(i) = dc(i)-max(C(i,:));
        elseif dc(i)<min(C(i,:))
            deltax(i) = dc(i)-min(C(i,:));
        else
            deltax(i) = 0;
        end
    end
    
    ndeltax= norm(deltax);
    if ndeltax<0.01
        qc=qs;
        break
    end
    
    Theta1=qs(1);
    Theta2=qs(2);
    Theta3=qs(3);
    J0 = trans3jointsxy_jacobian(10, Theta1, Theta2, Theta3);
    
    deltaqerror = J0'*inv(J0*J0')*deltax';
    
    qs = qs - deltaqerror';
    
    flag_q=outofrenge(qs);

    if norm(qs-qs_old)>2*step
        qc = [];
        break;
    end
    if ~flag_q
        qc = [];
        break;
    end
    
end

end
%%
function flag_q=outofrenge(q)
n = max(size(q));
for i = 1:n
    fq(i)=(q(i)>-pi)&&(q(i)<pi);
end

if sum(fq)==n
    flag_q = 1;
else
    flag_q = 0;
end
end
