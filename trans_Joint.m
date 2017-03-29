function T = trans_Joint( Theta, L )
% TRANSFORM MATRIX FOR ON JOINT
%{ 
R1 = [cos(Theta) sin(Theta) 0;
    -sin(Theta) cos(Theta) 0;
    0 0 1];
T2 = [1 0 0;
    0 1 L;
    0 0 1];

T = R1*T2;
%}
T = [cos(Theta) sin(Theta) L*cos(Theta);
    -sin(Theta) cos(Theta) L*sin(Theta);
    0 0 1];
end

