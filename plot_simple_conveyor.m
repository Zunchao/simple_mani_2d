function plot_simple_conveyor(L)
% draw the conveyor line
dots = [-L*4 L*1.5;
    L*4 L*1.5];
plot(dots(:,1), dots(:,2), 'b--')
end

