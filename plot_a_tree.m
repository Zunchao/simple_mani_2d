%%
function plot_a_tree(tree)
% plot the path
for i =1:(size(tree,1)-1)
    qtree_ = [tree(i,:);tree(i+1,:)];
    subplot(1,2,1)
    plot(qtree_(:,1),qtree_(:,2),'b.-');
    hold on
end
end
