%%
function [q_trees_, n_start] = find_each_path(qtree_matrics,n,qtree)
% find a path in the tree
%ni=size(qtree_matrics,2);
ni=n;
nj=1;
if (ni==1)
    q_tree_ = qtree(1,:);
    nj=2;
end

while ni>1
    mi = find(qtree_matrics(:,ni)==1);
    qtree_ = [qtree(mi,:);qtree(ni,:)];
    %subplot(1,2,1)
    %plot(qtree_(:,1),qtree_(:,2),'r-');
    q_tree_(nj,:) = qtree(ni,:);
    ni = mi;
    nj = nj+1;
    if (ni==1)
        q_tree_(nj,:) = qtree(ni,:);
        break;
    end
end
n_start = nj;
q_trees_ = q_tree_;
end


