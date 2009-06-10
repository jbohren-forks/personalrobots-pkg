function playbackSolution(solution, cfg, actions, selected_actions)

coords = find(cfg.environment~=0);

[h, w] = ind2sub(size(cfg.environment), coords);

figure();hold on;
% for i=1:length(w)
%     pts = [w(i) h(i); w(i)+1 h(i); w(i)+1 h(i)+1; w(i) h(i)+1; w(i) h(i)];
%     pts = pts .* cfg.cellsize;
%     plot(pts(:,1), pts(:,2),'r');
% end
plot(w.*cfg.cellsize, h.*cfg.cellsize, '.k');
%plot(w.*cfg.cellsize + ones(size(w)).*cfg.cellsize/2, h.*cfg.cellsize + ones(size(h)).*cfg.cellsize/2, '+k');
%plot(w, h, '+k');


% footprint = cfg.footprint;
% 
% 
% prev_location = [0 0 0];
% for i=1:size(selected_actions,1)
%     footprint_size = actions(selected_actions(i,1)+1).action_list(selected_actions(i,2)+1).footprint_size;
%     footprint_discrete = actions(selected_actions(i,1)+1).action_list(selected_actions(i,2)+1).footprint;
%     pts = []; 
%     for findx=1:footprint_size
%         pts = [pts; footprint_discrete(findx).x*cfg.cellsize footprint_discrete(findx).y*cfg.cellsize footprint_discrete(findx).theta*360/cfg.num_theta];
%     end
%     
%     pts = pts*[cos(prev_location(3)) sin(prev_location(3)); -sin(prev_location(3)) cos(prev_location(3))];
%     pts(:,1) = pts(:,1) + ones(size(pts,1),1)*prev_location(1);
%     pts(:,2) = pts(:,2) + ones(size(pts,1),1)*prev_location(2);
%     plot(pts(:,1), pts(:,2), '.g');
%     prev_location = pts(end,:);
%     pl = actions(selected_actions(i,1)+1).action_list(selected_actions(i,2)+1).path_length_rw;
%     prev_location = [prev_location actions(selected_actions(i,1)+1).action_list(selected_actions(i,2)+1).path_rw(pl).theta];
% end

% for i=1:4%size(solution.path,1)    
%     trans_footprint = footprint*[cos(solution.path(i,3)) sin(solution.path(i,3)); -sin(solution.path(i,3)) cos(solution.path(i,3))];
%     trans_footprint(:,1) = trans_footprint(:,1) + ones(size(trans_footprint,1),1)*solution.path(i,1);
%     trans_footprint(:,2) = trans_footprint(:,2) + ones(size(trans_footprint,1),1)*solution.path(i,2);
%     plot(trans_footprint(:,1), trans_footprint(:,2));
% end

plot(solution.path(:,1), solution.path(:,2), '.m');