function playActions(actions)

action_names = [{'Turn in place left'},...
        {'Turn in place right'},...
        {'rv=-1.57 rad/sec  tv=-1.0 m/s dt=2.0 sec'},...
        {'rv=-1.57 rad/sec  tv=1.0 m/s  dt=2.0 sec'},...
        {'rv=-0.785 rad/sec  tv=-1.0 m/s  dt=2.0 sec'},...
        {'rv=-0.785 rad/sec  tv=1.0 m/s  dt=2.0 sec'},...
        {'rv=0 rad/sec  tv=-1.0 m/s  dt=2.0 sec'},...
        {'rv=0 rad/sec  tv=-1.0 m/s  dt=0.5 sec'},...
        {'rv=0 rad/sec  tv=1.0 m/s  dt=2.0 sec'},...
        {'rv=0 rad/sec  tv=1.0 m/s  dt=0.5 sec'},...
        {'rv=0.785 rad/sec  tv=-1.0 m/s  dt=2.0 sec'},...
        {'rv=0.785 rad/sec  tv=1.0 m/s  dt=2.0 sec'},...
        {'rv=1.57 rad/sec  tv=-1.0 m/s  dt=2.0 sec'},...
        {'rv=1.57 rad/sec  tv=1.0 m/s  dt=2.0 sec'}];
subplot_cols = 2;
subplot_rows = 7;

figure(); hold on; %trajectories
%figure(2); %axis([-5 5 -5 5]);hold on; %footprint
 %for tind=1:length(actions)
 tind =1;
    action_list = [1, 14, 12, 10, 9];
  %offset = [-50 20; 0 20; 60 20; -25 -60; 25 -60];
  offset = [-50 60; 0 60; 60 60; -25 -20; 25 -20];    
    for a=1:length(action_list);
    
%    for aind=1:length(actions(tind).action_list)
    aind = action_list(a) ;
    figure(1); axis([-80 80 -90 90]);hold on;axis equal; %trajectories
     title(action_names(aind));
    fprintf('Action: %d, Size of footprint: %d\n', aind, actions(tind).action_list(aind).footprint_size + actions(tind).action_list(aind).footprint_circle_size);
    xlab = sprintf('Size of footprint: %d', actions(tind).action_list(aind).footprint_size + actions(tind).action_list(aind).footprint_circle_size);
    %xlab = sprintf('Size of footprint: %d', actions(tind).action_list(aind).path_length);
    xlabel(xlab);
        for find = 1:actions(tind).action_list(aind).footprint_size
             hold on; plot(actions(tind).action_list(aind).footprint(find).x+offset(a,1), actions(tind).action_list(aind).footprint(find).y+offset(a,2), '.k');
        end
%         for pind = 1:actions(tind).action_list(aind).path_length
%            % figure(tind);hold on;
%              %plot(actions(tind).action_list(aind).path(pind).x+offset(a,1), actions(tind).action_list(aind).path(pind).y+offset(a,2), '.k');
%              plot(actions(tind).action_list(aind).path(pind).x, actions(tind).action_list(aind).path(pind).y, '.k');
%         end
%         x = []; y = [];
%         for pind = 1:actions(tind).action_list(aind).path_length_rw
%             x = [x actions(tind).action_list(aind).path_rw(pind).x];
%             y = [y actions(tind).action_list(aind).path_rw(pind).y];
%             %plot(, actions(tind).action_list(aind).path_rw(pind).y, '.k');
%         end            
%         plot(x,y, 'k');
        for find = 1:actions(tind).action_list(aind).footprint_circle_size
            plot(actions(tind).action_list(aind).footprint_circle(find).x+offset(a,1),...
            actions(tind).action_list(aind).footprint_circle(find).y+offset(a,2), '.k'); 
        end
% 
%         for pind=1:actions(tind).action_list(aind).path_length
%             figure(99); hold on;
%              plot(actions(tind).action_list(aind).path(pind).x*.05, actions(tind).action_list(aind).path(pind).y*.05, '.r');
%         end
        
    end
    
%     car = [0.5 0.5; 0.5 -0.5; -0.5 -0.5; -0.5 0.5; 0.5 0.5];
%     figure(99); hold on; plot(car(:,1), car(:,2), 'b');
 %end