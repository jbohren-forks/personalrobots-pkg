function playActions(actions)

figure(1);% axis([-5 5 -5 5]);hold on; %trajectories
figure(2); %axis([-5 5 -5 5]);hold on; %footprint
for tind=1:length(actions)
    for aind=1:length(actions(tind).action_list)
%         for find = 1:actions(tind).action_list(aind).footprint_size
%             figure(tind); hold on; plot(actions(tind).action_list(aind).footprint(find).x, actions(tind).action_list(aind).footprint(find).y, '.r');
%         end
        for pind = 1:actions(tind).action_list(aind).path_length
            figure(tind);hold on; plot(actions(tind).action_list(aind).path(pind).x, actions(tind).action_list(aind).path(pind).y, '.b');
        end
        for pind = 1:actions(tind).action_list(aind).path_length_rw
            figure(tind);hold on; plot(actions(tind).action_list(aind).path_rw(pind).x, actions(tind).action_list(aind).path_rw(pind).y, '.b');
        end            

    end
end