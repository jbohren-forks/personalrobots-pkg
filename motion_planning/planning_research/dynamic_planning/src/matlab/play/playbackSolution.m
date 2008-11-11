function playbackSolution(solution, actions, cfg)

x = cfg.start_x/cfg.cellsize 
y = cfg.start_y/cfg.cellsize
theta = cfg.start_theta * cfg.num_theta/(2*pi)

params.delta_t = cfg.dur_disc;

fig_num = 3;



for i=1:length(solution)
    playFootprint(x, y, theta, cfg, actions(theta+1).action_list(solution(i)), fig_num);
    [x, y, theta] = playTrajectory(x, y, theta, actions(theta+1).action_list(solution(i)), fig_num);
end

figure(fig_num); hold on;
for h=1:size(cfg.environment, 1)
    for w=1:size(cfg.environment, 2)
        if(cfg.environment(h, w)~= 0)
            plot(w-1, h-1, '+k');
        end
    end
end