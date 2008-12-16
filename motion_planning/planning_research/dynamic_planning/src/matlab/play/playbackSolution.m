function playbackSolution(solution, cfg)

coords = find(cfg.environment~=0);

[h, w] = ind2sub(size(cfg.environment), coords);

figure(1);
plot(w.*cfg.cellsize + ones(size(w)).*cfg.cellsize/2, h.*cfg.cellsize + ones(size(h)).*cfg.cellsize/2, '+k');

hold on;
plot(solution(:,1), solution(:,2), '.b');