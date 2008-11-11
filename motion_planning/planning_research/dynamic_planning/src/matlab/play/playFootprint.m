function playFootprint(x, y, theta, cfg, action, fig_num)

% H = [cos(theta) -sin(theta) x;...
%     sin(theta) cos(theta) y; ...
%     0 0 1];


x = round(x);
y = round(y);

H = [1 0 x;
     0 1 y;
     0 0 1];

figure(fig_num); hold on;
 
%corners = H * [cfg.footprint ones(size(cfg.footprint,1) ,1)]';

%plot(corners(1,:), corners(2,:));

%footprint_x = action.footprint(:).x
%action_x = [footprint_x + x.*ones(size(footprint_x))];
%footprint_y = action.footprint(:).y
%action_y = [footprint_y + y.*ones(size(footprint_y))];

for i=1:length(action.footprint)
    txt = sprintf('Footprint: %f %f, Coord: %f %f\n', action.footprint(i).x, action.footprint(i).y, x, y);
    txt;
    coords = H*[action.footprint(i).x; action.footprint(i).y; 1];
    
    plot(coords(1), coords(2), '*m');
end
