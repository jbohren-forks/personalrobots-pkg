function [x, y, theta] = playTrajectory(x, y, theta, action, fig_num)

figure(fig_num);hold on;

H = [1 0 x;
     0 1 y;
     0 0 1];
 
for i=1:length(action.path)
     coords = H*[action.path(i).x; action.path(i).y; 1];
     plot(coords(1), coords(2), '.b');
end

x = coords(1);
y = coords(2);
theta = action.path(i).theta;
