function env = testEnvironmentGeneration(dimensions, cellsize, nominal_velocity,... 
    time_to_turn, start_pt, end_pt, robot_footprint, coverage, filename)

env = 0;

if(size(dimensions,1) ~= 1 || size(dimensions,2) ~= 2)
    fprintf(1, 'Illegal dimensions - usage: [height width]\n');
    return;
end

if(size(start_pt,1) ~= 1 || size(start_pt, 2)~=3)
    fprintf(1, 'Illegal start pt - usage: [x y theta]\n');
    return;
end

if(size(end_pt,1) ~= 1 || size(end_pt,2) ~= 3)
    fprintf(1, 'Illegal end pt - usage: [x y theta]\n');
    return;
end

max_itrs = 3000;

%TODO: figure out how to get the read robot dimensions
dist_from_robot_center = sqrt(robot_footprint(:,1).^2 + robot_footprint(:,2).^2);
radius = round(max(dist_from_robot_center));

env = zeros(dimensions);

num_cells = dimensions(1)*dimensions(2);
num_obstacles = num_cells*coverage/100;

acheived_coverage = 0;

s = repmat(start_pt, num_obstacles, 1);
e = repmat(end_pt, num_obstacles, 1);

itrs = 0;
while(~acheived_coverage && itrs < max_itrs)
    obstacles = round(1 + (num_cells-1).*rand(num_obstacles,1));
    [h, w] = ind2sub(dimensions, obstacles);
    
    %check proximity to starting point
    dist = sqrt((h-s(:,1)).^2 + (w - s(:,2)).^2);
    no_illegal_obstacles_s = isempty(find(dist < radius));
    
    %check proximity to end point
    dist = sqrt((h - e(:,1)).^2 + (w - e(:,2)).^2);
    no_illegal_obstacles_e = isempty(find(dist < radius));
    
    if(no_illegal_obstacles_e && no_illegal_obstacles_s)
        acheived_coverage = 1;
    end
    
    itrs = itrs + 1;
end

if(itrs == max_itrs)
    fprintf(1, 'Unable to acheive desired coverage\n');
    return;
end

env(obstacles) = 1;

%write the environment to file
fid = fopen(filename, 'w');

if(fid==0)
    fprintf(1, 'Unable to open file %s for writing\n', filename);
    return;
end

fprintf(fid, 'discretization(cells): %d %d\n', dimensions(1), dimensions(2));
fprintf(fid, 'cellsize(meters): %f\n', cellsize);
fprintf(fid, 'nominalvel(mpersecs): %f\n', nominal_velocity);
fprintf(fid, 'timetoturn45degsinplace(secs): %f\n', time_to_turn);
fprintf(fid, 'start(m,rad): %d %d %d\n', start_pt);
fprintf(fid, 'end(m,rad): %d %d %d\n', end_pt);
fprintf(fid, 'num_footprint_corners: %d\n', size(robot_footprint, 1));
fprintf(fid, 'footprint: \n');
fprintf(fid, '%1.2f %1.2f\n', robot_footprint);
fprintf(fid, 'environment:\n');
for i=1:size(env,1)
    for j=1:size(env,2)
        fprintf(fid, '%d ', env(i,j));
    end
    fprintf(fid, '\n');
end

fclose(fid);