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

start_grid = round(start_pt ./ cellsize)
end_grid = round(end_pt ./ cellsize)

%TODO: figure out how to get the read robot dimensions
robot_footprint2 = robot_footprint ./ cellsize;
dist_from_robot_center = sqrt(robot_footprint2(:,1).^2 + robot_footprint2(:,2).^2);
radius = round(max(dist_from_robot_center))


if(start_grid(1) - radius <= 0 || start_grid(1) + radius > dimensions(1) ||...
    start_grid(2) - radius <=0 || start_grid(2) + radius > dimensions(2))
    fprintf(1, 'Illegal start point - robot cannot physically start here\n');
    return;
end

if(end_grid(1) - radius <= 0 || end_grid(1) + radius > dimensions(1) ||...
        end_grid(2) - radius <=0 || end_grid(2) + radius > dimensions(2))
    fprintf(1, 'Illegal end point - robot cannot physically end here\n');
    return;
end

max_height = round(dimensions(1)/10);
max_width = round(dimensions(1)/10);

env = zeros(dimensions);

num_cells = dimensions(1)*dimensions(2);
num_obstacles = num_cells*coverage/100;

itr = 0;
max_itrs = 100;

obs = 0;

while(obs < num_obstacles && itr < max_itrs)
    %pick a random center
    center = round(1 + (num_cells-1).*rand(1,1));
    [cx, cy] = ind2sub(dimensions, center);
    
    %pick a random width and height
    hgt = round(1 + (max_height-1).*rand(1,1));
    wdt = round(1 + (max_width-1).*rand(1,1));
    
    %fill the obstacle
    if(cx-hgt <= 0 || cx+hgt > dimensions(1))
        continue;
    end
    if(cy-wdt <= 0 || cy+wdt > dimensions(2))
        continue;
    end
    
    env(cx-hgt:cx+hgt, cy-wdt:cy+wdt) = 1;

    %clear out start and goal - TODO: maybe this just once at end
    env(start_grid(1) - radius: start_grid(1) + radius, start_grid(2) - radius: start_grid(2) + radius) = 0;
    env(end_grid(1) - radius: end_grid(1) + radius, end_grid(2) - radius: end_grid(2) + radius) = 0;
    %obs = obs + (2*hgt*2*wdt) 
    obs = sum(sum(env))
    
    itr = itr + 1;
end

if(itr == max_itrs)
    fprintf(1, 'Unable to acheive desired coverage\n');
    return;
end

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
    fprintf(fid, '%d ', env(i,:));
    fprintf(fid, '\n');
end

fclose(fid);