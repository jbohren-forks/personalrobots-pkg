function solution = readSolution(solutionFile, cfg)

fid = fopen(solutionFile, 'r');
c = textscan(fid, 'Computing Successors: %d %d %d\n');

for i=1:length(c{1})
    orig_coords(i,:) = [c{1}(i) c{2}(i) c{3}(i)];
end

c_goal = textscan(fid, 'Adding goal state to list: %d %d %d\n');
orig_coords = [orig_coords; c_goal{1} c_goal{2} c_goal{3}];

solution = [];
for i=2:size(orig_coords,1)
    txt = sprintf('Original Coords: %d %d %d', orig_coords(i-1,1), orig_coords(i-1,2), orig_coords(i-1,3));
    txt = strcat(txt,', New Coords: %d %d %d\n');
    c2 = textscan(fid, txt, cfg.num_rv*2 + 2);
    
    for j=1:length(c2{1})
        if(c2{1}(j)==orig_coords(i,1) && c2{2}(j)==orig_coords(i,2) && c2{3}(j)==orig_coords(i,3))
            solution = [solution j];
            break;
        end
    end
end

    