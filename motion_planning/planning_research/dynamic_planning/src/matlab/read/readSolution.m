function solution = readSolution(solFile)

fid = fopen(solFile, 'r');

c = textscan(fid, '%d %d %f %f %f');

solution.thetas = [c{1}];
solution.actions = [c{2}];
solution.path = [c{3} c{4} c{5}];