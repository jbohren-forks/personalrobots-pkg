function path = readSolution(solFile)

fid = fopen(solFile, 'r');

c = textscan(fid, '%f %f %f');

path = [c{1} c{2} c{3}];