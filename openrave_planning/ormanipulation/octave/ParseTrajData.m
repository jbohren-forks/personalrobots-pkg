%% [trajvals,transvals,timevals] = ParseTrajData(trajdata)
%%
%% parses a trajectory file 
function [trajvals,transvals,timevals] = ParseTrajData(trajdata)
vals = sscanf(trajdata, '%f');
numpts = vals(1);
numdof = vals(2);

newvals = reshape(vals(4:(3+numpts*(numdof+8))),[numdof+8 numpts]);
timevals = newvals(1,:);
transvals = newvals((2+numdof):end,:);
trajvals = newvals(2:(1+numdof),:);
