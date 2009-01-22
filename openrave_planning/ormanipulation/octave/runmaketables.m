#!/usr/bin/env octave
%% generates grasp tables for specific objects
global probs
[status,rosoctpath] = system(['rospack find rosoct']);
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));
cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
GraspTable = MakePR2GraspTables('data/ricebox.kinbody.xml');
