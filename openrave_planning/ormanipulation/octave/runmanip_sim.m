#!/usr/bin/env octave
global updir probs
[status,rosoctpath] = system(['rospack find rosoct']);
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));
cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
[robot, scenedata] = SetupTableScene('data/pr2table.env.xml',0);
RunDynamicGrasping(robot, scenedata, 1,0);
