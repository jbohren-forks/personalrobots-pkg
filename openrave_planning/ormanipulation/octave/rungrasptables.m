#!/usr/bin/env octave
%% runs through all the grasps in a generated grasp table
global probs
cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
%% add the openrave grasping path
addpath(fullfile(rosoct_findpackage('openrave'),'share','openrave','examples','grasping'));

%% loop forever
while(1)
    RunGrasps('grasp_pr2_ricebox.mat');
    pause(0.1);
end
