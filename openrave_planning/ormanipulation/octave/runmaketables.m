#!/usr/bin/env octave
%% generates grasp tables for specific objects
global probs
cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
GraspTable = MakePR2GraspTables('data/ricebox.kinbody.xml');
