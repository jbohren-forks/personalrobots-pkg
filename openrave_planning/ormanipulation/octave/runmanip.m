#!/usr/bin/env octave
global updir probs
cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
[robot, scenedata] = SetupTableScene('data/pr2table_real.env.xml',1);
