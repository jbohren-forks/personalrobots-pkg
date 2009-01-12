#!/usr/bin/env octave
cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

global updir probs

startup;
[robot, scenedata] = SetupTableScene('data/pr2table_real.env.xml',1);

Tlaser = [1.00000   0.00062   0.00232   0.01577
          -0.00066   0.99988   0.01554  -0.00208
          -0.00231  -0.01555   0.99988   0.03934];
Tcamera = [0.01611   0.01631   0.99974   0.02890
         -0.99966  -0.02020   0.01644   0.03236
         0.02047  -0.99966   0.01598   0.09532];

%% objects from camera
out = orProblemSendCommand(['createsystem ObjectTransform topic /checkerdetector/ObjectDetection thresh 0.1 robot ' sprintf('%d ', robot.id) ' matrixoffset ' sprintf('%f ', Tcamera(1:3,1:4))],probs.task);
if( isempty(out) )
    error('failed to create checkerboard detector');
end

%% laser-based dynamic collision map
out = orProblemSendCommand('createsystem CollisionMap collision_map',probs.task);
if( isempty(out) )
    error('failed to create collision map');
end

%% phase space system
out = orProblemSendCommand('createsystem PhaseSpace phase_space_snapshot',probs.task);
if( isempty(out) )
    error('failed to create phasespace');
end
