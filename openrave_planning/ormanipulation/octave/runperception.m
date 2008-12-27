#!/usr/bin/env octave
global updir probs

cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
%orEnvSetOptions('debug debug');

robot = SetupTableScene('data/pr2table_real.env.xml');
orRobotControllerSet(robot.id, 'ROSRobot');

Tcamera = [0 0 1 -0.05;
           -1 0 0 -0.05;
           0 -1 0 -0.095];
out = orProblemSendCommand(['createsystem ObjectTransform topic /checkerdetector/ObjectDetection thresh 0.1 robot ' sprintf('%d ', robot.id) ' matrixoffset ' sprintf('%f ', Tcamera(1:3,1:4))],probs.task);
if( isempty(out) )
    error('failed to create checkerboard detector');
end

out = orProblemSendCommand('createsystem PhaseSpace phase_space_snapshot',probs.task);
if( isempty(out) )
    error('failed to create phasespace');
end
