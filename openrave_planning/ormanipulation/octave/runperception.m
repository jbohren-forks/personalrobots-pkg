#!/usr/bin/env octave
global updir probs

cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
%orEnvSetOptions('debug debug');

robot = SetupTableScene('data/pr2table_real.env.xml');
orRobotControllerSet(robot.id, 'ROSRobot');

out = orProblemSendCommand('createsystem ObjectTransform /checkerdetector/ObjectDetection 0.1',probs.task);
if( isempty(out) )
    error('failed to create checkerboard detector');
end

out = orProblemSendCommand('createsystem PhaseSpace phase_space_snapshot',probs.task);
if( isempty(out) )
    error('failed to create phasespace');
end
