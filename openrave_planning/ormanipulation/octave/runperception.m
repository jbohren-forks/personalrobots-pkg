#!/usr/bin/env octave
global updir probs

cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
%orEnvSetOptions('debug debug');

robot = SetupTableScene('data/pr2table_real.env.xml');

%% enable all but the left arm
enabledjoints = 0:(robot.dof-1);
enabledjoints([robot.manips{1}.armjoints; robot.manips{1}.handjoints]) = [];
jointnames_cell = transpose(robot.jointnames(enabledjoints+1));
jointnames_str = cell2mat (cellfun(@(x) [x ' '], jointnames_cell,'uniformoutput',false));
orRobotControllerSet(robot.id, 'ROSRobot',  ['joints ' jointnames_str]);

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
