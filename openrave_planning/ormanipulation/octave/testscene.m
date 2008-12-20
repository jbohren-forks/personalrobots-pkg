#!/usr/bin/env octave
global probs

cd(getenv('OCTAVE_WORKINGDIR'));
startup;
openraveros_restart();
orEnvLoadScene('',1); % reset the scene
orEnvSetOptions('debug debug');

%% create problem before everything so resources can init!
probs.rosplan = orEnvCreateProblem('ROSPlanningProblem');
if( isempty(probs.rosplan) )
    error('failed to create problem');
end

orEnvLoadScene('data/pr2table_real.env.xml');

out = orProblemSendCommand('createsystem ObjectTransform /checkerdetector/ObjectDetection 0.1',probs.rosplan);
if( isempty(out) )
    error('failed to create checkerboard detector');
end

out = orProblemSendCommand('createsystem PhaseSpace phase_space_snapshot',probs.rosplan);
if( isempty(out) )
    error('failed to create phasespace');
end
