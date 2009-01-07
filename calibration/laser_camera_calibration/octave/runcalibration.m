#!/usr/bin/env octave
startup;

%% start openrave
openraverospath = rosoct_findpackage('openraveros');
if( ~isempty(openraverospath) )
    addpath(fullfile(openraverospath,'octave'));
end
openraveros_restart([],[]); % don't launch any viewer

robotid = orEnvCreateRobot('pr2','robots/pr2full.robot.xml');
if( isempty(robotid) || robotid == 0 )
    error('failed to create robot');
end

robot = orEnvGetRobots(robotid);

calibdata = startgathering(robot); 

[Tcamera, Tlaser, jointoffsets] = calibratevalues(calibdata, robot); % compute the calibration values
Tcamera
Tlaser
jointoffsets
