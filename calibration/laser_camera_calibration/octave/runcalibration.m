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

%% initial estimate of camera
Tcamerainit = [0 0 1 0.05;
               -1 0 0 0.05;
               0 -1 0 0.095];
[Tcamera, Tlaser, jointoffsets] = calibratevalues(calibdata, robot, Tcamerainit); % compute the calibration values
disp('camera transform: ');
Tcamera
disp('laser transform: ');
Tlaser
disp('joint offsets: ');
jointoffsets
