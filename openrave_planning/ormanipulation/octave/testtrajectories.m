#!/usr/bin/env octave
global updir probs

cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
orEnvLoadScene('',1);
robotid = orEnvCreateRobot('pr2','robots/pr2full.robot.xml');
robot = orEnvGetRobots(robotid);
orBodySetJointValues(robot.id,[ 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.05 0.000000 0.000000 0.000000 0.000000 0.000000 1.582704 1.081165 0.000000 -2.299995 -0.000000 0.000000 0.000000 -0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 -0.979114 -0.400000 0.000000 -1.535151 0.000000 0.000000 0.000000 0.000000 0.000000]);

orRobotControllerSet(robot.id, 'ROSRobot',  ['trajectoryservice /']);

probs.manip = orEnvCreateProblem('BaseManipulation',robot.name);
imanipulator = 2;
orProblemSendCommand(sprintf('setactivemanip %d', imanipulator-1), probs.manip);

Tgripperstart = [1.00000   0.00000   0.00000   0.5
                 0.00000   1.00000   0.00000  -0.04616
                 0.00000   0.00000   1.00000   0.7];
Tgripperend = [1.00000   0.00000   0.00000   0.5
               0.00000   1.00000   0.00000  -0.04616
               0.00000   0.00000   1.00000   1.2];

Tgrippertraj = GetInterpolatedHandTrajectory(Tgripperstart, Tgripperend);
trajectory = [];
for i = 1:size(Tgrippertraj,2)
    iksol = orProblemSendCommand(['iktest matrix ' sprintf('%f ', Tgrippertraj(:,i))],probs.manip)
    if( isempty(iksol) )
        display(sprintf('failed to find ik %d/%d', i, size(Tgrippertraj,2)));
        break;
    end

    trajectory = [trajectory sscanf(iksol, '%f ')];
    pause(0.1);
end
trajectory = [trajectory trajectory(:,end:-1:1)];

%% make a service call
rosoct_add_srvs('pr2_mechanism_controllers');
resquery = rosoct_service_call('/TrajectoryQuery',pr2_mechanism_controllers_TrajectoryQuery());

req = pr2_mechanism_controllers_TrajectoryStart();
req.traj.points = {};
for i = 1:size(trajectory,2)
    req.traj.points{i} = pr2_mechanism_controllers_JointTrajPoint();
    req.traj.points{i}.positions = [trajectory(:,i);0.4];
end
res = rosoct_service_call('/TrajectoryStart',req);

%% play it out in simulation
for i = 1:size(trajectory,2)
    orBodySetJointValues(robot.id,trajectory(:,i),robot.manips{imanipulator}.armjoints);
    pause(0.1);
end

robothand = RobotCreatePR2Hand('pr2gripper');
for i = 1:size(Tgrippertraj,2)
    orBodySetTransform(robothand.id,Tgrippertraj(:,i));
    pause(0.1);
end
