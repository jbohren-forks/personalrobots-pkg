function success = RobotGoInitial(robot, home)
global probs robothome

success = 0;
armjoints = robot.manips{robot.activemanip}.armjoints;
handjoints = robot.manips{robot.activemanip}.handjoints;
robotid = robot.id;

if( ~exist('home','var') )
    if( length(robothome) == robot.dof )
        home = robothome;
    else
        display('robothome not set');
        home = zeros([robot.dof 1]);
    end
end

trajdata = orProblemSendCommand(['MoveManipulator execute 0 outputtraj armvals ' sprintf('%f ', home(armjoints+1))], probs.manip);

if( isempty(trajdata) )
    return;
end

success = StartTrajectory(robotid, trajdata)
if( ~success )
    return;
end

display('moving hand');

prevsession = openraveros_getglobalsession();
setrealsession();
orRobotSetDOFValues(robotid,home(handjoints),handjoints);
pause(0.4); % pause a little to give a chance for controller to start
success = WaitForRobot(robotid);
setclonesession(prevsession);

%orRobotSetActiveDOFs(robotid,0:(robot.dof-1));
%curvalues = orRobotGetDOFValues(robotid);
%orRobotStartActiveTrajectory(robotid,[curvalues home(:)]);
%WaitForRobot(robotid);

success = 1;
