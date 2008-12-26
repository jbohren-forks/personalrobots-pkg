%% success = StartTrajectory(robotid, probid, trajdata)
%%
%% Starts a trajectory on the real robot and waits for it, in the end sets the new robot values
%% in the cloned (current) world.
function success = StartTrajectory(robotid, probid, trajdata,timelimit)

if( isempty(trajdata) )
    success = 1;
    return;
end

if( ~exist('timelimit','var') )
    timelimit = 0;
end

prevsession = openraveros_getglobalsession();
setrealsession();
orProblemSendCommand(['traj stream ' trajdata],probid);

display('waiting for robot');
success = 1;
dowait = 1;
pause(0.3); % pause a little to give a chance for controller to start
tic;

while(dowait == 1 & (orEnvWait(robotid, 0.05) == 0) )
    if( timelimit > 0 && toc > timelimit )
        success = 0;
        break;
    end

    %% only update if different
    if( prevsession.id ~= openraveros_getglobalsession().id )
        newjointconfig = orBodyGetJointValues(robotid);
        setclonesession(prevsession);
        orBodySetJointValues(robotid,newjointconfig);
        setrealsession();
    end
end

newjointconfig = orBodyGetJointValues(robotid);
setclonesession(prevsession);
orBodySetJointValues(robotid,newjointconfig);
