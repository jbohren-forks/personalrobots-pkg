%% success = StartTrajectory(robotid, trajdata)
%%
%% Starts a trajectory on the real robot and waits for it, in the end sets the new robot values
%% in the cloned (current) world.
function success = StartTrajectory(robotid,trajdata,timelimit)
global probs

if( isempty(trajdata) )
    success = 1;
    return;
end

if( ~exist('timelimit','var') )
    timelimit = 0;
end

prevsession = openraveros_getglobalsession();
prevprobs = probs;
setrealsession();
[out,trajsuc] = orProblemSendCommand(['traj stream ' trajdata],probs.manip);
if( ~trajsuc )
    display('trajectory failed');
    success = 0;
    setclonesession(prevsession);
    probs = prevprobs
    return;
end

display('waiting for robot');
success = 1;
dowait = 1;
pause(0.3); % pause a little to give a chance for controller to start

basetime = toc;
while(dowait == 1 & (orEnvWait(robotid, 0.05) == 0) )
    if( timelimit > 0 && toc-basetime > timelimit )
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

display('wait ended');
newjointconfig = orBodyGetJointValues(robotid);
setclonesession(prevsession);
probs = prevprobs
orBodySetJointValues(robotid,newjointconfig);
