% [success, full_solution_index] = GraspAndPlaceObject(robot, curobj, squeeze, MySwitchModels, SwitchModelPatterns)
%
% finds a grasp for the current object depending on the environment
% The object will be placed on the table
% curobj has the following members
% - id - unique identifier
% - name - openrave name of obj
% - fatfilename - the filename of the padded object
% - grasps - the grasp table
% - axis - axis used to compute the distance map
% - dests - 12xn matrix of possible destinations of the object. Each column
%           is a 3x4 matrix, get by reshape(dests(:,i),[3 4])

%% Software License Agreement (BSD License)
%% Copyright (c) 2006-2009, Rosen Diankov
%% Redistribution and use in source and binary forms, with or without
%% modification, are permitted provided that the following conditions are met:
%%   * Redistributions of source code must retain the above copyright notice,
%%     this list of conditions and the following disclaimer.
%%   * Redistributions in binary form must reproduce the above copyright
%%     notice, this list of conditions and the following disclaimer in the
%%     documentation and/or other materials provided with the distribution.
%%   * The name of the author may not be used to endorse or promote products
%%     derived from this software without specific prior written permission.
%%
%% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%% POSSIBILITY OF SUCH DAMAGE.
function [graspsuccess, full_solution_index] = GraspAndPlaceObject(robot, curobj, squeeze, MySwitchModels, SwitchModelPatterns)

global updir probs

test = 0;
graspsuccess = 0;

% kill all test hands
handrobot = robot.CreateHandFn(robot.testhandname);
orBodySetTransform(handrobot.id, [100 0 0], [1 0 0 0]);

full_solution_index = -1;

display(sprintf('picking up object %s', curobj.info.name));

%switch back to real models
if( MySwitchModels(0) )
    curobj.id = orEnvGetBody(curobj.info.name);
end

% get the transformation
TargTrans = orBodyGetLinks(curobj.id);
TargTrans = reshape(TargTrans(:,1), [3 4]);

% transform
grasps = [];
GraspData = curobj.grasps;

gi = handrobot.grasp;
grasps(:,1:3) = GraspData(:,1:3) * transpose(TargTrans(:,1:3));
grasps(:,4:6) = GraspData(:,4:6) * transpose(TargTrans(:,1:3)) + repmat(transpose(TargTrans(:,4)), [size(grasps,1) 1]);
grasps(:,7:size(GraspData,2)) = GraspData(:,7:end);

order = 1:size(grasps,1);
curgrasp = 1;

armjoints = robot.manips{robot.activemanip}.armjoints;
handjoints = robot.manips{robot.activemanip}.handjoints;
wristlinkid = robot.manips{robot.activemanip}.eelink;
robotid = robot.id;

while(curgrasp < size(grasps,1))
                     
    %% C++ grasp testing (fast)
    g = transpose(grasps(curgrasp:end,:));
    
    offset = 0.02;
    
    cmd = ['testallgrasps combinepreshapetraj execute 0 outputtraj palmdir ' sprintf('%f ', handrobot.palmdir) ...
           ' target ' curobj.info.name ' robothand ' handrobot.name ' updir ' sprintf('%f ', updir) ...
           ' robothandjoints ' sprintf('%d ', length(handjoints), handjoints) ...
           ' handjoints ' sprintf('%d ', length(handrobot.handjoints), handrobot.handjoints) ...
           ' graspindices ' sprintf('%d ', [gi.direction(1) gi.center(1) gi.roll(1) gi.standoff(1) gi.joints(1)]-1) ...
           ' offset ' sprintf('%f ', offset) ' destposes ' num2str(size(curobj.dests,2)) ' ' sprintf('%f ', curobj.dests) ...
           ];
                                     
    for i = 1:length(SwitchModelPatterns)
        cmd = [cmd ' switch ' SwitchModelPatterns{i}.pattern ' ' SwitchModelPatterns{i}.fatfilename ' '];
    end

    cmd = [cmd ' grasps ' num2str(size(g,2)) ' ' num2str(size(g,1)) ' '];
    [response, success] = orProblemSendCommand([cmd sprintf('%f ', g)], probs.task);
        
    if( isempty(response) || ~success )
	    display(['failed to find grasp for object ' curobj.info.name]);
        return;
    end

    %% parse the response
    [goaliksol, rem] = ReadValsFromString(response,length(armjoints));
    [newhandTm, rem] = ReadValsFromString(rem,12);
    newhandTm = reshape(newhandTm,[3 4]);
    
    [graspindex,rem] = ReadValsFromString(rem,1);
    [destindex,rem] = ReadValsFromString(rem,1);
    [searchtime,trajdata] = ReadValsFromString(rem,1);
    curgrasp = curgrasp + graspindex;

	putsuccess=1;
    grasp = grasps(curgrasp,:);
    display(['grasp: ' sprintf('%d ', [curgrasp order(curgrasp)]) ]);
        
    %orRobotControllerSend(robotid, 'ignoreproxy'); % make sure nothing is ignored

    % have to set the desired open config here
    open_config = transpose(grasp(handrobot.grasp.joints));
    %orRobotSetDOFValues(robotid, open_config, handjoints);

    curgrasp = curgrasp+1; % want the next grasp

    if(test)
        return;
    end

    % start the trajectory
    success = StartTrajectory(robotid,probs.manip,trajdata);
    if( ~success )
        warning('failed to start initial traj');
        return;
    end
        
    display('moving hand');
    [trajdata, success] = orProblemSendCommand(['MoveHandStraight execute 0 outputtraj direction ' num2str(grasp(1:3)) ...
                                     ' stepsize 0.001 maxsteps ' num2str(ceil(offset/0.001+10))], probs.manip);
    if( ~success )
        error('failed to movehandstraight');
    end
    success = StartTrajectory(robotid,probs.manip,trajdata);
    if( ~success )
        warning('failed to move hand straight');
        return;
    end

%     if( squeeze )
%         display('squeezing');
%         orProblemSendCommand('Squeeze');
%         WaitForRobot(robotid);
% 
%         % check if successful
%         startoffset = robot.totaldof-robot.handdof;
%         sizeconfig = length(robot.closed_config);
%         curvalues = orRobotGetDOFValues(robotid,startoffset:(startoffset+sizeconfig-1));
%         if( all(curvalues > robot.closed_config') )
%             display(['squeeze failed (' sprintf('%f ', curvalues') ', going home and trying again.. ']);
%             orProblemSendCommand('releaseall');
%             orRobotControllerSend(robotid, 'ignoreproxy');
%             % first open the fingers
%             orProblemSendCommand(['ReleaseFingers target ' curobj.info.name ...
%                 ' open ' num2str(length(handrobot.open_config)) sprintf('%f ', handrobot.open_config) ...
%                 num2str(handjoints)]);
%             WaitForRobot(robotid);
%             RobotGoInitial(robot);
%             WaitForRobot(robotid);
%             % enable the object again
%             orProblemSendCommand(['visionenable ' num2str(curobj.id) ' 1']);
%             pause(0.5);
%             continue;
%         end
% 
%         display('squeeze successful before liftoff');
% 
%         % disable hand joints from listening to anymore commands
%         orRobotControllerSend(robotid, 'ignoreproxy 1');
%     else
        display('closing fingers');
        [trajdata, success] = orProblemSendCommand(['CloseFingers execute 0 outputtraj offset ' sprintf('%f ', 0.04*ones(size(handjoints)))] , probs.manip);
        if( ~success )
            error('failed to movehandstraight');
        end
        success = StartTrajectory(robotid,probs.manip,trajdata);
        if( ~success )
            warning('failed to close fingers');
            return;
        end
%    end

%     if( MySwitchModels(1) )
%          curobj.id = orEnvGetBody(curobj.info.name);
%     end

    orProblemSendCommand(['grabbody name ' curobj.info.name], probs.manip);
    [trajdata,success] = orProblemSendCommand(['MoveHandStraight execute 0 outputtraj stepsize 0.003 minsteps 10 ' ...
                                     ' maxsteps 60 direction ' sprintf('%f ', updir)], probs.manip);
    if( ~success )
        error('failed to movehandstraight');
    end

    %% not going to work well
    %ExecuteOnRealSession(@() orProblemSendCommand(['grabbody name ' curobj.info.name], probs.manip));

    success = StartTrajectory(robotid,probs.manip,trajdata);
    if( ~success )
        warning('failed to move hand straight');
        return;
    end
    
    squeezesuccess = 1;

%     if( squeezesuccess & squeeze )
%         display('checking for squeeze failures');
%         
%         startoffset = robot.totaldof-robot.handdof;
%         sizeconfig = length(robot.closed_config);
%         curvalues = orRobotGetDOFValues(robotid,startoffset:(startoffset+sizeconfig-1));
%         if( all(curvalues > robot.closed_config') )
%             display(['squeeze failed (' sprintf('%f ', curvalues') ', releasing cup']);
%             squeezesuccess = 0;
%         else
%             display('squeeze successful after liftoff');
%         end
%     end
    
    if( squeezesuccess > 0 )
        % check if the arm
        display('planning to table');
        % set current arm transformation to be grasping the cup initially

        putsuccess = 0;
        
        [trajdata,success] = orProblemSendCommand(['MoveToHandPosition execute 0 outputtraj matrix ' sprintf('%f ', newhandTm)], probs.manip);
        if( success )
            success = StartTrajectory(robotid,probs.manip,trajdata);
            if( ~success )
                warning('failed to move hand');
                return;
            end
            putsuccess = 1;
        else
            putsuccess = 0;
        end

        if( putsuccess == 0 )
            display('planned target inaccessible, trying new plan');
            curobj.id = orEnvGetBody(curobj.info.name);
            backuptrans = orBodyGetLinks(curobj.id);
            objTm = reshape(backuptrans,[3 4]);

            wristtrans = orBodyGetLinks(robotid);
            wristTm = reshape(wristtrans(:,wristlinkid),[3 4]);

            orProblemSendCommand('releaseall', probs.manip);

            for posi = 1:size(curobj.dests,2)

                T = reshape(curobj.dests(:,posi), [3 4]);
                goalTm = T;
                N = 10;
                for ang = 0:pi/N:pi*(N-1)/N
                    Rrand = openraveros_rotfromaxisangle([0 0 ang]);
                    goalTm(1:3,1:3) = Rrand * T(1:3,1:3);

                    newtrans = [goalTm; 0 0 0 1] * inv([objTm; 0 0 0 1]);
                    newhandTm = newtrans*[wristTm;0 0 0 1];

                    goalTm
                    newhandTm
                    
                    if ~isempty(orProblemSendCommand(['IKtest nocol matrix ' sprintf('%f ', newhandTm)], probs.manip))
                        putsuccess = 1;
                        break;
                    end
                end

                if( putsuccess )
                    break;
                end
            end
            
            orProblemSendCommand(['GrabBody name ' curobj.info.name], probs.manip);

            if( putsuccess )
                [trajdata,success] = orProblemSendCommand(['MoveToHandPosition execute 0 outputtraj matrix ' sprintf('%f ', newhandTm)], probs.manip);
                if( ~success )
                    warning('failed to movehandstraight');
                    continue;
                end

                success = StartTrajectory(robotid,probs.manip,trajdata);
                if( success )
                    warning('trajectory failed to execute');
                    break;
                end
            end
        end % putsuccess==0

%         if( MySwitchModels(0) )
%             curobj.id = orEnvGetBody(curobj.info.name);
%         end
        
        % after trajectory is done, check for failure of grasp (could have
        % dropped cup)
%         if( putsuccess & squeeze )
%             display('checking for squeeze failures');
% 
%             startoffset = robot.totaldof-robot.handdof;
%             sizeconfig = length(robot.closed_config);
%             curvalues = orRobotGetDOFValues(robotid,startoffset:(startoffset+sizeconfig-1));
%             if( all(curvalues > robot.closed_config') )
%                 % completely failed 
%                 display(['squeeze failed (' sprintf('%f ', curvalues') ', releasing cup']);
%                 orProblemSendCommand('releaseall');
%                 orRobotControllerSend(robotid, 'ignoreproxy');
%                 % first open the fingers
%                 orProblemSendCommand(['ReleaseFingers target ' curobj.info.name ...
%                     ' open ' num2str(length(handrobot.open_config)) sprintf('%f ', handrobot.open_config) ...
%                     num2str(handjoints)]);
%                 WaitForRobot(robotid);
%                 RobotGoInitial(robot);
%                 WaitForRobot(robotid);
%                 % enable the object again (the vision system will delete it)
%                 orProblemSendCommand(['visionenable ' num2str(curobj.id) ' 1']);
%                 pause(0.5);
%                 return;
%             else
%                 display('squeeze successful after liftoff');
%             end
%         end
    end % squeeze success
    
    % even if putsuccess==0 or squeezesuccess==0, need to move the object down and release it,
    % and lift the arm back up
        
    % move the hand down until collision
    display('moving hand down');

    [trajdata, success] = orProblemSendCommand(['MoveHandStraight execute 0 outputtraj direction ' sprintf('%f ', -updir) ...
                             ' maxdist ' sprintf('%f ', 0.3)],probs.manip);
    if( ~success )
        error('failed to movehandstraight');
    end
    success = StartTrajectory(robotid,probs.manip,trajdata);
    if( ~success )
        warning('failed to move hand straight');
        return;
    end

    display('opening hand');

    % reenable hand joints
    %orRobotControllerSend(robotid, 'ignoreproxy');
    orRobotSetActiveDOFs(robotid, handjoints);
    releasedir = 2*(open_config>handrobot.closed_config)-1;
    [trajdata, success] = orProblemSendCommand(['ReleaseFingers execute 0 outputtraj target ' curobj.info.name ...
                                     ' movingdir ' sprintf('%f ', handrobot.releasedir)], probs.manip);

    %% cannot wait forever since hand might get stuck
    if( ~success )
        error('failed to release fingers');
    end

    success = StartTrajectory(robotid,probs.manip,trajdata,4);
    if( ~success )
        warning('trajectory failed to release fingers');
        return;
    end

    %% now release object
    orProblemSendCommand('releaseall', probs.manip);
    
%     if( MySwitchModels(0) )
%         curobj.id = orEnvGetBody(curobj.info.name);
%     end

    if( squeezesuccess > 0 & putsuccess > 0 )
        display('success, putting down');
        % only break when succeeded
%         if( MySwitchModels(1) )
%             curobj.id = orEnvGetBody(curobj.info.name);
%         end
        
        %orProblemSendCommand(['MoveHandStraight stepsize 0.003 minsteps ' sprintf('%f ', 90) ' maxsteps ' sprintf('%f ', 100) ' direction ' sprintf('%f ', updir')]);
        %RobotGoInitial(robot);
        
        if( MySwitchModels(0) )
            curobj.id = orEnvGetBody(curobj.info.name);
        end

        graspsuccess = 1;
        break;
    else
        display('failed');
        
        % go to initial
        RobotGoInitial(robot);
        
        if( MySwitchModels(0) )
            curobj.id = orEnvGetBody(curobj.info.name);
        end
    end
end

if(test)
    display('failed to find successful grasp');
    return;
end
