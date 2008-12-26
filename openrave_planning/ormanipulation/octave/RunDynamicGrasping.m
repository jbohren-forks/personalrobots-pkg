%% RunDynamicGrasping(robot, scenedata, simulation,squeeze)
%%
%% if simulation is 1, will delete the objects once they are planned for

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
function RunDynamicGrasping(robot, scenedata, simulation,squeeze)

global updir probs realsession
setrealsession();

if( ~simulation )
    DeleteObjects(scenedata.TargetObjPattern);
end

MySwitchModels = @(x) SwitchModels(scenedata.SwitchModelPatterns, x);
%MySwitchModels = @(x) 0;

memory = [];
memory.ignorelist = [];

while(1)
    orProblemSendCommand('releaseall',probs.manip);

    [curobj,memory] = GetObjectToManipulate(robot,scenedata,memory);
    if( isempty(curobj) )
        if( simulation )
            display('task done!');
            return;
        end

        %% look around
        pause(0.1); % pause for a little
        continue;
    end
        
    %% if found, create a clone
    %setclonesession(openraveros_makeclone(openraveros_openrave_session().CloneBodies()));
    
    %% try to find the new object
    curobj.info = orEnvGetBodies(curobj.id);

    if( isempty(curobj.info) )
        display(sprintf('failed to get info for obj %d (might have been deleted)', curobj.info.name));
        continue;
    end

    display(['Grasping ' curobj.info.name ' numdests: ' num2str(size(curobj.dests,2))]);
        
    %% pick up and place one object
    [success, full_solution_index] = GraspAndPlaceObject(robot, curobj, squeeze, MySwitchModels, scenedata.SwitchModelPatterns);

    if( success )
        display(sprintf('sucessfully manipulated obj %s', curobj.info.name));
    else
        display(sprintf('%s failed', curobj.info.name));
    end

    % switch back to real
    setrealsession();
end
