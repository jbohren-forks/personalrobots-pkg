%% GraspTable = MakePR2GraspTables(targetfilename, grasptablefilename)
%%
%% Makes Grasp tables for the PR2

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
function GraspTable = MakePR2GraspTables(targetfilename, grasptablefilename)
global probs

%% add the openrave grasping path
addpath(fullfile(rosoct_findpackage('openrave'),'share','openrave','examples','grasping'));

thresh = 0;

% extract name from targetfilename
[tdir, name, text] = fileparts(targetfilename);

dots = strfind(name, '.');
if( ~isempty(dots) && dots(1) > 1)
    name = name(1:(dots(1)-1));
end

if( ~exist('grasptablefilename','var') )
    grasptablefilename = sprintf('grasp_pr2_%s.mat', name);
end

orEnvLoadScene('',1); % clear the scene

% setup the robot
robot = RobotCreatePR2Hand('TestHand');
probs.grasp = orEnvCreateProblem('GrasperProblem', robot.name);

preshapes = [robot.open_config 0.5*robot.open_config];

% setup the target
Target.name = name;
Target.filename = targetfilename;
Target.id = orEnvCreateKinBody(Target.name, Target.filename);

if( Target.id == 0 )
    error(['could not create body ' Target.filename]);
end

orBodySetTransform(Target.id, [0 0 0], [1 0 0 0]); % identity

standoffs = [0.01];
rolls = [0 pi/2]; % hand is symmetric

% start simulating grasps
[GraspTable, GraspStats] = MakeGraspTable(robot,Target,preshapes, standoffs, rolls,0,0.02);

if( ~isempty(GraspTable) && ~isempty(GraspStats) )
    %% save the table
    GraspTable = GraspTable(find(GraspStats(:,1) > 0),:);
    save('-v7',grasptablefilename,'GraspTable','robot','targetfilename');
end
