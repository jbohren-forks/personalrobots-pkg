%% [robot, scenedata] = SetupTableScene(scene,realrobot,randomize)
%%
%% setup a simple table scene with the objects to be manipulated

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
function [robot, scenedata] = SetupTableScene(scene,realrobot,randomize)

global updir probs robothome
setrealsession();

if( ~exist('realrobot','var') )
    realrobot = 0;
end

if( ~exist('randomize','var') )
    randomize = 0;
end

if( ~exist('scene','var') )
    scene = 'data/pr2table.env.xml';
end

clear grasp_pr2_ricebox.mat % clear the cache
load grasp_pr2_ricebox.mat; % has a robot itself
objind = 1;
TargetObjPattern{objind}.pattern = '^ricebox(\d)+$';
TargetObjPattern{objind}.fatfilename = 'data/riceboxf.kinbody.xml';
TargetObjPattern{objind}.grasps = GraspTable;
objind = objind + 1;

SwitchModelPatterns = {};
for i = 1:length(TargetObjPattern)
    SwitchModelPatterns{i}.pattern = TargetObjPattern{i}.pattern;
    SwitchModelPatterns{i}.fatfilename = TargetObjPattern{i}.fatfilename;
end

SwitchModelPatterns{end+1} = struct('pattern','^willow_table$','fatfilename','data/willow_tablef.kinbody.xml');

orEnvLoadScene('', 1);

%% create rosproblem before everything so resources can init!
probs.rosplan = orEnvCreateProblem('ROSPlanning');
if( isempty(probs.rosplan) )
    error('failed to create problem');
end

orEnvLoadScene(scene);

robots = orEnvGetRobots();
if( length(robots) == 0 )
    error('no robots in scene');
end

robot = robots{1};
robot.CreateHandFn = @RobotCreatePR2Hand;
robot.testhandname = 'testhand';

robothome = orBodyGetJointValues(robot.id);

probs.task = orEnvCreateProblem('TaskManipulation',robot.name);
if( isempty(probs.task) )
    error('failed to create TaskManipulation problem');
end

probs.manip = orEnvCreateProblem('BaseManipulation',robot.name);
if( isempty(probs.manip) )
    error('failed to create BaseManipulation problem');
end

orProblemSendCommand('setactivemanip 1',probs.manip); % right arm
robot.activemanip = 2; % update

tableaabb = [];
tablepattern = '^willow_table$';
bodies = orEnvGetBodies(0,openraveros_BodyInfo().Req_Names());
for i = 1:length(bodies)
    if( regexp(bodies{i}.name, tablepattern) )
        Tidentity = eye(4);
        Ttable = bodies{i}.T;
        orBodySetTransform(bodies{i}.id,Tidentity(1:3,1:4));
        ab = orBodyGetAABB(bodies{i}.id);
        orBodySetTransform(bodies{i}.id,Ttable);
        break;
    end
end

scenedata.TargetObjPattern = TargetObjPattern;
scenedata.SwitchModelPatterns = SwitchModelPatterns;
scenedata.GetDestsFn = @() GetDests(tablepattern, tableaabb);
scenedata.SetupCloneFn = @() SetupClone(robot.name);

updir = [0;0;1];

if( randomize )
    % randomize robot position, obj position, and dest position
    Trobot = orBodyGetLinks(robot.id);
    Trobot = reshape(Trobot(:,1), [3 4]);
    
    while(1)
        Tnew = Trobot;
        Tnew(1:2,4) = Tnew(1:2,4) + 4*(rand(2,1)-0.5);
        orBodySetTransform(robot.id, Tnew);
        if( ~orEnvCheckCollision(robot.id) )
            break;
        end
    end
end

if( realrobot )
    %% enable all but the left arm
    enabledjoints = 0:(robot.dof-1);
    enabledjoints([robot.manips{1}.armjoints; robot.manips{1}.handjoints]) = [];
    jointnames_cell = transpose(robot.jointnames(enabledjoints+1));
    jointnames_str = cell2mat (cellfun(@(x) [x ' '], jointnames_cell,'uniformoutput',false));
    orRobotControllerSet(robot.id, 'ROSRobot',  ['trajectoryservice /right_arm_trajectory_controller joints ' jointnames_str]);
end

%% dests is a 12xN array where every column is a 3x4 matrix
function [dests, surfaceplane] = GetDests(tablepattern, ab)

dests = [];
surfaceplane = [];
id = 0;
name = '';
bodies = orEnvGetBodies(0,openraveros_BodyInfo().Req_Names());
Ttable = [];
for i = 1:length(bodies)
    if( regexp(bodies{i}.name, tablepattern) )
        id = bodies{i}.id;
        Ttable = bodies{i}.T;
        break;
    end
end

if( id == 0 )
    display(sprintf('no table with pattern %s', tablepattern));
    return;
end

%% table up is assumed to be +z, sample the +y axis of the table
if( isempty(ab) )
    ab = orBodyGetAABB(id);
    ab(1:3,1) = ab(1:3,1) - Ttable(1:3,4);
end

if( isempty(ab) )
    display('missed table, trying again');
    dests = GetDests(tablepattern);
    return;
end

Nx = 4;
Ny = 10;
X = [];
Y = [];
for x = 0:(Nx-1)
    X = [X 0.5*rand(1,Ny)/(Nx+1) + (x+1)/(Nx+1)];
    Y = [Y 0.5*rand(1,Ny)/(Ny+1) + ([0:(Ny-1)]+0.5)/(Ny+1)];
end

offset = [ab(1,1)-ab(1,2);ab(2,1)-ab(2,2); ab(3,1)+ab(3,2)];
trans = [offset(1)+2*ab(1,2)*X; offset(2)+2*ab(2,2)*Y; repmat(offset(3),size(X))];
trans = Ttable*[trans;ones(size(X))];

Rots = [];
for roll = 0:pi/4:pi
    R = Ttable(1:3,1:3)*openraveros_rotfromaxisangle([0 0 roll]);
    Rots = [Rots R(:)];
end

%% permute the translations and rotations
dests = [repmat(Rots,[1 size(trans,2)]); reshape(repmat(trans,[size(Rots,2) 1]),[3 size(Rots,2)*size(trans,2)])];

surfaceplane = [Ttable(1:3,3);-Ttable(1:3,3)'*trans(1:3,1)]; % along z axis

orEnvClose();
orEnvPlot(dests(10:12,:)','size',10);

function SetupClone(robotname)

global probs

probs.task = orEnvCreateProblem('TaskManipulation',robotname);
if( isempty(probs.task) )
    error('failed to create TaskManipulation problem');
end

probs.manip = orEnvCreateProblem('BaseManipulation',robotname);
if( isempty(probs.manip) )
    error('failed to create BaseManipulation problem');
end
