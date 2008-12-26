%% robot = RobotCreatePR2Hand(name)
%%
%% creates the barrett hand robot

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
function robot = RobotCreatePR2Hand(name)

robot.id = orEnvGetBody(name);
robot.filename = 'robots/pr2gripperfull.robot.xml';

if( robot.id <= 0 )
    robot.id = orEnvCreateRobot(name, robot.filename);
    if( robot.id == 0 )
        error('failed to create %s', robot.filename);
    end
end

info = orEnvGetRobots(robot.id);

robot.name = name;
robot.totaldof = info.dof;
robot.lowerlimit = info.lowerlimit;
robot.upperlimit = info.upperlimit;

robot.hand_type = 'pr2';
robot.preshapejoints = [];
robot.palmdir = [1 0 0];

% joints that will be controlled by the grasper
% any joints not specified belong to the preshape
robot.handjoints = find(robot.upperlimit>robot.lowerlimit)-1;
robot.open_config = robot.upperlimit;
robot.closed_config = robot.lowerlimit;

robot.releasedir = 2*(robot.open_config>robot.closed_config)-1;

robot.grasp.direction = 1:3;
robot.grasp.center = 4:6;
robot.grasp.roll = 7;
robot.grasp.standoff = 8;
robot.grasp.joints = 8+(1:robot.totaldof);
