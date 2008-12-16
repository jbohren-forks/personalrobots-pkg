% robots = orEnvGetRobots(robotid,options)
%
%% Input:
%% robotid - uid of robot
%% options - set of options that controls what is sent back (BodyInfo.Req_X and RobotInfo.Req_X).
%%           If none specified gets everything.
%% Output:
% robots is a cell array of robots
% every cell contains a struct with the following parameters
% id - robotid
% filename - filename used to initialize the body with
% name - human robot name
% type - type of robot

function robots = orEnvGetRobots(robotid, options)
session = openraveros_getglobalsession();
req = openraveros_env_getrobots();
if( exist('robotid','var') )
    req.bodyid = robotid;
end
if( exist('options','var') )
    req.options = options;
else
    req.options = 65535; % get everything
end
res = rosoct_session_call(session.id,'env_getrobots',req);

if(~isempty(res))
    robots = cell(length(res.robots),1);
    for i = 1:length(res.robots)
        robots{i} = openraveros_getrobotinfo(res.robots{i});
    end
else
    robots = {};
end
