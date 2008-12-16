% values = orRobotGetDOFLimits(robotid)
%
% Gets the robot's dof limits in a Nx2 vector where N is the DOF, the first column
% is the low limit and the second column is the upper limit

function values = orRobotGetDOFLimits(robotid)

%% get some robot info
session = openraveros_getglobalsession();
req = openraveros_env_getrobots();
req.bodyid = robotid;
req.options = openraveros_RobotInfo().Req_ActiveLimits();
res = rosoct_session_call(session.id,'env_getrobots',req);
if( ~isempty(res) )
    if( res.robots{1}.bodyinfo.bodyid ~= robotid )
        error('wrong robot id');
    end

    lowerlimit = res.robots{1}.activelowerlimit;
    upperlimit = res.robots{1}.activeupperlimit;
    if( length(lowerlimit) ~= length(upperlimit) )
        error('limits not same size');
    end

    values = zeros(length(lowerlimit),2);
    values(:,1) = cell2mat(lowerlimit);
    values(:,2) = cell2mat(upperlimit);
else
    values = [];
end
