% values = orRobotGetDOFLimits(robotid)
%
% Gets the robot's dof limits in a Nx2 vector where N is the DOF, the first column
% is the low limit and the second column is the upper limit

function values = orRobotGetDOFLimits(robotid)

%% get some robot info
session = openraveros_getglobalsession();
req = openraveros_env_getbodies();
req.bodyid = robotid;
res = rosoct_session_call(session.id,'env_getbodies',req);
if( ~isempty(resinfo) )
    if( length(res.lowerlimit) ~= length(res.upperlimit) )
        error('limits not same size');
    end

    values = zeros(length(res.lowerlimit),2);
    values(:,1) = cell2mat(res.lowerlimit);
    values(:,2) = cell2mat(res.upperlimit);
else
    values = [];
end
