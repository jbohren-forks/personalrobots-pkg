% orRobotSetDOFValues(robotid, values, indices)
%
% Sets the DOF values of the robot
% robotid - unique id of the robot
% values - the joint values of the robot
% indices [optional] - the indices of the dofs to set of the robot. 
%                      If indices is not specified the active degrees of freedom
%                      set by previous calls to orRobotSetActiveDOFs will be used.
%                      Note that specifying indices will not change the active dofs
%                      of the robot.

function success = orRobotSetDOFValues(robotid, values, indices)
session = openraveros_getglobalsession();
req = openraveros_robot_setactivevalues();
req.bodyid = robotid;
req.values = mat2cell(values(:)',1,ones(1,length(values)));
if( exist('indices','var') )
    req.indices = mat2cell(indices(:)',1,ones(1,length(indices)));
end
res = rosoct_session_call(session.id,'robot_setactivevalues',req);
success = ~isempty(res);
