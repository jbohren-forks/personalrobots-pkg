% [output,success] = orProblemSendCommand(cmd, problemid,envlock)
%
% Sends a command to the problem. The function doesn't return until
% ProblemInstance::SendCommand returns.
% cmd - the string command to send the problem
% problemid [optional] - returned id of the problem, if not specified, then
%                        command is sent to all problems
% envlock - if non zero, locks the environment before calling SendCommand (default is 0)
% output - the concatenated output of all the problems that the command is sent to
% success - if 1, command completed successfully, otherwise 0
function [output, success] = orProblemSendCommand(cmd, problemid,envlock)
session = openraveros_getglobalsession();
req = openraveros_problem_sendcommand();
req.cmd = cmd;
req.problemid = problemid;

if( exist('envlock','var') )
    req.envlock = envlock;
end

res = rosoct_session_call(session.id,'problem_sendcommand',req);

if(~isempty(res))
    output = res.output;
    success = 1;
else
    output = [];
    success = 0;
end
