% output = orProblemSendCommand(cmd, problemid)
%
% Sends a command to the problem. The function doesn't return until
% ProblemInstance::SendCommand returns.
% cmd - the string command to send the problem
% problemid [optional] - returned id of the problem, if not specified, then
%                        command is sent to all problems
% dosync [optional] - If 1, the SendCommand is called in the main thread, in sync
%                        with the rest of the primitives. If 0, called in a different thread.
% output - the concatenated output of all the problems that the command is sent to
function output = orProblemSendCommand(cmd, problemid)
session = openraveros_getglobalsession();
req = openraveros_problem_sendcommand();
req.cmd = cmd;
req.problemid = problemid;
res = rosoct_session_call(session.id,'problem_sendcommand',req);

if(~isempty(res))
    output = res.output;
else
    output = [];
end
