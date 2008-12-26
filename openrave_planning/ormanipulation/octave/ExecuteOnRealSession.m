%% output = ExecuteOnRealSession(cmd)
%%
%% Execute a script on the real session
function output = ExecuteOnRealSession(cmd)
prevsession = openraveros_getglobalsession();
setrealsession();
output = cmd();
setclonesession(prevsession);
