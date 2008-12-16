% bodies = orEnvGetBodies(bodyid,options)
%
% Input:
% bodyid (optional) - specific body to get info for, if not specified returns all bodies
% options - mask of BodyInfo.Req_X options (if not specified gets all information)
% Output:
% bodies is a cell array of all body objects in the scene
% every cell contains a struct with the following parameters
% id - bodyid
% filename - filename used to initialize the body with
% name - human robot name
% type - xml type of body

function bodies = orEnvGetBodies(bodyid,options)
session = openraveros_getglobalsession();
req = openraveros_env_getbodies();
if( exist('bodyid','var') )
    req.bodyid = bodyid;
end
if( exist('options','var') )
    req.options = options;
else
    req.options = 65535; % get everything
end
res = rosoct_session_call(session.id,'env_getbodies',req);

if(~isempty(res))
    bodies = cell(length(res.bodies),1);
    for i = 1:length(res.bodies)
        bodies{i} = openraveros_getbodyinfo(res.bodies{i});
    end
else
    bodies = [];
end