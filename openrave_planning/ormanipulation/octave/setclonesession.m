%% setclonesession(session)
function setclonesession(session)
global realsession

if( isempty(session) )
    error('setting an empty clone');
end

if( isempty(realsession) )
    %% revert back to the real session
    realsession = openraveros_getglobalsession();
end

openraveros_setglobalsession(session);
