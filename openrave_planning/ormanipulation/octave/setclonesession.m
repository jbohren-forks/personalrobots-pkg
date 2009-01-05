%% setclonesession(session)
function setclonesession(session)
global realsession realprobs probs

if( isempty(session) )
    error('setting an empty clone');
end

if( isempty(realsession) )
    %% revert back to the real session
    realsession = openraveros_getglobalsession();
    realprobs = probs;
end

openraveros_setglobalsession(session);
probs = [];
