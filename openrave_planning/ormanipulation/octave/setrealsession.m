%% setrealsession()
function setrealsession()
global realsession

if( ~isempty(realsession) )
    %% revert back to the real session
    openraveros_setglobalsession(realsession);
    realsession = [];
end
