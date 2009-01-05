%% setrealsession()
function setrealsession()
global realsession realprobs probs

if( ~isempty(realsession) )
    %% revert back to the real session
    openraveros_setglobalsession(realsession);

    if( ~isempty(realprobs) )
        probs = realprobs;
    end

    realsession = [];
end
