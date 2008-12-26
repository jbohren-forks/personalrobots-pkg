%% startup file that adds the correct octave paths across the ROS system

more off;

[status,rosoctpath] = system(['rospack find rosoct']);
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));

openraverospath = rosoct_findpackage('openraveros');
if( ~isempty(openraverospath) )
    addpath(fullfile(openraverospath,'octave'));
end

setrealsession();
openraveros_restart();
orEnvSetOptions('wdims 800 600');
%orEnvSetOptions('collision bullet');
%orEnvSetOptions('debug verbose');
