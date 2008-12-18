%% startup file that adds the correct octave paths across the ROS system
[status,rosoctpath] = system(['rospack find rosoct']);
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));

openraverospath = rosoct_findpackage('openraveros');
if( ~isempty(openraverospath) )
    addpath(fullfile(openraverospath,'octave'));
end

openraveros_restart();
