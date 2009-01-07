%% startup file that adds the correct octave paths across the ROS system
more off;
[status,rosoctpath] = system(['rospack find rosoct']);
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));
rosoct('shutdown');
