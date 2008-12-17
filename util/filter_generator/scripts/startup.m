more off;

[status,rosoctpath] = system('rospack find rosoct');
rosoctpath = strtrim(rosoctpath);

addpath(fullfile(rosoctpath, 'octave'));
addpath(fullfile(rosoctpath, 'msg/oct/rosoct'));
addpath(fullfile(rosoctpath, 'srv/oct/rosoct'));

rosoct('shutdown');
