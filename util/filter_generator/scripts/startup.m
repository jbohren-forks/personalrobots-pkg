more off;

[status,rosoctpath] = system('rospack find rosoct');
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));

rosoct_add_srvs('filter_generator');

rosoct('shutdown');
