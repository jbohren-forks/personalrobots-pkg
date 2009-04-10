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
viewer = '';
if( ~strcmp(getenv('ORMANIPULATION_NOVIEWER'),'1') )
    viewer = 'qtcoin';
end

openraveros_restart('openrave_session',viewer,1);
orEnvSetOptions('wdims 800 600');
orEnvSetOptions('simulation timestep 0.001');
orEnvSetOptions('collision ode');
%orEnvSetOptions('debug debug');
