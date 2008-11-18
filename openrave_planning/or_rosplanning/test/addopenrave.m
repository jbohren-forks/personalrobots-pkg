function addopenrave()

[status,orprefix] = system('openrave-config --prefix');
orprefix = strtrim(orprefix);
if( exist('OCTAVE_VERSION') ~= 0 )
    addpath(fullfile(orprefix,'share','openrave','octave'));
else
    addpath(fullfile(orprefix,'share','openrave','matlab'));
end
