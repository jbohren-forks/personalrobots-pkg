%% addexamplesdir(subdir)
%%
%% Adds the openrave examples directory
function addexamplesdir(subdir)

basepath = [];
if( isunix() )
    %% try getting from openrave-config
    [status,basepath] = system('openrave-config --prefix');
    basepath = strtrim(basepath);
end

if( isempty(basepath) )
    basepath = 'C:\Program Files\openrave';
end

if( ~exist('subdir','var') )
    subdir = [];
end

addpath(fullfile(basepath,'share','openrave','examples',subdir));
