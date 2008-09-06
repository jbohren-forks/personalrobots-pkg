% ADDSVLPATHS Add STAIR Vision Library paths to Matlab.
% Stephen Gould <sgould@stanford.edu>
% 
% Execute as: run('svl/scripts/addSVLPaths');
%

baseDir = [pwd, '/../..'];

warning('off', 'MATLAB:dispatcher:pathWarning');
path(path, [baseDir, '/svl/scripts']);
path(path, [baseDir, '/bin']);
warning('on', 'MATLAB:dispatcher:pathWarning');
