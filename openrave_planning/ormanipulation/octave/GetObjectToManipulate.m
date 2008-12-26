%% [curobj,outmemory] GetObjectToManipulate(scenedata, inmemory)
%%
%% finds the next object to manipulate

%% Software License Agreement (BSD License)
%% Copyright (c) 2006-2009, Rosen Diankov
%% Redistribution and use in source and binary forms, with or without
%% modification, are permitted provided that the following conditions are met:
%%   * Redistributions of source code must retain the above copyright notice,
%%     this list of conditions and the following disclaimer.
%%   * Redistributions in binary form must reproduce the above copyright
%%     notice, this list of conditions and the following disclaimer in the
%%     documentation and/or other materials provided with the distribution.
%%   * The name of the author may not be used to endorse or promote products
%%     derived from this software without specific prior written permission.
%%
%% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%% POSSIBILITY OF SUCH DAMAGE.
function [curobj,outmemory] = GetObjectToManipulate(robot, scenedata, inmemory)
global updir

targetobjs = {};
curobj = [];
outmemory = inmemory;

%% analyze the scene for possible objects to grab
bodies = orEnvGetBodies();
bodynames = {};
for ibody = 1:length(bodies)
    bodynames{ibody} = bodies{ibody}.name;
end
    
%% add new bodies
for ipad = 1:length(scenedata.TargetObjPattern)
    found = {};
    for j = 1:length(bodynames)
        found{j} = regexp(bodynames{j}, scenedata.TargetObjPattern{ipad}.pattern);
    end
    
    found = regexp(bodynames, scenedata.TargetObjPattern{ipad}.pattern);
    for i = 1:length(found)
        if( ~isempty(found{i}) )
            newobj = scenedata.TargetObjPattern{ipad};
            newobj.id = bodies{i}.id;
            newobj.info = bodies{i}; % copy the rest of the info
            targetobjs{end+1} = newobj;
        end
    end
end

% choose the current target somehow (closest to robot)
dists = [];
Trobot = reshape(orBodyGetTransform(robot.id),[3 4]);

for i = 1:length(targetobjs)
    dists(i) = sum( (newobj.info.T(1:3,4)-Trobot(1:3,4)).^2 );
end

[dists, orderedtargets] = sort(dists);

%% get the first target not in the ignore list
for i = 1:length(orderedtargets)
    if( isempty(find(outmemory.ignorelist==targetobjs{orderedtargets(i)}.id)) )
        curobj = targetobjs{orderedtargets(i)};
        break;
    end
end

if( isempty(curobj) && ~isempty(outmemory.ignorelist) && ~isempty(orderedtargets) )
    %% reset the ignore list
    outmemory.ignorelist = [];
    curobj = targetobjs{orderedtargets(1)};
end

[dests, surfaceplane] = scenedata.GetDestsFn(); % compute a set of destinations

offsetfromtable = 0.02; %% set destination a little above the table
distup = surfaceplane(1:3)*(transpose(surfaceplane(1:3))*curobj.info.T(1:3,4) + surfaceplane(4) + offsetfromtable);

curobj.dests = zeros(size(dests));
for i = 1:size(dests,2)
    Td = reshape(dests(:,i),[3 4]);
    Rnew = Td(1:3,1:3)*curobj.info.T(1:3,1:3); % preserve the rotation
    pos = Td(1:3,4) + distup;
    curobj.dests(:,i) = [Rnew(:);pos];
end

%% for now
outmemory.ignorelist(end+1) = curobj.id;
