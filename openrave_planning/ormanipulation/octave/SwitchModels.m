%% success = SwitchModels(patterns, tofat)
%%
%% switches the models of all objects specified. No padded models need to be
%% added to the environment prior to this call! The function will add the
%% models if they are missing.
%% pattern is a cell array where each member holds
%% pattern - regular expression to match (for example 'mug\d$')
%% fatfilename - use this filename

%% Software License Agreement (BSD License)
%% Copyright (c) 2008, Willow Garage, Inc.
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
%%
%% author: Rosen Diankov
function success = SwitchModels(patterns, tofat)
global probs

strcmd = 'SwitchModels ';
strpad = [' padded ' num2str(tofat)];

bodies = orEnvGetBodies();

bodynames = {};
for ibody = 1:length(bodies)
    bodynames{ibody} = bodies{ibody}.name;
end

for ibody = 1:length(bodies)
    for ipat = 1:length(patterns)
        if( ~isempty(regexp(bodies{ibody}.name, patterns{ipat}.pattern)) )
            % check if created, if not, create, then switch
            
            if( tofat && isempty(strmatch([bodies{ibody}.name 'fat'], bodynames, 'exact')) && ...
                        isempty(strmatch([bodies{ibody}.name 'thin'], bodynames, 'exact')) )
                % if there are corresponding thin objects, then don't create
                id = orEnvCreateKinBody([bodies{ibody}.name 'fat'], patterns{ipat}.fatfilename);
                display(['adding ' bodies{ibody}.name 'fat: ' num2str(id)]);
                orBodySetTransform(id, [0 100 0], [1 0 0 0]);
            end
            
            strcmd = [strcmd ' name ' bodies{ibody}.name strpad];
        end
    end
end

orProblemSendCommand(strcmd,probs.task);
success = 1;
