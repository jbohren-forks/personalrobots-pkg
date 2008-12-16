%% openraveros_restart(sessionserver, viewer)
%%
%% restars an openraveros session if the current one is invalid

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
function openraveros_restart(sessionserver,viewer)
global openraveros_globalsession

openraveros_startup();

if( ~isempty(openraveros_globalsession) )
    %% send a dummy env_set command
    res = rosoct_session_call(openraveros_globalsession.id,'env_set',openraveros_env_set());
    if( ~isempty(res) )
        %% success
        return;
    end
end

if( ~exist('sessionserver','var') )
    sessionserver = 'openrave_session';
end
openraveros_globalsession = openraveros_createsession(sessionserver);

if( ~exist('viewer','var') )
    viewer = 'qtcoin';
end

if( ~isempty(viewer) && ~isempty(openraveros_globalsession) )
    %% set the viewer
    reqset = openraveros_env_set();
    reqset.setmask = reqset.Set_Viewer();
    reqset.viewer = viewer;
    resset = rosoct_session_call(openraveros_globalsession.id,'env_set',reqset);
end
