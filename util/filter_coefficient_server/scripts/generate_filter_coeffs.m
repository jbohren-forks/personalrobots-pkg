#!/usr/bin/env octave
%/
% Copyright (c) 2008, Willow Garage, Inc.
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
%     % Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     % Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     % Neither the name of the Willow Garage, Inc. nor the names of its
%       contributors may be used to endorse or promote products derived from
%       this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%/

% Original version: Melonee Wise <mwise@willowgarage.com>

% Adds the startup script to the path for roslaunching
[status,rosoctpath] = system('rospack find filter_coefficient_server');
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, '/scripts'));
startup;

__rosoct_unadvertise_service('generate_filter_coeffs');
% Creates the service
suc = rosoct_advertise_service('generate_filter_coeffs',@filter_coefficient_server_Filter,@filterserv);

if( ~suc )
    error('failed to advertise service!');
end

% Loops to keep the octave session going
while(1)
  __rosoct_worker();
end
