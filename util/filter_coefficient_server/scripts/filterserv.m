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

function res = filterserv(req)
%filterserv - Parses arguments for octave filtering functions.

res = req._create_response();
method = req.name;

switch method
  % Parsing for butter
  case{'butter'} 
    if(length(req.args) == 2) % Defualt lowpass
      [res.b, res.a] = butter(str2num(req.args{1}), str2num(req.args{2}));
    elseif(length(req.args) == 3) 
      if(strcmp(req.args{3}, 'high') || strcmp(req.args{3}, 'low'))% Select type
        [res.b,res.a] = butter(str2num(req.args{1}), str2num(req.args{2}), req.args{3});
      else % Bandstop
        [res.b, res.a] = butter(str2num(req.args{1}), [str2num(req.args{2}) str2num(req.args{3})]);
      endif
    elseif(length(req.args) == 4) % Bandstop
      [res.b, res.a]=butter(str2num(req.args{1}), [str2num(req.args{2}) str2num(req.args{3})], req.args{4});
    else % Wrong number of arguments
      res = [];
    endif
  % Parsing for cheby1  
  case{'cheby1'}
    if(length(req.args) == 3) % Defualt lowpass
      [res.b, res.a] = cheby1(str2num(req.args{1}), str2num(req.args{2}), str2num(req.args{3}));
    elseif(length(req.args) == 4) 
      if(strcmp(req.args{4}, 'high') || strcmp(req.args{4}, 'low'))% Select type
        [res.b, res.a] = cheby1(str2num(req.args{1}), str2num(req.args{2}), str2num(req.args{3}), req.args{4});
      else % Bandstop
        [res.b, res.a] = cheby1(str2num(req.args{1}), str2num(req.args{2}), [str2num(req.args{3}) str2num(req.args{4})]);
      endif
    elseif(length(req.args) == 5) % Bandstop
      [res.b, res.a] = cheby1(str2num(req.args{1}), str2num(req.args{2}), [str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else % Wrong number of arguments
      res = [];
    endif
  % Parsing for cheby2  
  case{'cheby2'}
    if(length(req.args) == 3) % Defualt lowpass
      [res.b,res.a]=cheby2(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}));
    elseif(length(req.args) == 4)
      if(strcmp(req.args{4}, 'high') || strcmp(req.args{4}, 'low'))% Select type
        [res.b, res.a] = cheby2(str2num(req.args{1}), str2num(req.args{2}), str2num(req.args{3}), req.args{4});
      else % Bandstop
        [res.b, res.a] = cheby2(str2num(req.args{1}), str2num(req.args{2}), [str2num(req.args{3}) str2num(req.args{4})]);
      endif
    elseif(length(req.args) == 5) % Bandstop
      [res.b, res.a] = cheby2(str2num(req.args{1}), str2num(req.args{2}), [str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else % Wrong number of arguments
      res = [];
    endif
  % Parsing for ellip 
  case{'ellip'}
    if(length(req.args) == 4) % Defualt lowpass
      [res.b, res.a]=ellip(str2num(req.args{1}), str2num(req.args{2}), str2num(req.args{3}), str2num(req.args{4}));
    elseif(length(req.args) == 5) 
      if(strcmp(req.args{5}, 'high') || strcmp(req.args{5}, 'low'))% Select type
        [res.b, res.a] = ellip(str2num(req.args{1}), str2num(req.args{2}), str2num(req.args{3}), str2num(req.args{4}), req.args{5});
      elseif % Bandstop
        [res.b, res.a] = ellip(str2num(req.args{1}), str2num(req.args{2}), str2num(req.args{3}), [str2num(req.args{4}) str2num(req.args{5})]);
      endif
    elseif(length(req.args) == 6) % Bandstop
      [res.b, res.a] = ellip(str2num(req.args{1}), str2num(req.args{2}), str2num(req.args{3}), [str2num(req.args{4}) str2num(req.args{5})], req.args{6}); 
    else % Wrong number of arguments
      res = [];
    endif
  otherwise %filter doesn't exist
    res = [];
endswitch

