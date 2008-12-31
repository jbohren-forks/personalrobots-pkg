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
%filterserv Parses arguments for octave filtering functions.

res = req._create_response();
method = req.name;

switch method
  case{'butter'} 
    if(length(req.args)==2) %defualt lowpass
      [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}));
    elseif(length(req.args)==3) 
      if(strcmp(req.args{3},'high') || strcmp(req.args{3},'h') || strcmp(req.args{3},'low') || strcmp(req.args{3},'l'))%select type
        [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}), req.args{3});
      else %bandstop
        [b,a]=butter(str2num(req.args{1}),[str2num(req.args{2}), str2num(req.args{3})]);
      endif
    elseif(length(req.args)==4) %bandstop
      [b,a]=butter(str2num(req.args{1}),[str2num(req.args{2}) str2num(req.args{3})], req.args{4});
    else %wrong number of arguments
      res=[];
    end
  case{'cheby1'}
    if(length(req.args)==3) %defualt lowpass
      [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}));
    elseif(length(req.args)==4) 
      if(strcmp(req.args{4},'high') || strcmp(req.args{4},'h') || strcmp(req.args{4},'low') || strcmp(req.args{4},'l'))%select type
        [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}), req.args{4});
      else %bandstop
        [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}), [str2num(req.args{3}), str2num(req.args{4})]);
      endif
    elseif(length(req.args)==5) %bandstop
      [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}),[str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else %wrong number of arguments
      res=[];
    end
  case{'cheby2'}
    if(length(req.args)==3) %defualt lowpass
      [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}));
    elseif(length(req.args)==4)
      if(strcmp(req.args{4},'high') || strcmp(req.args{4},'h') || strcmp(req.args{4},'low') || strcmp(req.args{4},'l'))%select type
        [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}), req.args{4});
      else %bandstop
        [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}), [str2num(req.args{3}), str2num(req.args{4})]);
      endif
    elseif(length(req.args)==5) %bandstop
      [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}),[str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else %wrong number of arguments
      res=[];
    end
  case{'ellip'}
    if(length(req.args)==4) %defualt lowpass
      'here'
      [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),str2num(req.args{4}));
    elseif(length(req.args)==5) 
      if(strcmp(req.args{5},'high') || strcmp(req.args{5},'h') || strcmp(req.args{5},'low') || strcmp(req.args{5},'l'))%select type
        [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}),str2num(req.args{4}), req.args{5});
      elseif %bandstop
        [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}),[str2num(req.args{4}), str2num(req.args{5})]);
      endif
    elseif(length(req.args)==6) %bandstop
      [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),[str2num(req.args{4}) str2num(req.args{5})], req.args{6}); 
    else %wrong number of arguments
      res=[];
    end
  otherwise %filter doesn't exist
    res=[];
end
res.a=a;
res.b=b; 
