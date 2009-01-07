% R = RotationMatrixFromQuat(quat)
%
% quat - the format is [cos(angle/2) axis*sin(angle/2)]

% Software License Agreement (BSD License)
% Copyright (c) 2008, Rosen Diankov
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%   * Redistributions of source code must retain the above copyright notice,
%     this list of conditions and the following disclaimer.
%   * Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the distribution.
%   * Neither the name of Stanford University nor the names of its
%     contributors may be used to endorse or promote products derived from
%     this software without specific prior written permission.
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
function R = RotationMatrixFromQuat(quat)

R = zeros(3,3);
qq1 = 2*quat(2)*quat(2);
qq2 = 2*quat(3)*quat(3);
qq3 = 2*quat(4)*quat(4);
R(1,1) = 1 - qq2 - qq3;
R(1,2) = 2*(quat(2)*quat(3) - quat(1)*quat(4));
R(1,3) = 2*(quat(2)*quat(4) + quat(1)*quat(3));
R(2,1) = 2*(quat(2)*quat(3) + quat(1)*quat(4));
R(2,2) = 1 - qq1 - qq3;
R(2,3) = 2*(quat(3)*quat(4) - quat(1)*quat(2));
R(3,1) = 2*(quat(2)*quat(4) - quat(1)*quat(3));
R(3,2) = 2*(quat(3)*quat(4) + quat(1)*quat(2));
R(3,3) = 1 - qq1 - qq2;
