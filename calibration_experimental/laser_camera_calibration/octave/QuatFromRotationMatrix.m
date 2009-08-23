% quat = QuatFromRotationMatrix(R)
%
% R - 3x3 orthogonal rotation matrix
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
function quat = QuatFromRotationMatrix(R)

quat = zeros(4,1);
tr = R(1,1) + R(2,2) + R(3,3);
if (tr >= 0)
    s = sqrt(tr + 1);
    quat(1) = 0.5*s;
    quat(2:4) = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1) - R(1,2)]*0.5/s;
else
    %% find the largest diagonal element and jump to the appropriate case
    [rmax, convcase] = max([R(1,1) R(2,2) R(3,3)]);
    switch(convcase)
        case 1
            s = sqrt((R(1,1) - (R(2,2) + R(3,3))) + 1);
            quat(2) = 0.5 * s;
            s = 0.5 / s;
            quat(3) = (R(1,2) + R(2,1)) * s;
            quat(4) = (R(3,1) + R(1,3)) * s;
            quat(1) = (R(3,2) - R(2,3)) * s;
        case 2
            s = sqrt((R(2,2) - (R(3,3) + R(1,1))) + 1);
            quat(3) = 0.5 * s;
            s = 0.5 / s;
            quat(4) = (R(2,3) + R(3,2)) * s;
            quat(2) = (R(1,2) + R(2,1)) * s;
            quat(1) = (R(1,3) - R(3,1)) * s;
        case 3
            s = sqrt((R(3,3) - (R(1,1) + R(2,2))) + 1);
            quat(4) = 0.5 * s;
            s = 0.5 / s;
            quat(2) = (R(3,1) + R(1,3)) * s;
            quat(3) = (R(2,3) + R(3,2)) * s;
            quat(1) = (R(2,1) - R(1,2)) * s;
    end
end
