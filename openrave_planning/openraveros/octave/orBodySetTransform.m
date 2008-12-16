% success=orBodySetTransform(bodyid, translation, quaternion)
% success=orBodySetTransform(bodyid, [quaternion translation])
% success=orBodySetTransform(bodyid, transform matrix) (12x1, 1x12, or 3x4)
%
% Set the affine transformation of the body. The transformation actually describes the first
% link of the body. The rest of the links are derived by the joint angles.
% a quaternion is related to axis and angle via: [cos(theta/2);sin(theta/2)*axis]
function success = orBodySetTransform(varargin)
session = openraveros_getglobalsession();
req = openraveros_body_settransform();
req.bodyid = varargin{1};

if(nargin >= 3)
    R = openraveros_rotfromquat(varargin{3});
    req.transform.m(1:9) = R(:);
    req.transform.m(10:12) = trans(:);
elseif(nargin == 2)
    t = varargin{2};
    req.transform.m(1:12) = t(:);
else
    error('orBodySetTransform not enough arguments');
end

res = rosoct_session_call(session.id,'body_settransform',req);
success = ~isempty(res);
