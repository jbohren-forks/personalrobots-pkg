% Auto-generated.  Do not edit!

% msg = people_PositionMeasurement()
%
% PositionMeasurement message type, fields include:
%   roslib_Header header
% string name
% string object_id
%   robot_msgs_Point pos
% double reliability
% double{} covariance
% int8 initialization

% //! \htmlinclude PositionMeasurement.msg.html
function msg = people_PositionMeasurement()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/u/pantofaru/Install/ros/ros-pkg/common/robot_msgs/msg/oct/robot_msgs');
    addpath('/u/pantofaru/Install/ros/ros/core/roslib/msg/oct/roslib');
end


msg = [];
msg.header = roslib_Header();
msg.name = '';
msg.object_id = '';
msg.pos = robot_msgs_Point();
msg.reliability = double(0);
msg.covariance = zeros(9, 1, 'double');
msg.initialization = int8(0);
msg.md5sum_ = @people_PositionMeasurement___md5sum;
msg.type_ = @people_PositionMeasurement___type;
msg.serializationLength_ = @people_PositionMeasurement___serializationLength;
msg.serialize_ = @people_PositionMeasurement___serialize;
msg.deserialize_ = @people_PositionMeasurement___deserialize;

function x = people_PositionMeasurement___md5sum()
x = '9da2997e69ab83d26b742b4c5d4d0d04';

function x = people_PositionMeasurement___type()
x = 'people/PositionMeasurement';

function l__ = people_PositionMeasurement___serializationLength(msg)
l__ =  ...
    + msg.header.serializationLength_(msg.header) ...
    + 4 + numel(msg.name) ...
    + 4 + numel(msg.object_id) ...
    + msg.pos.serializationLength_(msg.pos) ...
    + 8 ...
    + 9 * (8) ...
    + 1;

function dat__ = people_PositionMeasurement___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
if (msg__.header.seq == 0)
    msg__.header.seq = seq__;
end
if (msg__.header.stamp.sec == 0 && msg__.header.stamp.nsec == 0)
    msg__.header.stamp = rosoct_time_now();
end
msg__.header.serialize_(msg__.header, seq__, fid__);
fwrite(fid__, numel(msg__.name), 'uint32');
fwrite(fid__, msg__.name, 'uint8');
fwrite(fid__, numel(msg__.object_id), 'uint32');
fwrite(fid__, msg__.object_id, 'uint8');
msg__.pos.serialize_(msg__.pos, seq__, fid__);
c__ = c__ + fwrite(fid__, msg__.reliability, 'double');
c__ = c__ + fwrite(fid__, msg__.covariance(1:9), 'double');
c__ = c__ + fwrite(fid__, msg__.initialization, 'int8');
if( c__ ~= 11 )
    error('some members of msg people:PositionMeasurement are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = people_PositionMeasurement___deserialize(dat__, fid__)
msg__ = people_PositionMeasurement();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.header = roslib_Header();
msg__.header = msg__.header.deserialize_(msg__.header, fid__);
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.name = fread(fid__, size__, '*char')';
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.object_id = fread(fid__, size__, '*char')';
msg__.pos = robot_msgs_Point();
msg__.pos = msg__.pos.deserialize_(msg__.pos, fid__);
msg__.reliability = fread(fid__,1,'double=>double');
msg__.covariance(1:9) = fread(fid__, 9, 'double=>double');
msg__.initialization = fread(fid__,1,'int8=>int8');
if( file_created__ )
    fclose(fid__);
end
function l__ = people_PositionMeasurement___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

