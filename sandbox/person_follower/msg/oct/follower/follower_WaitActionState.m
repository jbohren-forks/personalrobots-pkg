% Auto-generated.  Do not edit!

% msg = follower_WaitActionState()
%
% WaitActionState message type, fields include:
%   roslib_Header header
%   robot_actions_ActionStatus status
%   follower_WaitActionGoal goal
%   std_msgs_Empty feedback

% //! \htmlinclude WaitActionState.msg.html
function msg = follower_WaitActionState()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/u/ethand/ros/ros-pkg/sandbox/follower/msg/oct/follower');
    addpath('/u/ethand/ros/ros-pkg/stacks/common/robot_actions/msg/oct/robot_actions');
    addpath('/u/ethand/ros/ros/core/roslib/msg/oct/roslib');
    addpath('/u/ethand/ros/ros/std_msgs/msg/oct/std_msgs');
end


msg = [];
msg.header = roslib_Header();
msg.status = robot_actions_ActionStatus();
msg.goal = follower_WaitActionGoal();
msg.feedback = std_msgs_Empty();
msg.md5sum_ = @follower_WaitActionState___md5sum;
msg.type_ = @follower_WaitActionState___type;
msg.serializationLength_ = @follower_WaitActionState___serializationLength;
msg.serialize_ = @follower_WaitActionState___serialize;
msg.deserialize_ = @follower_WaitActionState___deserialize;

function x = follower_WaitActionState___md5sum()
x = '3da8cfb44ffcd3ffb1b26035cdd0dbb5';

function x = follower_WaitActionState___type()
x = 'follower/WaitActionState';

function l__ = follower_WaitActionState___serializationLength(msg)
l__ =  ...
    + msg.header.serializationLength_(msg.header) ...
    + msg.status.serializationLength_(msg.status) ...
    + msg.goal.serializationLength_(msg.goal) ...
    + msg.feedback.serializationLength_(msg.feedback);

function dat__ = follower_WaitActionState___serialize(msg__, seq__, fid__)
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
msg__.status.serialize_(msg__.status, seq__, fid__);
msg__.goal.serialize_(msg__.goal, seq__, fid__);
msg__.feedback.serialize_(msg__.feedback, seq__, fid__);
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = follower_WaitActionState___deserialize(dat__, fid__)
msg__ = follower_WaitActionState();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.header = roslib_Header();
msg__.header = msg__.header.deserialize_(msg__.header, fid__);
msg__.status = robot_actions_ActionStatus();
msg__.status = msg__.status.deserialize_(msg__.status, fid__);
msg__.goal = follower_WaitActionGoal();
msg__.goal = msg__.goal.deserialize_(msg__.goal, fid__);
msg__.feedback = std_msgs_Empty();
msg__.feedback = msg__.feedback.deserialize_(msg__.feedback, fid__);
if( file_created__ )
    fclose(fid__);
end
function l__ = follower_WaitActionState___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

