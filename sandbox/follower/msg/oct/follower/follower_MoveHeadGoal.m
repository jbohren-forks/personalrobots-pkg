% Auto-generated.  Do not edit!

% msg = follower_MoveHeadGoal()
%
% MoveHeadGoal message type, fields include:

% //! \htmlinclude MoveHeadGoal.msg.html
function msg = follower_MoveHeadGoal()

msg = [];
msg.md5sum_ = @follower_MoveHeadGoal___md5sum;
msg.type_ = @follower_MoveHeadGoal___type;
msg.serializationLength_ = @follower_MoveHeadGoal___serializationLength;
msg.serialize_ = @follower_MoveHeadGoal___serialize;
msg.deserialize_ = @follower_MoveHeadGoal___deserialize;

function x = follower_MoveHeadGoal___md5sum()
x = 'd41d8cd98f00b204e9800998ecf8427e';

function x = follower_MoveHeadGoal___type()
x = 'follower/MoveHeadGoal';

function l__ = follower_MoveHeadGoal___serializationLength(msg)
l__ = 0;

function dat__ = follower_MoveHeadGoal___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = follower_MoveHeadGoal___deserialize(dat__, fid__)
msg__ = follower_MoveHeadGoal();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
if( file_created__ )
    fclose(fid__);
end
function l__ = follower_MoveHeadGoal___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

