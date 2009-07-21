% Auto-generated.  Do not edit!

% msg = follower_WaitActionGoal()
%
% WaitActionGoal message type, fields include:
% int32 num_events
% string topic_name

% //! \htmlinclude WaitActionGoal.msg.html
function msg = follower_WaitActionGoal()

msg = [];
msg.num_events = int32(0);
msg.topic_name = '';
msg.md5sum_ = @follower_WaitActionGoal___md5sum;
msg.type_ = @follower_WaitActionGoal___type;
msg.serializationLength_ = @follower_WaitActionGoal___serializationLength;
msg.serialize_ = @follower_WaitActionGoal___serialize;
msg.deserialize_ = @follower_WaitActionGoal___deserialize;

function x = follower_WaitActionGoal___md5sum()
x = '54f5dc6d242ed96aa3e20c82006143e4';

function x = follower_WaitActionGoal___type()
x = 'follower/WaitActionGoal';

function l__ = follower_WaitActionGoal___serializationLength(msg)
l__ =  ...
    + 4 ...
    + 4 + numel(msg.topic_name);

function dat__ = follower_WaitActionGoal___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.num_events, 'int32');
fwrite(fid__, numel(msg__.topic_name), 'uint32');
fwrite(fid__, msg__.topic_name, 'uint8');
if( c__ ~= 1 )
    error('some members of msg follower:WaitActionGoal are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = follower_WaitActionGoal___deserialize(dat__, fid__)
msg__ = follower_WaitActionGoal();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.num_events = fread(fid__,1,'int32=>int32');
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.topic_name = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = follower_WaitActionGoal___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

