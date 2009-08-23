% Auto-generated.  Do not edit!

% msg = compressed_video_streaming_binaryblob()
%
% binaryblob message type, fields include:
% uint8{} blob

% //! \htmlinclude binaryblob.msg.html
function msg = compressed_video_streaming_binaryblob()

msg = [];
msg.blob = [];
msg.md5sum_ = @compressed_video_streaming_binaryblob___md5sum;
msg.type_ = @compressed_video_streaming_binaryblob___type;
msg.serializationLength_ = @compressed_video_streaming_binaryblob___serializationLength;
msg.serialize_ = @compressed_video_streaming_binaryblob___serialize;
msg.deserialize_ = @compressed_video_streaming_binaryblob___deserialize;

function x = compressed_video_streaming_binaryblob___md5sum()
x = '686a5a6faa4b2c7d2070ef2a260d09e7';

function x = compressed_video_streaming_binaryblob___type()
x = 'compressed_video_streaming/binaryblob';

function l__ = compressed_video_streaming_binaryblob___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.blob) * (1);

function dat__ = compressed_video_streaming_binaryblob___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.blob), 'uint32');
fwrite(fid__, msg__.blob(:), 'uint8');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = compressed_video_streaming_binaryblob___deserialize(dat__, fid__)
msg__ = compressed_video_streaming_binaryblob();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.blob = fread(fid__, size__, 'uint8=>uint8');
if( file_created__ )
    fclose(fid__);
end
function l__ = compressed_video_streaming_binaryblob___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

