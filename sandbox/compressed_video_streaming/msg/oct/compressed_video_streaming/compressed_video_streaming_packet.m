% Auto-generated.  Do not edit!

% msg = compressed_video_streaming_packet()
%
% packet message type, fields include:
% uint8{} blob
% int32 bytes
% int32 b_o_s
% int32 e_o_s
% int64 granulepos
% int64 packetno

% //! \htmlinclude packet.msg.html
function msg = compressed_video_streaming_packet()

msg = [];
msg.blob = [];
msg.bytes = int32(0);
msg.b_o_s = int32(0);
msg.e_o_s = int32(0);
msg.granulepos = int64(0);
msg.packetno = int64(0);
msg.md5sum_ = @compressed_video_streaming_packet___md5sum;
msg.type_ = @compressed_video_streaming_packet___type;
msg.serializationLength_ = @compressed_video_streaming_packet___serializationLength;
msg.serialize_ = @compressed_video_streaming_packet___serialize;
msg.deserialize_ = @compressed_video_streaming_packet___deserialize;

function x = compressed_video_streaming_packet___md5sum()
x = 'd804434eb295ca184dd6d2e32479185c';

function x = compressed_video_streaming_packet___type()
x = 'compressed_video_streaming/packet';

function l__ = compressed_video_streaming_packet___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.blob) * (1) ...
    + 4 ...
    + 4 ...
    + 4 ...
    + 8 ...
    + 8;

function dat__ = compressed_video_streaming_packet___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.blob), 'uint32');
fwrite(fid__, msg__.blob(:), 'uint8');
c__ = c__ + fwrite(fid__, msg__.bytes, 'int32');
c__ = c__ + fwrite(fid__, msg__.b_o_s, 'int32');
c__ = c__ + fwrite(fid__, msg__.e_o_s, 'int32');
c__ = c__ + fwrite(fid__, msg__.granulepos, 'int64');
c__ = c__ + fwrite(fid__, msg__.packetno, 'int64');
if( c__ ~= 5 )
    error('some members of msg compressed_video_streaming:packet are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = compressed_video_streaming_packet___deserialize(dat__, fid__)
msg__ = compressed_video_streaming_packet();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.blob = fread(fid__, size__, 'uint8=>uint8');
msg__.bytes = fread(fid__,1,'int32=>int32');
msg__.b_o_s = fread(fid__,1,'int32=>int32');
msg__.e_o_s = fread(fid__,1,'int32=>int32');
msg__.granulepos = fread(fid__,1,'int64=>int64');
msg__.packetno = fread(fid__,1,'int64=>int64');
if( file_created__ )
    fclose(fid__);
end
function l__ = compressed_video_streaming_packet___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

