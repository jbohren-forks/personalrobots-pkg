function funcs = rosrecord()
% Provides tools to read messages from ROS bagfiles.
% Valid commands:
%   fid = rosrecord.open(filename);
%     * Open a ROS bagfile. Returns an fid
%   msgs = rosrecord.dumpTopic(fid, topic)
%     * Dumps all messages on a topic to a cell array
%   [topic, msg, t] = rosrecord.getNextMsg(fid)
%     * Gets the next message in the bagfile
%   rosrecord.close(fid)
%     * Closes the bagfile referenced by fid
%
  funcs.open       = @rosrecord_open;
  funcs.getNextMsg = @rosrecord_getNextMsg;
  funcs.close      = @rosrecord_close;
  funcs.dumpTopic  = @rosrecord_dumpTopic;
end    


function msg_list = rosrecord_dumpTopic(fid, topic)

  msg_list = { };
  total_count = 0;
  topic_count = 0;
  % Keep looping until end-of-bag
  while (true)
    [bag_topic, msg, t] = rosrecord_getNextMsg(fid);
    if (size(msg,1)==0)
      break;
    end

    %keyboard;
    
    if (strcmp(bag_topic, topic) == 1)
      msg_list = [msg_list, {msg}];
      printf('On msg #%u.  Saving as #%u\n', total_count, topic_count);
      fflush(stdout);
      topic_count = topic_count+1;
    else
      printf('On msg #%u.  Skipping [%s]\n', total_count, bag_topic);
      fflush(stdout);      
    end
    total_count = total_count+1;
  end
  

function fid = rosrecord_open(filename)
  f = fopen (filename, 'r');
  if (f < 0)
    error ('error opening input file %s', filename);
  else
    bag_header = fgetl (f);
    if (strcmp(bag_header, '#ROSRECORD V1.2'))
      fid = f;
    else
      error ('rosrecord.m only supports ROSRECORD V1.2, file version is %s', bag_header);
    end
  end
end

function [topic, msg, t] = rosrecord_getNextMsg(fid)
  
  got_message = false;

  while(!got_message)  
    if (feof(fid))
      topic = "" ;
      msg = [] ;
      t = 0.0 ;
      return
    end
  
    [fields, values] = grabHeader(fid) ;
    
    % Check if we failed getting the header. Exit if so.
    if (size(fields,1) == 0)
      topic = "";
      msg = [];
      t = 0.0;
      return;
    end
    
    
    % Find all fields with the name 'op'
    op_code_indices = find(cellfun(@(x) strcmp(x, 'op'), fields)) ;
    if (op_code_indices < 1)
      error('could not find op code field') ;
    end
    % Assume the first op-code field is one we want
    op_code_val = values{op_code_indices(1)} ;

    if (length(op_code_val) != 1)
      error('op code field invalid length') ;
    end
    
    if (op_code_val(1) == 1)
      % Just grab the data and throw it away
      data = grabData(fid) ;
      got_message = false ;
    elseif (op_code_val(1) == 2)
      % todo: add check to make sure that these fields exist
      topic = values{find(cellfun(@(x) strcmp(x, 'topic'), fields))(1)} ;
      type  = values{find(cellfun(@(x) strcmp(x, 'type'), fields))(1)} ;
      md5   = values{find(cellfun(@(x) strcmp(x, 'md5'), fields))(1)} ;
      
      % todo: convert time info into a better datatype
      secs = values{find(cellfun(@(x) strcmp(x, 'sec'), fields))(1)} ;
      nsecs = values{find(cellfun(@(x) strcmp(x, 'nsec'), fields))(1)} ;

      secs_numeric  = sum(secs .* 2.^((0:3)*8)) ;
      nsecs_numeric = sum(nsecs .* 2.^((0:3)*8)) ;
      
      t = secs_numeric + nsecs_numeric * 1e-9 ;
      

      [data_len, count] = fread(fid, 1, '*uint32') ;
      if (count < 1)
        error('Error getting length of data') ;
      end        

      octtype = get_message_class (type);
      msg_object = octtype ();
      
      %printf('Starting to deserialize\n') ;
      tmp = msg_object.deserialize_ ([], fid) ;
      msg = tmp ;

      got_message = true ;
    else
      error('Unknown opcode') ;

    
  end
    
end

function rosrecord_close(fid)
  fclose(fid); 
end

function data = grabData(fid)
  [data_len, count] = fread(fid, 1, '*uint32') ;
  if (count < 1)
    error('Error getting length of data') ;
  end

  [data, count] = fread(fid, data_len, '*uint8');
  if (count < data_len)
    error('Error reading in data')
  end
end


% Returns a cell array of field names, and a corresponding
% cell array of field values.
function [names, values] = grabHeader(fid)
  [header_len, count] = fread (fid, 1, '*uint32') ;
  if (count != 1)
    names = [];
    values = [];
    return;
  end

  header_start_pos = ftell (fid);
 
  names = { } ;
  values = { } ;
 
  % printf('Header Start=%u\n', header_start_pos) ;
 
  while (ftell(fid) < header_start_pos + header_len - 1)
    %printf('Before Grabbing Field = %u\n', ftell(fid)) ;
    [name, value] = grabField(fid) ;
    names{end+1} = name ;
    values{end+1} = value ;
    %printf('After Grabbing Field = %u\n', ftell(fid)) ;
  end
end

function [name, value] = grabField(fid)

  % Determine field length
  [field_len, count] = fread (fid, 1, '*uint32') ;
  if (count != 1)
    error('Error getting field length') ;
  end
  
  % Extract all the field data
  [raw, count] = fread(fid, field_len, '*uchar') ;
  raw_str = raw' ;
  if (count != field_len)
    error('Error getting field data') ;
  end
  
  % Extract the first equals sign
  all_equals_loc = find(raw_str == '=') ;
  if (length(all_equals_loc) < 1)
    error('Could not find equals sign') ;
  end
  equals_loc = all_equals_loc(1) ;
  
  % Split data into name and value
  name =  raw_str(1:(equals_loc-1)) ;
  value = raw_str(equals_loc+1:end) ;
  
end

function [package, base_type] = package_resource_name (name)
  idx = strfind (name, '/');
  if (isempty (idx))
    package = '';
    base_type = name;
  else
    package = name(1:idx-1);
    base_type = name(idx+1:end);
  end
end

function msg_class = get_message_class (datatype)
  [package, base_type] = package_resource_name (datatype);
  rosoct_add_msgs(package);
  msg_class = str2func (sprintf ('%s_%s', package, base_type));
end



