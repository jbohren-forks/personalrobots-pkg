function action = readAction(fid)
    action.rv = fread(fid, 1, 'double');
    action.tv = fread(fid, 1, 'double');
    action.time = fread(fid, 1, 'int');
    action.path_length = fread(fid, 1, 'uint');
    for pind = 1:action.path_length
        action.path(pind).x = fread(fid, 1, 'int');
        action.path(pind).y = fread(fid, 1, 'int');
        action.path(pind).theta = fread(fid, 1, 'int');
    
    end
    
    action.path_length_rw = fread(fid, 1, 'uint');
    for pind = 1:action.path_length_rw
        action.path_rw(pind).x = fread(fid, 1, 'double');
        action.path_rw(pind).y = fread(fid, 1, 'double');
        action.path_rw(pind).theta = fread(fid, 1, 'double');
    end

    action.footprint_size = fread(fid, 1, 'uint');
    for find = 1:action.footprint_size
        action.footprint(find).x = fread(fid, 1, 'double');
        action.footprint(find).y = fread(fid, 1, 'double');
    end