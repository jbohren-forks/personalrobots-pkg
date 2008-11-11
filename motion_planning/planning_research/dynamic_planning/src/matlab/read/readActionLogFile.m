function actions = readActionLogFile(logFile)
    
    fid = fopen(logFile, 'r');


   while(~feof(fid))
        theta_idx = fread(fid, 1, 'uint');
        num_actions = fread(fid, 1, 'uint');
        for aind = 1:num_actions
            actions(theta_idx+1).action_list(aind) = readAction(fid);
        end
   end
    
    fclose(fid);
