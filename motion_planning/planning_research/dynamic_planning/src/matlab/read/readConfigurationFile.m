function cfg = readConfigurationFile(cfgFile)

fid = fopen(cfgFile, 'r');

c = textscan(fid, 'discretization(cells): %d %d');
cfg.width = c{1};
cfg.height = c{2};

c = textscan(fid, 'cellsize(meters): %f');
cfg.cellsize = c{1};

c = textscan(fid, 'num_theta: %d')
cfg.num_theta = c{1};

c = textscan(fid, 'tv(m/s): %d');
cfg.tv = c{1};

c = textscan(fid, 'max_rv(rad/s): %f');
cfg.max_rv = c{1};
    
c = textscan(fid, 'num_rv: %d');
cfg.num_rv = c{1};

c = textscan(fid, 'short_long_dur_for_actions: %f %f');
cfg.short_dur = c{1};
cfg.long_dur = c{2};

c = textscan(fid, 'dur_disc(s): %f');
cfg.dur_disc = c{1};

c = textscan(fid, 'start(m,rad): %f %f %f');
cfg.start_x = c{1};
cfg.start_y = c{2};
cfg.start_theta = c{3};

c = textscan(fid, 'end(m,rad): %f %f %f');
cfg.end_x = c{1};
cfg.end_y = c{2};
cfg.end_theta = c{3};

c = textscan(fid, 'num_footprint_corners: %d');
cfg.num_footprint_corners = c{1};

c = textscan(fid, 'footprint:');
if(cfg.num_footprint_corners)
    c = textscan(fid, '%f %f');
    cfg.footprint = [c{1} c{2}];
    cfg.footprint = [cfg.footprint; cfg.footprint(1,:)];
end

c = textscan(fid, 'environment:');
for i=1:cfg.height
    c = textscan(fid, '%d ', cfg.width);
    cfg.environment(i,:) = c{:};
end

fclose(fid);