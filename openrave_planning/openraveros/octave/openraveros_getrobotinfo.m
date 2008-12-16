%% robot = openraveros_getrobotinfo(robotinfo)
%%
%% parses the RobotInfo.msg into a simpler form for octave consumption
function robot = openraveros_getrobotinfo(robotinfo)

robot = openraveros_getbodyinfo(robotinfo.bodyinfo);

robot.manips = cell(length(robotinfo.manips),1);
for i = 1:length(robot.manips)
    mi = robotinfo.manips{i};
    robot.manips{i} = struct('baselink',mi.baselink+1,...
                             'eelink',mi.eelink+1,...
                             'Tgrasp',reshape(cell2mat(mi.tgrasp.m),[3 4]),...
                             'handjoints',cell2mat(mi.handjoints),...
                             'armjoints',cell2mat(mi.armjoints),...
                             'iksolvername',mi.iksolvername);
end

robot.sensors = cell(length(robotinfo.sensors),1);
for i = 1:length(robot.sensors)
    si = robotinfo.sensors{i};
    robot.sensors{i} = struct('name',si.name,...
                              'attachedlink',si.attachedlink+1,...
                              'Trelative',reshape(cell2mat(si.trelative.m),[3 4]),...
                              'Tglobal',reshape(cell2mat(si.tglobal.m),[3 4]),...
                              'type',si.type);
end

robot.activedof = robotinfo.activedof;
robot.affinedof = robotinfo.active.affine;
robot.activejoints = cell2mat(robotinfo.active.joints);
robot.rotationaxis = cell2mat(robotinfo.active.rotationaxis);

robot.activelowerlimit = cell2mat(robotinfo.activelowerlimit);
robot.activeupperlimit = cell2mat(robotinfo.activeupperlimit);

