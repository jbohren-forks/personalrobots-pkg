global updir probs
cd(fullfile(rosoct_findpackage('ormanipulation'),'octave'));

startup;
[robot, scenedata] = SetupTableScene('data/pr2table.env.xml');

RunDynamicGrasping(robot, scenedata, 1,0);

%% squeeze testing
robot = RobotCreateIntel('BarrettWAM', 1); % create a 4dof robot
problem = orEnvCreateProblem('Manipulation BarrettWAM');

v = orRobotGetDOFValues(robot.id)
v(end) = 0;
orRobotSetDOFValues(robot.id, v);

orProblemSendCommand('squeeze');
orRobotControllerSend(robot.id, 'ignoreproxy 1');

orRobotSetDOFValues(robot.id, v);

%% run the grasps
grasps = curobj.grasps;
TargTrans = reshape(orBodyGetLinks(targetid), [3 4]);
grasps(:,robothand.grasp.direction) = grasps(:,robothand.grasp.direction) * TargTrans(:,1:3)';
grasps(:,robothand.grasp.center) = grasps(:,robothand.grasp.center) * TargTrans(:,1:3)' + repmat(TargTrans(:,4)', [size(grasps,1) 1]);

Tobjs = {};
robothand = robot.CreateHandFn(['hand']);
for i = 1:length(goodgrasps)
    contactsraw = RunGrasp(robothand, grasps(goodgrasps(i),:), Target,0,0);
    Tobjs{end+1}.T = reshape(orBodyGetTransform(robothand.id),[3 4]);
    Tobjs{end}.V = orBodyGetJointValues(robothand.id);
end

%robothand = robot.CreateHandFn(['temp' num2str(i)]);   
for i = 1:length(Tobjs)
    robothand = robot.CreateHandFn(['testhand' num2str(i)]);   
    orBodySetTransform(robothand.id, Tobjs{i}.T);
    orRobotSetDOFValues(robothand.id, Tobjs{i}.V);
    pause(0.5);
end

Tobjs = {};
for i = 1:size(grasps,1)
    % test the grasp planner
    %robothand = robot.CreateHandFn(['testhand' num2str(i)]);
    contactsraw = RunGrasp(robothand, grasps(i,:), Target,0,0);
    %Tobjs{end+1} = reshape(orBodyGetTransform(robothand.id),[3 4])
    %orProblemSendCommand(['triangulate hrp2_g' num2str(length(Tobjs)) '.txt']);
    pause;
end
