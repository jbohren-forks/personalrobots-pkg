function testrosik()

addopenrave();

orEnvLoadScene('',1);
robotid = orEnvCreateRobot('pr2','robots/pr2full.robot.xml');
probid = orEnvCreateProblem('basemanipulation','pr2');

manips = orRobotGetManipulators(robotid);

%% test any specific ik configuration
% orBodySetJointValues(robotid,[ -0.503115 0.573333 -2.99203 0.365955 1.95347 2.04003 -0.175888 ],manips{1}.armjoints);
% links = orBodyGetLinks(robotid);
% Thand = reshape(links(:,manips{1}.eelink),[3 4]);
% s = orProblemSendCommand(['iktest matrix ' sprintf('%f ',Thand(:))]);
% s
% if( isempty(s) )
%     return;
% end

%% left arm
orProblemSendCommand('SetActiveManip 0')
tic;
orProblemSendCommand('debugik numtests 200',probid);
toc

%% right arm
orProblemSendCommand('SetActiveManip 1')
orProblemSendCommand('debugik numtests 200',probid);
