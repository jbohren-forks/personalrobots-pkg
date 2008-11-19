function testrosik()

addopenrave();

orEnvLoadScene('',1);
robotid = orEnvCreateRobot('pr2','robots/pr2full.robot.xml');
probid = orEnvCreateProblem('basemanipulation','pr2');

manips = orRobotGetManipulators(robotid);

% orBodySetJointValues(robotid,[ -0.21 0.754256 0.40712 0.323661 1.28898 0.175237 2.13528],manips{1}.armjoints);
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
orProblemSendCommand('debugik numtests 100',probid);
toc

%% right arm
% orProblemSendCommand('SetActiveManip 1')
% orProblemSendCommand('debugik numtests 1',probid);
