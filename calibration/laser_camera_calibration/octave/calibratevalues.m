%% [Tcamera, Tlaser, jointoffsets] = calibratevalues(calibdata, robot, Tcamerainit, Tlaserinit)
%%
%% The main optimization loop to calibrate the values, needs openraveros to perform the kinematics
%% Tcamerainit (optional) - 3x4 matrix of the initial camera frame
%% Tlaserinit (optional) - 3x4 matrix of the initial laser frame
function [Tcamera, Tlaser, jointoffsets] = calibratevalues(calibdata, robot, Tcamerainit, Tlaserinit)

tic;
links = orBodyGetLinks(robot.id);

%% find the link offsets
laser_tilt_mount_link = find(cellfun(@(x) strcmp(x,'laser_tilt_mount_link'), robot.linknames),1,'first');
laser_tilt_link = find(cellfun(@(x) strcmp(x,'laser_tilt_link'), robot.linknames),1,'first');
head_tilt_link = find(cellfun(@(x) strcmp(x,'head_tilt_link'), robot.linknames),1,'first');
stereo_link = find(cellfun(@(x) strcmp(x,'stereo_link'), robot.linknames),1,'first');
if( isempty(laser_tilt_mount_link) || isempty(laser_tilt_link) || isempty(head_tilt_link) || isempty(stereo_link) )
    error('links empty');
end

% joint offsets to calibrate
jointnames = {'head_pan_joint'};
joints = []; 
for i = 1:length(jointnames)
    joints(i) = find(cellfun(@(x) strcmp(x,jointnames{i}), robot.jointnames),1,'first');
end

for i = 1:length(calibdata)
    if( robot.dof ~= length(calibdata{i}.jointvalues) )
        error('robot dof not equal');
    end

    orBodySetJointValues(robot.id, calibdata{i}.jointvalues);
    calibdata{i}.links = orBodyGetLinks(robot.id);
end

%% get initial values
if( ~exist('Tcamerainit','var') || isempty(Tcamerainit) )
    Tcamerainit = inv([reshape(links(:,head_tilt_link),[3 4]); 0 0 0 1]) * [reshape(links(:,stereo_link),[3 4]); 0 0 0 1];
end

if( ~exist('Tlaserinit','var') || isempty(Tlaserinit) )
    Tlaserinit = inv([reshape(links(:,laser_tilt_mount_link),[3 4]); 0 0 0 1]) * [reshape(links(:,laser_tilt_link),[3 4]); 0 0 0 1];
end

Tcamerainitinv = inv([Tcamerainit(1:3,1:4); 0 0 0 1]);
Xinit = [AxisAngleFromRotation(Tcamerainitinv(1:3,1:3));Tcamerainitinv(1:3,4);...
         AxisAngleFromRotation(Tlaserinit(1:3,1:3));Tlaserinit(1:3,4);zeros(size(joints))];

x = zeros(length(calibdata),1);
y = x;
display('starting to optimize');
[f,X, kvg, iter] = leasqr(x,y, Xinit, @(data,X) calibdist(X,robot,calibdata,joints,laser_tilt_mount_link,head_tilt_link), 0.0001, 400);
Tcamera = inv([RotationMatrixFromAxisAngle(X(1:3)) X(4:6); 0 0 0 1]);
Tlaser = [RotationMatrixFromAxisAngle(X(7:9)) X(10:12); 0 0 0 1];
jointoffsets = zeros(robot.dof,1);
jointoffsets(joints) = X(13:end);
display(sprintf('total time for optimization: %f',toc));

function F = calibdist(X,robot,calibdata,joints,laser_tilt_mount_link,head_tilt_link)

Tinvcamera = [RotationMatrixFromAxisAngle(X(1:3)) X(4:6); 0 0 0 1];
Tlaser = [RotationMatrixFromAxisAngle(X(7:9)) X(10:12); 0 0 0 1];
jointoffsets = zeros(robot.dof,1);
jointoffsets(joints) = X(13:end);
inv(Tinvcamera)
Tlaser

F = zeros(length(calibdata),1);

for i = 1:length(calibdata)
    if( ~isempty(jointoffsets) )
        orBodySetJointValues(robot.id, calibdata{i}.jointvalues+jointoffsets);
        links = orBodyGetLinks(robot.id);
        Tlaser_tilt_mount_link = [reshape(links(:,laser_tilt_mount_link),[3 4]); 0 0 0 1];
        Thead_tilt_link = [reshape(links(:,head_tilt_link),[3 4]); 0 0 0 1];
    else
        Tlaser_tilt_mount_link = [reshape(calibdata{i}.links(:,laser_tilt_mount_link),[3 4]); 0 0 0 1];
        Thead_tilt_link = [reshape(calibdata{i}.links(:,head_tilt_link),[3 4]); 0 0 0 1];
    end
    
    Nlaserplane = transpose(Tinvcamera * inv(Thead_tilt_link) * Tlaser_tilt_mount_link * Tlaser) * calibdata{i}.Ncamera;
    if( abs(norm(Nlaserplane(1:3))-1) > 0.01 )
        error('wtf');
    end
    
    %% find the 'area' under the line using the plane as the 0
    %% first transform the line in plane space so that y axis is plane
    p1 = [calibdata{i}.laserline(1:2);0];
    p2 = [calibdata{i}.laserline(3:4);0];
    right = cross(Nlaserplane(1:3), cross(Nlaserplane(1:3),p1-p2));
    len = norm(right);
    if( len > 1e-4 )
        right = right / len;
        s1 = [right'*p1 Nlaserplane(1:3)'*p1+Nlaserplane(4)];
        s2 = [right'*p2 Nlaserplane(1:3)'*p2+Nlaserplane(4)];

        if( abs(s2(2)-s1(2)) > 0 )
            x0 = s1(1) - s1(2)*(s2(1)-s1(1))/(s2(2)-s1(2)); %% find the zero crossing 
            s1(1) = s1(1)-x0;
            s2(1) = s2(1)-x0;
            area1 = (s1(2)^2+s1(1)^2)*s1(2)/s1(1);
            area2 = (s2(2)^2+s2(1)^2)*s2(2)/s2(1);
            if( sign(s1(1)) ~= sign(s2(1)) )
                dist = area1 + area2; % two tris
            else
                dist = area1 - area2; % one cutoff tri
            end
        else
            dist = s1(2) * (s1(1)-s2(1));
        end

%         dist
%         clf();
%         plot([s1(1) s2(1)],[0 0],'r'); hold on;
%         plot([s1(1) s2(1)],[s1(2) s2(2)],'b','linewidth',3);
%         drawnow;
        F(i) = max(min(dist,1000),-1000);
    else
        F(i) = 1000;
    end
end
%F'
sqrt(sum(F.^2))
