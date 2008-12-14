% data = orRobotSensorGetData(robotid, sensorindex)
%
% Gets the sensor data. The format returned is dependent on the type
% of sensor. Look at the different data SensorData implementations in rave.h.
% Although the data returned is not necessarily one of them.

% data.type - contains the id of the data type (see SensorBase::SensorType)
% For laser data
%  data.laserrange - 3xN array where each column is the direction * distance
%  data.laserpos - 3xN array where each column is the corresponding origin of each range measurement
%  data.laserint - 1xN optional laser intensity array
% For image data
%  data.KK - 3x3 intrinsic matrix
%  data.T - 3x4 camera matrix (to project a point multiply by KK*inv(T))
%  data.I - the rgb image size(I) = [height width 3]
function data = orRobotSensorGetData(robotid, sensorindex)

session = openraveros_getglobalsession();
req = openraveros_robot_sensorgetdata();
req.bodyid = robotid;
req.sensorindex = sensorindex;
res = rosoct_session_call(session.id,'robot_sensorgetdata',req);
if( isempty(res) )
    data = [];
    return;
end

data.type = res.type;
switch(data.type)
    case 'laser'
        numrange = length(res.laserrange);
        data.laserrange = reshape(cell2mat(res.laserrange),[3 numrange/3]);

        numpos = length(res.laserpos);
        data.laserpos = reshape(cell2mat(res.laserpos),[3 numpos/3]);

        numint = length(res.laserint);
        data.laserint = reshape(cell2mat(res.laserint),[3 numint/3]);
    case 'camera'
        error('camera not supported yet');
        data.KK = reshape(cell2mat(res.KK),[3 3]);
        data.T = reshape(cell2mat(res.T.m),[3 4]);
        data.rawimage = res.image;
        display('image decoding not implemented yet');
    otherwise
        error('unknown type')
end
