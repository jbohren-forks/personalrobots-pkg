#include "rosTF/rosTF.h"

rosTFClient::rosTFClient(ros::node & rosnode):
  myNode(rosnode)
{
  myNode.subscribe("TransformEuler", eulerIn, &rosTFClient::receiveEuler, this);
  myNode.subscribe("TransformDH", dhIn, &rosTFClient::receiveDH, this);
  myNode.subscribe("TransformQuaternion", quaternionIn, &rosTFClient::receiveQuaternion, this);

};

void rosTFClient::receiveEuler()
{
  setWithEulers(eulerIn.frame, eulerIn.parent, eulerIn.x, eulerIn.y, eulerIn.z, eulerIn.yaw, eulerIn.pitch, eulerIn.roll, eulerIn.header.stamp_secs * 1000000000ULL + eulerIn.header.stamp_nsecs);
  std::cout << "recieved euler frame: " << eulerIn.frame << " with parent:" << eulerIn.parent << std::endl;
};

void rosTFClient::receiveDH()
{
  setWithDH(dhIn.frame, dhIn.parent, dhIn.length, dhIn.twist, dhIn.offset, dhIn.angle, dhIn.header.stamp_secs * 1000000000ULL + dhIn.header.stamp_nsecs);
  std::cout << "recieved DH frame: " << dhIn.frame << " with parent:" << dhIn.parent << std::endl;
};

void rosTFClient::receiveQuaternion()
{
  setWithQuaternion(quaternionIn.frame, quaternionIn.parent, quaternionIn.xt, quaternionIn.yt, quaternionIn.zt, quaternionIn.xr, quaternionIn.yr, quaternionIn.zr, quaternionIn.w, quaternionIn.header.stamp_secs * 1000000000ULL + quaternionIn.header.stamp_nsecs);
  std::cout << "recieved quaternion frame: " << quaternionIn.frame << " with parent:" << quaternionIn.parent << std::endl;
};


rosTFServer::rosTFServer(ros::node & rosnode):
  myNode(rosnode)
{
  myNode.advertise<MsgTransformEuler>("TransformEuler");
  myNode.advertise<MsgTransformDH>("TransformDH");
  myNode.advertise<MsgTransformQuaternion>("TransformQuaternion");

};


void rosTFServer::sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs)
{
  eulerOut.frame = frame;
  eulerOut.parent = parent;
  eulerOut.x = x;
  eulerOut.y = y;
  eulerOut.z = z;
  eulerOut.yaw = yaw;
  eulerOut.pitch = pitch;
  eulerOut.roll = roll;
  eulerOut.header.stamp_secs = secs;
  eulerOut.header.stamp_nsecs = nsecs;

  myNode.publish("TransformEuler", eulerOut);

};


void rosTFServer::sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs)
{
  dhOut.frame = frame;
  dhOut.parent = parent;
  dhOut.length = length;
  dhOut.twist = twist;
  dhOut.offset = offset;
  dhOut.angle = angle;
  dhOut.header.stamp_secs = secs;
  dhOut.header.stamp_nsecs = nsecs;

  myNode.publish("TransformDH", dhOut);

};

void rosTFServer::sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs)
{
  quaternionOut.frame = frame;
  quaternionOut.parent = parent;
  quaternionOut.xt = xt;
  quaternionOut.yt = yt;
  quaternionOut.zt = zt;
  quaternionOut.xr = xr;
  quaternionOut.yr = yr;
  quaternionOut.zr = zr;
  quaternionOut.w = w;
  quaternionOut.header.stamp_secs = secs;
  quaternionOut.header.stamp_nsecs = nsecs;

  myNode.publish("TransformQuaternion", quaternionOut);

};
