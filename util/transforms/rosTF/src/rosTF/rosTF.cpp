#include "rosTF/rosTF.h"

rosTFClient::rosTFClient(ros::node & rosnode):
  myNode(rosnode)
{
  myNode.subscribe("TransformEuler", eulerIn, &rosTFClient::receiveEuler, this);
  

};

void rosTFClient::receiveEuler()
{
  setWithEulers(eulerIn.frame, eulerIn.parent, eulerIn.x, eulerIn.y, eulerIn.z, eulerIn.yaw, eulerIn.pitch, eulerIn.roll, eulerIn.header.stamp_secs * 1000000000ULL + eulerIn.header.stamp_nsecs);
  std::cout << "recieved frame: " << eulerIn.frame << " with parent:" << eulerIn.parent << std::endl;
};


rosTFServer::rosTFServer(ros::node & rosnode):
  myNode(rosnode)
{
  myNode.advertise<MsgTransformEuler>("TransformEuler");

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
