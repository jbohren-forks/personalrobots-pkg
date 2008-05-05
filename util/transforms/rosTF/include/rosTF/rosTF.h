
#include <iostream>
#include "ros/node.h"
#include "std_msgs/MsgTransformEuler.h"
#include "std_msgs/MsgTransformDH.h"
#include "std_msgs/MsgTransformQuaternion.h"
#include "libTF/libTF.h"

class rosTFClient : public TransformReference
{
 public:
  //Constructor
  rosTFClient(ros::node & rosnode);


  void receiveEuler();
  void receiveDH();
  void receiveQuaternion();

 private:
  ros::node & myNode;
  MsgTransformEuler eulerIn;
  MsgTransformDH dhIn;
  MsgTransformQuaternion quaternionIn;

};


class rosTFServer
{
 public:
  //Constructor
  rosTFServer(ros::node & rosnode);

  void sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned int secs, unsigned int nsecs);
  void sendDH(unsigned int frame, unsigned int parent, double length, double twist, double offset, double angle, unsigned int secs, unsigned int nsecs);
  void sendQuaternion(unsigned int frame, unsigned int parent, double xt, double yt, double zt, double xr, double yr, double zr, double w, unsigned int secs, unsigned int nsecs);
  //  void sendQuaternion();
  // void sendDH();

 private:
  ros::node & myNode;
  MsgTransformEuler eulerOut;
  MsgTransformDH dhOut;
  MsgTransformQuaternion quaternionOut;

};
