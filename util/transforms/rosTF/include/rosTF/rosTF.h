
#include <iostream>
#include "ros/node.h"
#include "std_msgs/MsgTransformEuler.h"
#include "libTF/libTF.h"

class rosTFClient : public TransformReference
{
 public:
  //Constructor
  rosTFClient(ros::node & rosnode);


  void receiveEuler();

 private:
  ros::node & myNode;
  MsgTransformEuler eulerIn;

};


class rosTFServer
{
 public:
  //Constructor
  rosTFServer(ros::node & rosnode);

  void sendEuler(unsigned int frame, unsigned int parent, double x, double y, double z, double yaw, double pitch, double roll, unsigned long long time);
  //  void sendQuaternion();
  // void sendDH();

 private:
  ros::node & myNode;
  MsgTransformEuler eulerOut;

};
