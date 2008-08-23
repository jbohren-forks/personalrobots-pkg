#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <iostream>
#include <smartScan.h>

using namespace std;

class DorylusNode : public ros::node
{
public:
  dorylus_node() : ros::node("dorylus_node")
  {
  }

private:
  DorylusDataset dd;
  Dorylus dory;
  SceneLabelerStereo sls;

};

int main(int argc, char **argv) {
  ros::init(argc, argv);
  DorylusNode d;
  while(d.ok())
    usleep(1000);

  //  d.self_destruct();
  ros::fini();
  return 0;
}


