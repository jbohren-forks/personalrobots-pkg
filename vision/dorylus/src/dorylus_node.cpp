#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <iostream>
#include <smartScan.h>
#include <dorylus.h>
#include <scene_labeler_stereo.h>

using namespace std;

class DorylusNode : public ros::node
{
public:

  vector<Descriptor*> descriptors_;
  SpinLarge spinL_;
  SpinMedium spinM_;

  DorylusNode() : ros::node("dorylus_node")
  {

    // -- Setup descriptor functions.
    descriptors_.push_back(&spinL_);
    descriptors_.push_back(&spinM_);
    
  }

private:
  //DorylusDataset dd;
  //Dorylus dory;
  //SceneLabelerStereo sls;
  
};

int main(int argc, char **argv) {
  ros::init(argc, argv);
  DorylusNode d;

  cout << "Running functions" << endl;

  cout << d.descriptors_.size() << " descriptors." << endl;
  
  (*d.descriptors_[0])();
  (*d.descriptors_[1])();
  cout << "Done." << endl;

  while(d.ok())
    usleep(1000);

  //  d.self_destruct();
  ros::fini();
  return 0;
}


