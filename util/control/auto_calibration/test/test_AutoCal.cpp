#include "auto_calibration/AutoCal.h"
#include "ros/node.h"

using namespace std;

class test : public ros::node
{
  test() : ros::node("test"){}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  test T;
  
  T.setparam("junk", 89898);
  int happy;
  T.getparam("junk", happy);
  
  cout<< "happy "<<happy<<endl;
  EtherDrive e;
  if (!e.init("192.168.0.100")) {
    cout << "Could not initialize etherdrive." << endl;
    return -1;
  }
  AutoCal robot(e);
  robot.RunAutoCal(argv[1]);
  
 
}
