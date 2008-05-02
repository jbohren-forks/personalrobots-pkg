#include "auto_calibration/AutoCal.h"
#include "ros/node.h"

using namespace std;

class test : public ros::node
{
  public:
  test() : ros::node("test"){}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  test T;
  
  T.set_param("junk", 89898);
  int happy;
  
  if(T.get_param("junk", happy))
  {
  	cout<< "happy "<<happy<<endl;
  }
  else
  {
    cout<<"sad"<<endl;
  }

  EtherDrive e;
  if (!e.init("192.168.0.100")) {
    cout << "Could not initialize etherdrive." << endl;
    return -1;
  }
  AutoCal robot(e);
  robot.RunAutoCal(argv[1]);
  T.set_param("autocal",robot.paramMap);
 
}
