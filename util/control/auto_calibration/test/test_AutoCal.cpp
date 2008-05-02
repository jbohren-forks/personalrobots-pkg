#include "auto_calibration/AutoCal.h"
#include "ros/node.h"

using namespace std;

class Test_Autocal : public ros::node
{
public:
  EtherDrive e;
  AutoCal ac;

  Test_Autocal() : ros::node("test"), ac(e)
  {
    if (!e.init("192.168.0.100")) {
      cout << "Could not initialize etherdrive." << endl;
      exit(-1);
    }
  }

  void run() {
    ac.RunAutoCal(argv[1]);    
    set_param("autocal",ac.paramMap);
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Test_Autocal T;

  T.run();

  return 1;
}
