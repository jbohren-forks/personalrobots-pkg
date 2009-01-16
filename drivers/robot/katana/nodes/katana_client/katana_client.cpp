#include <cstdio>
#include <stdexcept>
#include <string>
#include "ros/node.h"
#include "std_srvs/StringString.h"

using std::vector;
using std::endl;
using std::string;

class KatanaClient : public ros::Node
{
public:
  KatanaClient() : ros::Node("katana_client")
  {
  }
  void move_for_camera()
  {
    std_srvs::StringString::request  req;
    std_srvs::StringString::response res;
    if (!ros::service::call("katana_move_for_camera_service", req, res))
      printf("couldn't move for camera\n");
    else
      printf("move for camera OK\n");
  }
  void move_to_upright()
  {
    std_srvs::StringString::request  req;
    std_srvs::StringString::response res;
    if (!ros::service::call("katana_move_upright_service", req, res))
      printf("couldn't move upright\n");
    else
      printf("move upright OK\n");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  KatanaClient c;
  if (string(argv[1]) == string("move_for_camera"))
    c.move_for_camera();
  else if (string(argv[1]) == string("move_to_upright"))
    c.move_to_upright();
  ros::fini();
  return 0;
}

