#include "ros/node.h"
#include "namelookup/NameToNumber.h"
#include <iostream>

class testClient : public ros::node
{
public:
  testClient(): ros::node("testNameToNumber")
  {
  };

  int lookup(std::string str_in)
  {
    namelookup::NameToNumber::request req;
    namelookup::NameToNumber::response res;
    req.name = str_in;
    if (ros::service::call("nameToNumber", req, res))
      return res.number;
    else 
      return 0;
  }

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv);
  testClient myClient;
  std::string myString;
  while(myClient.ok())
    {
      std::cout << "Enter a string:";
      std::cin>>myString;
      std::cout << myClient.lookup(myString) << std::endl;
    }

  ros::fini();

  return 0;
}
