#include "ros/node.h"
#include "namelookup/nameLookupClient.hh"
#include "namelookup/NameToNumber.h"
#include <iostream>


int main(int argc, char ** argv)
{
  ros::init(argc, argv);
  ros::node myNode("nameLookupTestClient");

  nameLookupClient myClient(myNode);
  std::string myString;
  while(myNode.ok())
    {
      std::cout << "Enter a string:";
      std::cin>>myString;
      std::cout << myClient.lookup(myString) << std::endl;
    }

  ros::fini();

  return 0;
}
