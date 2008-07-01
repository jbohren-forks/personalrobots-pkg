#ifndef NAMELOOKUPCLIENT_HH
#define NAMELOOKUPCLIENT_HH

#include "ros/node.h"
#include "namelookup/NameToNumber.h"
#include <iostream>

class nameLookupClient 
{
public:
  nameLookupClient(ros::node &aNode): myNode(aNode)
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

private:
  ros::node &myNode;


};



#endif //NAMELOOKUPCLIENT_HH
