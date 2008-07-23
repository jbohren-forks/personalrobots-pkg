#ifndef NAMELOOKUPCLIENT_HH
#define NAMELOOKUPCLIENT_HH

#include "ros/node.h"
#include "namelookup/NameToNumber.h"
#include <iostream>
#include <map>

class nameLookupClient 
{
public:
  nameLookupClient(ros::node &aNode): myNode(aNode)
  {
    pthread_mutex_init(&protect_call, NULL);
  };

  int lookup(std::string str_in)
  {
    std::map<std::string, int>::iterator it;
    it = nameMap.find(str_in);
    if ( it == nameMap.end())
      {
        
        int value = 0;
        value = lookupOnServer(str_in);
        while (value == 0)
          {
            printf("Waiting for namelookup_server\n");
            usleep(500000);//wait for service to come up
            value = lookupOnServer(str_in);
          };
        nameMap[str_in] = value;
        return value;
      }
    else
      {
        return (*it).second;
      }
    
  }  
      
  
  
  
private:
  ros::node &myNode;
  std::map<std::string, int> nameMap;  
  pthread_mutex_t protect_call;

  int lookupOnServer(std::string str_in)
  {
    namelookup::NameToNumber::request req;
    namelookup::NameToNumber::response res;
    req.name = myNode.map_name(str_in);
    pthread_mutex_lock(&protect_call);   
    if (ros::service::call("/nameToNumber", req, res))
      {
        pthread_mutex_unlock(&protect_call);   
        return res.number;
      }
    else 
      {
        pthread_mutex_unlock(&protect_call);   
        return 0;
      }
  }
  
};



#endif //NAMELOOKUPCLIENT_HH
