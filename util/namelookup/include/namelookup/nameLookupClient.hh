#ifndef NAMELOOKUPCLIENT_HH
#define NAMELOOKUPCLIENT_HH

#include <ros/node.h>
#include <ros/time.h>
#include <namelookup/NameToNumber.h>
#include <namelookup/NumberToName.h>
#include <iostream>
#include <map>

class nameLookupClient 
{
public:
  nameLookupClient(ros::node &aNode): myNode(aNode)
  {
    pthread_mutex_init(&protect_call, NULL);

    int value;
    while (value == 0 && myNode.ok())
    {
      printf("Waiting for namelookup_server\n");
      ros::Duration(0.5).sleep();//wait for service to come up
      value = lookupOnServer("");
    };

  };

  virtual ~nameLookupClient(void)
  {
    pthread_mutex_destroy(&protect_call);
  };

  int lookup(const std::string& str_in)
  {
    std::map<std::string, int>::iterator it = nameMap.find(str_in);
    if ( it == nameMap.end())
      {
        
        int value = 0;
        value = lookupOnServer(str_in); //Ask the server
        if (value != 0) //don't record a failure
        {
          nameMap[str_in] = value; //Record lookup
          reverseNameMap[value] = str_in;  //Record reverse lookup
        }
        return value;
      }
    else
      {
        return (*it).second;
      }
  }  
    
  std::string reverseLookup(unsigned int frameid)
  {
    std::map<int, std::string>::iterator it = reverseNameMap.find(frameid);
    if ( it == reverseNameMap.end())
      {
        std::string  name;
        name = reverseLookupOnServer(frameid);
        if (name != "") //Dont' cache a failure
          reverseNameMap[frameid] = name;
        return name;
      }
    else
    {
      return (*it).second;
    }
  }  
  
private:

  ros::node &myNode;
  std::map<std::string, int> nameMap;  
  std::map<int, std::string> reverseNameMap;  
  pthread_mutex_t protect_call;

  int lookupOnServer(const std::string &str_in)
  {
    namelookup::NameToNumber::request req;
    namelookup::NameToNumber::response res;
    req.name = myNode.map_name(str_in);
    
    int result = 0;
    pthread_mutex_lock(&protect_call);   
    if (ros::service::call("/nameToNumber", req, res))
  	  result = res.number;
  
    pthread_mutex_unlock(&protect_call);   
    return result;
  }
  
  std::string reverseLookupOnServer(int frameid)
  {
    namelookup::NumberToName::request req;
    namelookup::NumberToName::response res;
    req.number = frameid;
    
    std::string result;
    pthread_mutex_lock(&protect_call);   
    if (ros::service::call("/numberToName", req, res))
  	  result = res.name;
  
    pthread_mutex_unlock(&protect_call);   
    return result;
  }
};



#endif //NAMELOOKUPCLIENT_HH
