#include "configuration_dictionary.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "configuration_dictionary_test");
  MutableConfigurationDictionary d;
  //d.setParam("abc", 1.0);
  //d.setParam("key123", 1.0);
  //d.setParam("pi", 3.1415);
  //d.setParam("key2", "1.0");
  //d.loadFromYamlFile(std::string("test.yaml"));
  d.loadFromParamServer("");
  
  cout << (d.asYaml()) << endl;
  
  ros::spin();

  exit(0);
}
