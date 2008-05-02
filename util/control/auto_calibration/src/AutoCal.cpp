#include "auto_calibration/AutoCal.h"


using namespace std;

using namespace XmlRpc;

AutoCal::AutoCal(EtherDrive &_e):e(_e)
{
  //make the part
  XmlRpcValue head;
  //make the infos
  //neck (pan)
  XmlRpcValue info; 
  info["motorNum"] = 0;
  info["rotationRange"]= 360.0;
  info["maxEncoder"] = 0.0;
  info["minEncoder"]= 0.0;
  info["negOffset"] = -180.0;
  info["posOffset"]= 180.0;
  info["signRhr"] = 0.0;
  info["flag"]= 0;
  info["count"]= 1000;
  head["info1"] = info;
  //eyes (tilt)
  info.clear();
  info["motorNum"] = 1;
  info["rotationRange"]= 120.0;
  info["maxEncoder"] = 0.0;
  info["minEncoder"]= 0.0;
  info["negOffset"] = -50.0;
  info["posOffset"]= 70.0;
  info["signRhr"] = 0.0;
  info["flag"]= 0;
  info["count"]= 1000;
  head["info2"] = info;
  //mouth (hokuyo)
  info.clear();
  info["motorNum"] = 2;
  info["rotationRange"]= 180.0;
  info["maxEncoder"] = 0.0;
  info["minEncoder"]= 0.0;
  info["negOffset"] = -90.0;
  info["posOffset"]= 90.0;
  info["signRhr"] = 0.0;
  info["flag"]= 0;
  info["count"]= 1000;
  head["info3"] = info;
  //make the map
  paramMap["head"] = head;   

  //make the part
  XmlRpcValue arm;
  //make the infos
  //Humerus (shoulder)
  info.clear(); 
  info["motorNum"] = 0;
  info["rotationRange"]= 180.0;
  info["maxEncoder"] = 0.0;
  info["minEncoder"]= 0.0;
  info["negOffset"] = -90.0;
  info["posOffset"]= 90.0;
  info["signRhr"] = 0.0;
  info["flag"]= 0;
  info["count"]= 1000;
  arm["info1"] = info;
  //make the map
  paramMap["arm"] = arm;   


}


AutoCal::~AutoCal()
{

}


double AutoCal::RunAutoCal(string objectName)
{
  int speed = 125;
  int flag = 1;
  int count = 0;
  
  XmlRpcValue object = paramMap[objectName];

  cout << "Now auto calibrating " << object.size() <<" motors." << endl;   
  
  e.motors_on();
  e.set_control_mode(0);
  
  for (XmlRpcValue::iterator it2 = object.begin(); it2 != object.end(); ++it2)
  {
      e.set_drv((int)(*it2).second["motorNum"], speed);
  }  
  
  while(flag)
  {
    if (!e.tick())
    {
      printf("Tick problem!.");
      flag=0;
      e.motors_off();
      exit(0);
    }
      
    for (XmlRpcValue::iterator it2 = object.begin(); it2 != object.end(); ++it2)
    {
	
        if(e.get_cur((int)(*it2).second["motorNum"])>425 && (int)(*it2).second["count"]>999)
        {   
          e.set_drv((int)(*it2).second["motorNum"], 0);
          
          if((int)(*it2).second["flag"] == 1)
          {
            (*it2).second["maxEncoder"] = e.get_enc((*it2).second["motorNum"]);
            (*it2).second["flag"]=0;
            (*it2).second["count"] =0;   
          }
          else if((int)(*it2).second["flag"]==-1)
          {
            (*it2).second["minEncoder"] = e.get_enc((*it2).second["motorNum"]);
            (*it2).second["flag"]=-2;
          }
        }
        else if((int)(*it2).second["flag"]==0)
        {
          e.set_drv((*it2).second["motorNum"], -speed);
          (*it2).second["flag"]=-1;
        } 
        else if((int)(*it2).second["flag"]==-2)
        {
          count=count+1;
	        (*it2).second["flag"]=-3;
	   
        }
        if(count==object.size())
        {
          flag=0;
          e.motors_off();
          cout << "Done calibrating " << object.size() <<" motors." << endl;
          
        } 
        (*it2).second["count"] = (int)(*it2).second["count"] +1;
    } 
    usleep(300);
    
  }
  for (XmlRpcValue::iterator it2 = object.begin(); it2 != object.end(); ++it2)
  {
    cout << "Min Encoder : " << (*it2).second["minEncoder"] << " Max Encoder : "<< (*it2).second["maxEncoder"] << endl;
  }  
    
}


