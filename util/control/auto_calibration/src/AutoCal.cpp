#include "auto_calibration/AutoCal.h"


using namespace std;

using namespace XmlRpc;

AutoCal::AutoCal(EtherDrive &_e):e(_e)
{

  // Info for head stage
  Info pan = {0, 360.0, 0.0, 0.0, -180.0, 180.0, 0, 0, 1000}; 
  Info tilt = {1, 120.0, 0.0, 0.0, -50.0, 70.0, 0, 0, 1000};
  Info hokuyo = {2, 180.0, 0.0, 0.0, -90.0, 90.0, 0, 0, 1000};

  XmlRpcValue head;
  pan.toXmlRpcValue(head["pan"]);
  tilt.toXmlRpcValue(head["tilt"]);
  hokuyo.toXmlRpcValue(head["hokuyo"]);

  paramMap["head"] = head;   


  // Info for arm stage
  Info shoulder = {0, 180.0, 0.0, 0.0, -90.0, 90.0, 0, 0, 1000};

  XmlRpcValue arm;
  shoulder.toXmlRpcValue(arm["shoulder"]);
  
  paramMap["arm"] = arm;   

}


AutoCal::~AutoCal()
{

}


void AutoCal::RunAutoCal(string objectName)
{
  int speed = 125;
  int flag = 1;
  int count = 0;
  
  Info info;

  XmlRpcValue object = paramMap[objectName];

  cout << "Now auto calibrating " << object.size() <<" motors." << endl;   
  
  e.motors_on();
  e.set_control_mode(0);
  
  for (XmlRpcValue::iterator it2 = object.begin(); it2 != object.end(); ++it2)
  {
      if (info.fromXmlRpcValue(it2) {
	e.set_drv(info.motorNum, speed);
      }
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
      if (info.fromXmlRpcValue(it2))
      {
	if(e.get_cur(info.motorNum)>425 && info.count>999)
	{   
	  e.set_drv(info.motorNum, 0);
		
	  if(info.flag == 1)
	  {
	    info.maxEncoder = e.get_enc(info.motorNum);
	    info.flag = 0;
	    info.count = 0;   
	  }
	  else if(info.flag == -1)
	  {
	    info.minEncoder = e.get_enc(info.motorNum);
	    info.flag = -2;
	  }
	}
	else if(info.flag==0)
	{
	  e.set_drv(info.motorNum, -speed);
	  info.flag = -1;
	} 
	else if(info.flag == -2)
	{
	  count = count+1;
	  info.flag = -3;
	   
	}
	if(count==object.size())
	{
	  flag=0;
	  e.motors_off();
	  cout << "Done calibrating " << object.size() <<" motors." << endl;
          
	} 
	info.count = info.count +1;
	info.toXmlRpcValue(it2);
      }
      usleep(300);
    }
  }

  for (XmlRpcValue::iterator it2 = object.begin(); it2 != object.end(); ++it2)
  {
    if (info.fromXmlRpcValue(it2) {
      cout << "Min Encoder : " << info.minEncoder << " Max Encoder : "<< info.maxEncoder << endl;
    }
  }  
}
