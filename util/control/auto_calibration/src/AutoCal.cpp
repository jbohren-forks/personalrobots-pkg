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
  Info shoulder = {0, 180.0, 0.0, 0.0, -90.0, 90.0, 0, 1, 1000};

  XmlRpcValue arm;
  shoulder.toXmlRpcValue(arm["shoulder"]);
  
  paramMap["arm"] = arm;   

}


AutoCal::~AutoCal()
{

}


void AutoCal::RunAutoCal(string objectName)
{
  int speed = 70;
  int flag = 1;
  int count = 0;
  
  Info info;

  XmlRpcValue object = paramMap[objectName];

  cout << "Now auto calibrating " << object.size() <<" motors." << endl;   
  
  if (e.set_control_mode(0))
    cout << "Control mode set to 0" << endl;
  if (e.motors_on())
    cout << "Motors turned on" << endl;
  
  for (XmlRpcValue::iterator it2 = object.begin(); it2 != object.end(); ++it2)
  {
    //cout << "XmlRpcValue size: " << (*it2).second.size() << endl;
    if (info.fromXmlRpcValue(it2)) {
      
      //cout<<"here"<<endl;
      e.set_drv(info.motorNum, speed);
      e.tick();
      cout<<e.get_cur(info.motorNum)<<endl;
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
        
      	if(e.get_cur(info.motorNum)>400 && info.count>50)
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
      	else if(info.flag==0 && info.count>50)
      	{
      	  e.set_drv(info.motorNum, -speed);
      	  info.flag = -1;
      	  info.count=0;
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
    if (info.fromXmlRpcValue(it2)) 
    {
      cout << "Min Encoder : " << info.minEncoder << " Max Encoder : "<< info.maxEncoder << endl;
    }
  }  
}
