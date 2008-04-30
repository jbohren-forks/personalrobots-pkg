#include "auto_calibration/AutoCal.h"


using namespace std;



AutoCal::AutoCal()
{
  Info headinfo1 = {0, 180, 0.0, 0.0, 0.0, 0.0, 0};
  paramMap.insert(pair<string, Info>("head", headinfo1));   
  Info headinfo2 = {1, 180, 0.0, 0.0, 0.0, 0.0, 0};
  paramMap.insert(pair<string, Info>("head", headinfo2));
  Info headinfo3 = {2, 180, 0.0, 0.0, 0.0, 0.0, 0};
  paramMap.insert(pair<string, Info>("head", headinfo3));  
  Info shoulderinfo1 = {0, 180.0, 0.0, 0.0, 90.0, -90.0, 1};
  paramMap.insert(pair<string, Info>("shoulder", shoulderinfo1)); 
}


AutoCal::~AutoCal()
{

}



double AutoCal::RunAutoCal(string object )
{
  int speed = 50;
  int flag = 1;
  cout << "Now auto calibrating " << paramMap.count(object) <<" motors." << endl;   
  
  e.motors_on();
  e.set_control_mode(0);
  
  pair<multimap<string, Info>::iterator, multimap<string, Info>::iterator> ppp;
  ppp = paramMap.equal_range(object);
  
  for (multimap<string, Info>::iterator it2 = ppp.first; it2 != ppp.second; ++it2)
  {
      e.set_drv((*it2).second.motornum, speed);
  }  
  
  while(flag)
  {
    e.tick();
    for (multimap<string, Info>::iterator it2 = ppp.first; it2 != ppp.second; ++it2)
    {
        if(e.get_cur((*it2).second.motornum)>200)
        {
          e.set_drv((*it2).second.motornum, 0);
          (*it2).second.maxEncoder = e.get_enc((*it2).second.motornum);
        }
    }  
  }
    
#if 0
  for 
  int a[]={0, 0, 0, 0, 0, 0};
  e.drive(6,a);
  e.set_drv(2,100) //this is the motor and command
  
  //e.tick
#endif 
}


