#include "auto_calibration/AutoCal.h"


using namespace std;



AutoCal::AutoCal(EtherDrive &_e):e(_e)
{
  Info headinfo1 = {0, 180, 0.0, 0.0, 0.0, 0.0, 0,1,1000}; //neck
  paramMap.insert(pair<string, Info>("head", headinfo1));   
  Info headinfo2 = {1, 180, 0.0, 0.0, 0.0, 0.0, 0,1,1000}; //eyes
  paramMap.insert(pair<string, Info>("head", headinfo2));
  Info headinfo3 = {2, 180, 0.0, 0.0, 0.0, 0.0, 0,1,1000}; //hokuyo
  paramMap.insert(pair<string, Info>("head", headinfo3));  
  Info shoulderinfo1 = {0, 180.0, 0.0, 0.0, 90.0, -90.0, 1,1,1000};
  paramMap.insert(pair<string, Info>("shoulder", shoulderinfo1)); 
}


AutoCal::~AutoCal()
{

}


double AutoCal::RunAutoCal(string object)
{
  int speed = 125;
  int flag = 1;
  int count = 0;
  
  
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
    if (!e.tick())
    {
      printf("Tick problem!.");
      flag=0;
      e.motors_off();
      exit(0);
    }
      
    for (multimap<string, Info>::iterator it2 = ppp.first; it2 != ppp.second; ++it2)
    {
	
        if(e.get_cur((*it2).second.motornum)>425 && (*it2).second.count>999)
        {   
          e.set_drv((*it2).second.motornum, 0);
          
          if((*it2).second.flag == 1)
          {
            (*it2).second.maxEncoder = e.get_enc((*it2).second.motornum);
            (*it2).second.flag=0;
            (*it2).second.count =0;   
          }
          else if((*it2).second.flag==-1)
          {
            (*it2).second.minEncoder = e.get_enc((*it2).second.motornum);
            (*it2).second.flag=-2;
          }
        }
        else if((*it2).second.flag==0)
        {
          e.set_drv((*it2).second.motornum, -speed);
          (*it2).second.flag=-1;
        } 
        else if((*it2).second.flag==-2)
        {
          count=count+1;
	  (*it2).second.flag=-3;
	   
        }
        if(count>=paramMap.count(object))
        {
          flag=0;
          e.motors_off();
          cout << "Done calibrating " << paramMap.count(object) <<" motors." << endl;
          
        } 
        (*it2).second.count = (*it2).second.count +1;
    } 
    usleep(300);
    
  }
   for (multimap<string, Info>::iterator it2 = ppp.first; it2 != ppp.second; ++it2)
   {
      cout << "Min Encoder : " << (*it2).second.minEncoder << " Max Encoder : "<< (*it2).second.maxEncoder << endl;
   }  
    
}


