#include "AutoCal.h"


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


void AutoCal::InitAutoCal()
{
 
 
}


double AutoCal::RunAutoCal( )
{
#if 0
  for 
  int a[]={0, 0, 0, 0, 0, 0};
  e.drive(6,a);
  e.set_drv(2,100) //this is the motor and command
  
  //e.tick
#endif 
}


