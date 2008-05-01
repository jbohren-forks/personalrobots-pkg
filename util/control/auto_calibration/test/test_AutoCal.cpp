#include "auto_calibration/AutoCal.h"

using namespace std;

int main(int argc, char **argv)
{
  EtherDrive e;
  if (!e.init("192.168.1.12")) {
    cout << "Could not initialize etherdrive." << endl;
    return -1;
  }
  AutoCal robot(e);
  robot.RunAutoCal(argv[1]);
  #if 0 
  multimap<string, Info> paramMap;
  Info headinfo1 = {0, 180, 0.0, 0.0, 0.0, 0.0, 0};
  paramMap.insert(pair<string, Info>("head", headinfo1));   
  Info headinfo2 = {1, 180, 0.0, 0.0, 0.0, 0.0, 0};
  paramMap.insert(pair<string, Info>("head", headinfo2));
  Info headinfo3 = {2, 180, 0.0, 0.0, 0.0, 0.0, 0};
  paramMap.insert(pair<string, Info>("head", headinfo3));  
  Info shoulderinfo1 = {0, 180.0, 0.0, 0.0, 90.0, -90.0, 1};
  paramMap.insert(pair<string, Info>("shoulder", headinfo3));    
     
  cout << "Number of elements: " << paramMap.count(argv[1]) << endl;   
  
  pair<multimap<string, Info>::iterator, multimap<string, Info>::iterator> ppp;

   ppp = paramMap.equal_range(argv[1]);

   // Loop through range of maps of key "b"
   cout << endl << "Range of \"" << argv[1] << "\" elements:" << endl;
   for (multimap<string, Info>::iterator it2 = ppp.first;
       it2 != ppp.second;
       ++it2)
   {
       cout << "  [" << (*it2).first << ", " << (*it2).second.motornum << "]" << endl;
   }  
   #endif
}
