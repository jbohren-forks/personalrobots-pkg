#include "katana/katana.h"
#include <iostream>
#include <vector>

using namespace std;

void print_tMotInit(const TMotInit& tmi);

int
main(int argc, char** argv)
{
  // Get a Katana instance.
  Katana katana; 


  vector<TMotInit> v = katana.get_motor_parameters();
  vector<int> encoders = katana.get_joint_encoders();
  for(unsigned i = 0; i < v.size(); ++i) {
    cout << "[MOT[" << i << "]]" << endl; 
    print_tMotInit(v[i]);
    cout << "init_encoder_value = " << encoders[i] << ";" << endl;
    cout << endl;
  }
  
  return 0;
}  

void
print_tMotInit(const TMotInit& tmi)
{
  cout << "encoder_offset = " << tmi.encoderOffset << ";" << endl;
  cout << "encoders_per_cycle = " << tmi.encodersPerCycle << ";" << endl;
  cout << "angle_offset = " << tmi.angleOffset << ";" << endl;
  cout << "angle_range = " << tmi.angleRange << ";" << endl;
  cout << "rotation_direction = " << tmi.rotationDirection << ";" << endl;
  cout << "angle_stop = " << tmi.angleStop << ";" << endl;
}

