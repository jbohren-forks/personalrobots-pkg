#include <etherdrive_hardware/etherdrive_hardware.h>
#include <hw_interface/hardware_interface.h>
#include <pr2Controllers/BaseController.h>
#include <genericControllers/JointController.h>
#include <robot_model/joint.h>
#include <sys/time.h>

using namespace CONTROLLER;

const double PGain = 2.5;
const double IGain = 0; 
const double DGain = 0;
const double IMax  = 0;
const double IMin  = 0;
//const double maxPositiveTorque = 0.0528; 
//const double maxNegativeTorque = -0.0528; 
//const double maxEffort = 0.0528;
const double maxPositiveTorque = 0.75; 
const double maxNegativeTorque = -0.75; 
const double maxEffort = 0.75;
const double PGain_Pos = 0.5; 
const double IGain_Pos = 0; 
const double DGain_Pos = 0.04; 

const int NUM_JOINTS = 12;


double GetTime()
{
   struct timeval t;
   gettimeofday( &t, 0);
   return (double) (t.tv_usec *1e-6 + t.tv_sec);
}

int main(int argc, char *argv[]){

  /* 
  int numBoards = 2;
  int numActuators = 12;
  int boardLookUp[] ={0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1}; 
  int portLookUp[] = {0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
  int jointId[]={1, 2, 0, 7, 8, 6, 4, 5, 3, 10, 11, 9};
  string etherIP[] = {"10.12.0.103", "10.11.0.102"};
  string hostIP[] = {"10.12.0.2", "10.11.0.3"};
  */

  int numBoards = 1;
  int numActuators = 1;
  int boardLookUp[] ={0}; 
  int portLookUp[] = {0};
  int jointId[]={0};
  string etherIP[] = {"10.12.0.103"};
  string hostIP[] = {"10.12.0.2"};


  HardwareInterface *hi = new HardwareInterface(1);
  EtherdriveHardware *h = new EtherdriveHardware(numBoards, numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP,hi);
  JointController *jc = new JointController();
  Joint *joint = new Joint();

  //  BaseController *b = new BaseController();
  //  float pos[] = {0,0,0,0,0,0,0,0,0,0,0,0};

  h->init();
  jc->Init(PGain_Pos, IGain_Pos, DGain_Pos, IMax, IMin, CONTROLLER_POSITION, GetTime(), maxPositiveTorque, maxNegativeTorque, maxEffort, joint);
  hi->actuator[0].command.enable = true;
  jc->EnableController();
  joint->effortLimit = maxPositiveTorque;

  for(int ii =0; ii < 100000; ii++) {

  h->updateState();
  joint->position = ((double)hi->actuator[0].state.encoderCount)/14000.0;

  jc->SetPosCmd(0);
  jc->Update();

  cout << "pos:: " << joint->position << ", eff:: " << joint->commandedEffort << endl;

  hi->actuator[0].command.current = joint->commandedEffort;
  h->sendCommand();
  //  h->sendCommand(jc->CommandedEffort);
  h->update();


  usleep(1000);
  }  

  delete(h);
  delete(hi);
}
