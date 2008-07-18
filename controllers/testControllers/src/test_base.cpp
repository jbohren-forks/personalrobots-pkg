#include <etherdrive_hardware/etherdrive_hardware.h>
#include <hw_interface/hardware_interface.h>
#include <pr2Controllers/BaseController.h>
#include <genericControllers/JointController.h>
#include <robot_model/joint.h>
#include <sys/time.h>
#include <signal.h>

using namespace CONTROLLER;

const double PGain = 10.0;
const double IGain = 0; 
const double DGain = 0;

//const double maxPositiveTorque = 0.0528; 
//const double maxNegativeTorque = -0.0528; 
//const double maxEffort = 0.0528;
const double maxEffort = 0.75;
const double maxPositiveTorque = maxEffort; 
const double maxNegativeTorque = -maxEffort; 
const double PGain_Pos = 10.0; 
const double IGain_Pos = 0; 
const double DGain_Pos = 0.1; 

const double IMax  = 10;
const double IMin  = -10;

const int NUM_JOINTS = 12;
int notDone = 1;

double GetTime()
{
  struct timeval t;
  gettimeofday( &t, 0);
  return (double) (t.tv_usec *1e-6 + t.tv_sec);
}

void finalize(int dummy){
  notDone = 0;
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

  timespec req, rem;
  req.tv_nsec = (long) 1e5;
  req.tv_sec = (time_t) 0;

  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

  int iB, iP;

  int numBoards = 1;
  int numActuators = 1;

  int boardLookUp[] ={0}; 
  int portLookUp[] = {2};

  double cmd = 0;
  
  if(argc > 2){
    iB = atoi(argv[1]);
    iP = atoi(argv[2]);
    boardLookUp[0] =iB; 
    portLookUp[0] = iP;
  }

  if(argc > 3)
    cmd = atof(argv[3]);

  double pGain = PGain;
  double iGain = IGain;
  double dGain = DGain;

  if(argc > 6){
    pGain = atof(argv[4]);
    iGain = atof(argv[5]);
    dGain = atof(argv[6]);
    }

  int jointId[]={0};
  //     string etherIP[] = {"10.12.0.103", "10.11.0.102"};
  //     string hostIP[] = {"10.12.0.2", "10.11.0.3"};
  string etherIP[] = {"10.11.0.102"};
  string hostIP[] = {"10.11.0.3"};
  //     string etherIP[] = {"10.12.0.103"};
  //   string hostIP[] = {"10.12.0.2"};
  HardwareInterface *hi = new HardwareInterface(1);
  EtherdriveHardware *h = new EtherdriveHardware(numBoards, numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP,hi);
  JointController *jc = new JointController();
  Joint *joint = new Joint();

  SimpleTransmission *sc = new SimpleTransmission(joint,&hi->actuator[0],1,1,90000);

  //  BaseController *b = new BaseController();
  //  float pos[] = {0,0,0,0,0,0,0,0,0,0,0,0};

  h->init();
  //  jc->Init(PGain_Pos, IGain_Pos, DGain_Pos, IMax, IMin, CONTROLLER_POSITION, GetTime(), maxPositiveTorque, maxNegativeTorque, maxEffort, joint);
   jc->Init(pGain, iGain, dGain, IMax, IMin, ETHERDRIVE_SPEED, GetTime(), maxPositiveTorque, maxNegativeTorque, maxEffort, joint);
  hi->actuator[0].command.enable = true;
  jc->EnableController();
  joint->effortLimit = maxPositiveTorque;
  double lcmd;
  //  for(;;) 
  while(notDone) {

    h->updateState();
    sc->propagatePosition();
    //    jc->SetPosCmd(cmd);
    //   lcmd = 100*(joint->position - cmd);
    lcmd = cmd;
    jc->SetVelCmd(lcmd);
    jc->Update();

    //cout << "pos:: " << joint->position << ", eff:: " << joint->commandedEffort << endl;
    cout << "test_base.cpp:: vel:: " << joint->velocity << ", eff:: " << joint->commandedEffort << endl;
    //int print_n = (int)(joint->velocity * 5);
    //if(print_n > 1000)
    //  print_n = 1000;
    //for(int i = 0; i < print_n; i++) {cout << "#";}
    //cout << endl;
    
    sc->propagateEffort();
    h->sendCommand();
    h->tick();

    //        nanosleep(&req,&rem);
    //usleep(200);
  }  

  delete(jc);
  delete(joint);
  delete(h);
  delete(hi);
}
