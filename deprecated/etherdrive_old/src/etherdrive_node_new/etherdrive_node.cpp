///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "ros/node.h"
#include "std_msgs/ActuatorState.h"
#include "std_msgs/ActuatorCmd.h"
#include "etherdrive/etherdrive.h"
#include <sstream>

class EtherDrive_Node : public ros::node
{
public:
  std_msgs::ActuatorState mot[6];
  std_msgs::ActuatorCmd mot_cmd[6];

  string host;
  EtherDrive *ed;
  int frame_id;

  float last_mot_val[6];
  double pulse_per_rad[6];

  int lastEncPos[6];
  int mode; // 0=voltage, 1=current, 2=position, 3=velocity, 4=position on comp
  bool firstTimeVelCtrl;
  int fooCounter;
  int counter;
  float last_mot_cmd[6];


  EtherDrive_Node() : ros::node("etherdrive"), ed(NULL)
  {

    for (int i = 0;i < 6;i++) {
      ostringstream oss;
      oss << "mot" << i;
      advertise<std_msgs::ActuatorState>(oss.str().c_str());

      oss << "_cmd";
      subscribe(oss.str().c_str(), mot_cmd[i], &EtherDrive_Node::mot_callback);

      last_mot_val[i] = 0;
    }

    //if (!get_param(string(".host"), host))
     // host = "192.168.1.12";
    host = "10.12.0.102";
    printf("EtherDrive host set to [%s]\n", host.c_str());

    //if (!get_param(string(".frame_id"), frame_id))
    frame_id = -1;
    printf("EtherDrive frame_id set to [%d]\n", frame_id);

    for (int i = 0;i < 6;i++) {
      ostringstream oss;
      oss << "pulse_per_rad" << i;
      //if (!get_param(oss.str().c_str(), pulse_per_rad[i]))
	    pulse_per_rad[i] = 1591.54943;
      printf("Using %g pulse_per_rad for motor %d\n", pulse_per_rad[i], i);
    }

    ed = new EtherDrive();
    ed->init(host);

// debugging JS: initialize values
    for (int i = 0; i < 6; i++) {
      mot_cmd[i].valid = true;
      mot_cmd[i].rel = 0;
      mot_cmd[i].cmd = 0;

      ed->set_gain(i,'P',0);
      ed->set_gain(i,'I',10);
      ed->set_gain(i,'D',0);
      ed->set_gain(i,'W',100);
      ed->set_gain(i,'M',1004);
      ed->set_gain(i,'Z',1);

    }

    ed->set_control_mode(1); // 0=voltage, 1=current control



    firstTimeVelCtrl = true;
    fooCounter = 0;
    ed->motors_on();
    counter = 0;
  }

  void mot_callback() {
    //printf("received motor 3 command %f\n", mot_cmd[3].val);
  }

  virtual ~EtherDrive_Node()
  {
    if (ed)
      delete ed;
  }

  bool do_tick()
  {
//printf("%i: enc: %i \n", fooCounter, ed->get_enc(5));
    int tmp_mot_cmd[6];
    int currEncPos[6];

    float curVel;
    float desVel;
    float desPos[6];
    float k_p[6];
    float pos_k_p[6];
    float pos_k_i[6];
    float pos_k_d[6];
    float foo;

    float posError[6];
    float intPosError[6];
    float lastPosError[6];

    k_p[0]= 0.003;
    k_p[1]= 0.003;
    k_p[2]= -0.003;

    k_p[3]= 0.003;
    k_p[4]= 0.003;
    k_p[5]= -0.003;


    k_p[0]= 5;
    k_p[1]= 5;
    k_p[2]= -5;

    k_p[3]= 5;
    k_p[4]= 5;
    k_p[5]= -5;

    // position control:
    pos_k_p[0] = 1;
    pos_k_p[1] = 1;
    pos_k_p[2] = -3;
    pos_k_p[3] = 1.2;
    pos_k_p[4] = 1.2;
    pos_k_p[5] = -1.2; //

    pos_k_i[0] = 0;
    pos_k_i[1] = 0;
    pos_k_i[2] = 0;
    pos_k_i[3] = 0.01;
    pos_k_i[4] = 0.01;//0;
    pos_k_i[5] =-0.01; // 4.8;

    pos_k_d[0] = 1;
    pos_k_d[1] = 1;
    pos_k_d[2] = -3;
    pos_k_d[3] = 0.075; //1;
    pos_k_d[4] = 0.075; //-0.075; //1;
    pos_k_d[5] = -0.075;//-0.075; //
    /*
    voltage control settings:
    pos_k_p[3] = -1;
    pos_k_p[4] = 1;
    pos_k_p[5] = 1;
    */
    for (int i = 0; i < 6; i++) {
      if(i == 5) printf("counter: %i\n", fooCounter);
      mot_cmd[i].lock();
      switch(mot_cmd[i].mode)
      {

        case 0: // direct
          tmp_mot_cmd[i] = (int)mot_cmd[i].cmd; //last_mot_val[i]; //JS
          break;
        case 1: // position control on computer
          desPos[i] = float(mot_cmd[i].cmd)/360*90000; //desPos[i]=ed->get_enc(i) + mot[i].val;
//printf("desPos: %i, enc: %i\n", desPos[i], ed->get_enc(i));
          posError[i] =   desPos[i] - float(ed->get_enc(i));
          tmp_mot_cmd[i] = int(pos_k_p[i]*posError[i]) + int(pos_k_d[i]*(posError[i]-lastPosError[i])/0.001) + int(pos_k_i[i]*intPosError[i]);

          lastPosError[i] = posError[i];
          intPosError[i] += posError[i];
        break;
        case 2: // velocity control
         if(fooCounter < 10) { // first couple of encoder readings are messed up for some reason. JS
            lastEncPos[i] = ed->get_enc(i);
            last_mot_cmd[i] = 0;
          }
          fooCounter++;

          currEncPos[i] = ed->get_enc(i);
          curVel =  float(currEncPos[i] - lastEncPos[i])/9000*0.48;   //encoder (units/msec) need to calibrate this
          desVel = mot_cmd[i].cmd; // enc units/msec.  should be (wheel rev/s)
          if((desVel-float(curVel))>0) tmp_mot_cmd[i] = int(last_mot_cmd[i] + 10*k_p[i]);
          if((desVel-float(curVel))<0) tmp_mot_cmd[i] = int(last_mot_cmd[i] - 10*k_p[i]);
          if(desVel <= 0.0001 && desVel >=-0.0001) tmp_mot_cmd[i] = 0;
          //tmp_mot_cmd[i] = int(last_mot_cmd[i] + k_p[i]*(desVel-float(curVel)));


          lastEncPos[i] = currEncPos[i];
          last_mot_cmd[i] = last_mot_cmd[i] + k_p[i]*(desVel-curVel);
        break;
        case 3:  // On board position control?
          if (mot_cmd[i].valid) {
	          if (mot_cmd[i].rel) {
	            last_mot_val[i] = mot[i].pos + mot_cmd[i].cmd;
	          } else {
	            last_mot_val[i] = mot_cmd[i].cmd;
	          }
            tmp_mot_cmd[i] = (int)(last_mot_val[i] * pulse_per_rad[i]);
          }
          break;
        default:
          // do nothing
          tmp_mot_cmd[i] = 0;
          break;

      }
      mot_cmd[i].valid = false;  // set to invalid so we don't re-use
      mot_cmd[i].unlock();
      if(tmp_mot_cmd[i] > 3000) {
        tmp_mot_cmd[i] = 3000;
        printf("Max current\n");
      }
      if(tmp_mot_cmd[i] < -3000) {
        tmp_mot_cmd[i] = -3000;
        printf("Max current\n");
      }
    }

    ed->drive(6,tmp_mot_cmd);
    int val[6];
    if (!ed->tick(6,val)) {
      return false;
    }

    for (int i = 0; i < 6; i++) {
      mot[i].pos = val[i] / pulse_per_rad[i];
      mot[i].pos_valid = true;
      mot[i].pos = ed->get_enc(i);
      ostringstream oss;
      oss << "mot" << i;
      publish(oss.str().c_str(), mot[i]);
    }
    return true;
  } // end do_tick()

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  EtherDrive_Node a;
  while (a.ok()) {
    usleep(1000);
    if (!a.do_tick())
    {
      ROS_ERROR("Etherdrive tick failed.");
      break;
    }
  }
  ros::fini();
  return 0;
}
