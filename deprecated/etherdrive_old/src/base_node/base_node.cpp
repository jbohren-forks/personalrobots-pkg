///////////////////////////////////////////////////////////////////////////////
// This node talks to the etherdrive boards that drive the casters as well as
// takes commands from other nodes.
//
// Copyright (C) 2008, Jimmy Sastra
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
    std_msgs::ActuatorState mot[12];
    std_msgs::ActuatorCmd mot_cmd[12];

    string host;
    string host2;
    EtherDrive *edRight;
    EtherDrive *edLeft;
    int frame_id;

    float last_mot_val[12];
    double pulse_per_rad[12];

    int lastEncPos[12];
    int mode; // 0=voltage, 1=current, 2=position, 3=velocity, 4=position on comp
    bool firstTimeVelCtrl;
    int fooCounter;
    int counter;
    float last_mot_cmd[12];


    EtherDrive_Node() : ros::node("etherdrive"), edLeft(NULL)
    {
      for (int i = 0;i < 12;i++)
      {
        ostringstream oss;
        oss << "mot" << i;
        advertise<std_msgs::ActuatorState>(oss.str().c_str());

        oss << "_cmd";
        subscribe(oss.str().c_str(), mot_cmd[i], &EtherDrive_Node::mot_callback);
        last_mot_val[i] = 0;
      }

      //if (!get_param(string(".host"), host))
      // host = "192.168.1.12";
      // host[0] = "10.12.0.103";
      host = "10.11.0.102";
      host2 = "10.12.0.103";
      printf("EtherDrive host set to [%s]\n",host.c_str());

      //if (!get_param(string(".frame_id"), frame_id))
      frame_id = -1;
      printf("EtherDrive frame_id set to [%d]\n", frame_id);

      for (int i = 0;i < 12;i++)
      {
        ostringstream oss;
        oss << "pulse_per_rad" << i;
        //if (!get_param(oss.str().c_str(), pulse_per_rad[i]))
        pulse_per_rad[i] = 1591.54943;
        printf("Using %g pulse_per_rad for motor %d\n", pulse_per_rad[i], i);
      }

      edLeft = new EtherDrive();
      edLeft->init(host, "10.11.0.3");
      edRight = new EtherDrive();
      edRight->init(host2, "10.12.0.2");


      // debugging JS: initialize values
      for (int i = 0; i < 12; i++)
      {
        mot_cmd[i].valid = true;
        mot_cmd[i].rel = 0;
        mot_cmd[i].cmd = 0;
      }
      // debugging JS: initialize values
      for (int i = 0; i < 6; i++)
      {
        // setting gains on controls on ethercard drive
        edLeft->set_gain(i,'P',0);
        edLeft->set_gain(i,'I',10);
        edLeft->set_gain(i,'D',0);
        edLeft->set_gain(i,'W',100);
        edLeft->set_gain(i,'M',1004);
        edLeft->set_gain(i,'Z',1);
        edRight->set_gain(i,'P',0);
        edRight->set_gain(i,'I',10);
        edRight->set_gain(i,'D',0);
        edRight->set_gain(i,'W',100);
        edRight->set_gain(i,'M',1004);
        edRight->set_gain(i,'Z',1);
      }


      edLeft->set_control_mode(1); // 0=voltage, 1=current control
      edRight->set_control_mode(1); // 0=voltage, 1=current control

      firstTimeVelCtrl = true;
      fooCounter = 0;
      edLeft->motors_on();
      edRight->motors_on();

      counter = 0;
    }

    void mot_callback()
    {
      //printf("received motor 3 command %f\n", mot_cmd[3].val);
    }

    virtual ~EtherDrive_Node()
    {
      if (edRight)
      {
        printf("deleting edRight\n");
        delete edRight;
      }
      if (edLeft)
      {
        printf("deleting edLeft\n");
        delete edLeft;
      }
      int footmp[6]  = {0,0,0,0,0,0};
      edLeft->drive(6,footmp);
      edRight->drive(6,footmp);
    }

    bool do_tick()
    {

      int tmp_mot_cmd[12];
      int tmp_mot_cmd_left[6];
      int tmp_mot_cmd_right[6];
      int currEncPos[12];

      float curVel;
      float desVel;
      float desPos[12];
      // float k_p[12]  = {5,5,-1,5,5,-1, 5,5,-1,5,5,-1};
      float k_p[12]     = {5,5,-0.5,5,5,-0.5, 5,5,-0.5,5,5,-0.5};
      // float k_p[12]  = {0,0,0,0,0,0,0,0,0,0,0,0};
      float k_i[12]     = {0,0,0,0,0,0,0,0,0,0,0,0};
      float k_d[12]     = {0,0,0,0,0,0,0,0,0,0,0,0};
      // float k_i[12]  = {0,0,-0.01,0,0,-0.01, 0,0,-0.01,0,0,0};
      // float k_d[12]  = {0,0,-0.08,0,0,-0.08, 0,0,-0.08,0,0,-0.08};
      float k_cross[12] = {-1, -1, 0 , -1, -1, 0, -1, -1, 0 , -1, -1, 0};
      float foo;

      float posError[12];
      float intPosError[12];
      float lastPosError[12];


      for (int i = 0; i < 12; i++)
      {
        mot_cmd[i].lock();
        if(i == 2|i == 5|i == 8|i == 11)
        { // turret motors, position control
          desPos[i] = float(mot_cmd[i].cmd)/360*90000;
          if(i < 6) posError[i] =   desPos[i] - float(edLeft->get_enc(i));
          else posError[i] = desPos[i] - float(edRight->get_enc(i-6));
  if(i==11) printf("posError: %f, despos: %f, enc: %i\n", posError[i], desPos[i], edRight->get_enc(i-6));

          tmp_mot_cmd[i] = int(k_p[i]*posError[i]) + int(k_d[i]*(posError[i]-lastPosError[i])/0.001) + int(k_i[i]*intPosError[i]);

          lastPosError[i] = posError[i];
          intPosError[i] += posError[i];
        }
        else
        { // wheels, velocity control

          if(i<6) currEncPos[i] = edLeft->get_enc(i);
          else currEncPos[i] = edRight->get_enc(i-6);
          curVel =  float(currEncPos[i] - lastEncPos[i])/9000*0.48;
          desVel = mot_cmd[i].cmd; // enc units/msec.  should be (wheel rev/s)
          if(i == 0| i ==1) tmp_mot_cmd[i] = int(k_p[i]*(desVel - curVel) + k_cross[i]*tmp_mot_cmd[2]);
          else if(i == 3| i ==4) tmp_mot_cmd[i] = int(k_p[i]*(desVel - curVel) + k_cross[i]*tmp_mot_cmd[5]);
          else if(i == 6| i ==7) tmp_mot_cmd[i] = int(k_p[i]*(desVel - curVel) + k_cross[i]*tmp_mot_cmd[8]);
          else if(i == 9| i ==10) tmp_mot_cmd[i] = int(k_p[i]*(desVel - curVel) + k_cross[i]*tmp_mot_cmd[11]);
          lastEncPos[i] = currEncPos[i];
        }

        mot_cmd[i].valid = false;  // set to invalid so we don't re-use
        mot_cmd[i].unlock();
        if(tmp_mot_cmd[i] > 3000)
        {
          tmp_mot_cmd[i] = 3000;
          // printf("Max +ve current motor %i\n", i);
        }
        if(tmp_mot_cmd[i] < -3000)
        {
          tmp_mot_cmd[i] = -3000;
          // printf("Max -ve current motor %i \n", i);
        }

      }

      // edLeft->drive(6,tmp_mot_cmd);
      for(int i = 0; i<6; i++)
      {
        tmp_mot_cmd_left[i] = tmp_mot_cmd[i];
        tmp_mot_cmd_right[i] = tmp_mot_cmd[i+6];
      }
      edLeft->drive(6,tmp_mot_cmd_left);
      edRight->drive(6,tmp_mot_cmd_right);
      // int footmp[6]  = {500,500,0,500,500,0};
      // edRight->drive(6,footmp);
      // edLeft->drive(6,footmp);

      int valLeft[6];
      int valRight[6];
      if (!edLeft->tick(6,valLeft))
      {
        return false;
      }
      if (!edRight->tick(6,valRight))
      {
        return false;
      }


      for (int i = 0; i < 6; i++)
      {
        mot[i].pos_valid = true;
        mot[i].pos = edLeft->get_enc(i);
        mot[i+6].pos = edRight->get_enc(i);
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
printf("start\n");
  while (a.ok())
  {
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
