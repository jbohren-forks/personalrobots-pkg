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
#include "rosTF/rosTF.h"
#include "std_msgs/Actuator.h"
#include "etherdrive/etherdrive.h"
#include <sstream>

class EtherDrive_Node : public ros::node
{
public:
  rosTFServer tf;

  std_msgs::Actuator mot[6];
  std_msgs::Actuator mot_cmd[6];

  string host;
  EtherDrive *ed;

  float last_mot_val[6];
  double pulse_per_rad[6];

  string frame_id[6];
  string parent_id[6];
  int rot_axis[6];

  int count;
  ros::Time next_time;

  EtherDrive_Node() : ros::node("etherdrive"), tf(*this), ed(NULL), count(0)
  {

    for (int i = 0;i < 6;i++) {
      ostringstream oss;
      oss << "mot" << i;
      advertise<std_msgs::Actuator>(oss.str().c_str(), 1);

      oss << "_cmd";
      subscribe(oss.str().c_str(), mot_cmd[i], &EtherDrive_Node::mot_callback);

      last_mot_val[i] = 0;
      mot_cmd[i].valid = false;
    }

    param("etherdrive/host", host, string("192.168.0.100"));
    printf("EtherDrive host set to [%s]\n", host.c_str());

    for (int i = 0;i < 6;i++) {
      ostringstream oss;
      oss << "etherdrive/pulse_per_rad" << i;
      param(oss.str().c_str(), pulse_per_rad[i], 1591.54943);
      printf("Using %g pulse_per_rad for motor %d\n", pulse_per_rad[i], i);

      oss.str("");
      oss << "etherdrive/frame_id" << i;
      param(oss.str().c_str(), frame_id[i], string("BAD_ID"));

      oss.str("");
      oss << "etherdrive/parent_id" << i;
      param(oss.str().c_str(), parent_id[i], string("BAD_ID"));

      oss.str("");
      oss << "etherdrive/rot_axis" << i;
      param(oss.str().c_str(), rot_axis[i], 0);

      printf("Using frame %s and parent %s for motor %d\n", frame_id[i].c_str(), parent_id[i].c_str(), i);
    }

    ed = new EtherDrive();
    ed->init(host);
    ed->motors_on();

    next_time = ros::Time::now();
  }

  virtual ~EtherDrive_Node()
  {
    if (ed)
      delete ed;
  }

  bool do_tick()
  {

    count++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << " tics/sec at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }

    int tmp_mot_cmd[6];

    for (int i = 0; i < 6; i++) {

      mot_cmd[i].lock();
      if (mot_cmd[i].valid) {
	if (mot_cmd[i].rel) {
	  last_mot_val[i] = mot[i].val + mot_cmd[i].val;
	} else {
	  last_mot_val[i] = mot_cmd[i].val;
	}
      }
      mot_cmd[i].valid = false;  // set to invalid so we don't re-use

      tmp_mot_cmd[i] = (int)(last_mot_val[i] * pulse_per_rad[i]);

      mot_cmd[i].unlock();
    }

    ed->drive(6,tmp_mot_cmd);

    int val[6];

    ros::Time before = ros::Time::now();

    if (!ed->tick(6,val)) {
      return false;
    }

    ros::Time after = ros::Time::now();

    ros::Duration delta = after - before;

    ros::Time stamp = before + ros::Duration( delta.to_double() / 2.0 );

    for (int i = 0; i < 6; i++) {
      mot[i].val = val[i] / pulse_per_rad[i];
      mot[i].rel = false;
      mot[i].valid = true;
      mot[i].header.stamp = stamp;
      ostringstream oss;
      oss << "mot" << i;

      if (frame_id[i] != string("BAD_ID") && parent_id[i] != string("BAD_ID"))
      {

        tf.sendEuler(frame_id[i], parent_id[i],
                     0, 0, 0,
                     (rot_axis[i] == 1) ? mot[i].val : 0,
                     (rot_axis[i] == 2) ? mot[i].val : 0,
                     (rot_axis[i] == 3) ? mot[i].val : 0,
                     stamp);
      }
      publish(oss.str().c_str(), mot[i]);
    }
    return true;
  }

  void mot_callback() { }

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
