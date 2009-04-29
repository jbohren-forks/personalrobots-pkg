/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Wim Meeussen */


#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>

using namespace tf;
using namespace ros;
using namespace std;


int main(int argc, char ** argv)
{
  //Initialize ROS
  init(argc, argv);

  string framea, frameb;
  if (argc == 3){
    framea = argv[1];
    frameb = argv[2];
  }
  else{
    ROS_INFO("TF_Monitor: usage: tf_monitor framea frameb");
    return -1;
  }

  Node node_("tf_monitor");

  // create tf listener
  TransformListener tf_(node_);
  double max_diff = 0;
  double avg_diff = 0;
  double lowpass = 0.01;
  unsigned int counter = 0;

  Stamped<Transform> tmp;
  cout << "Waiting for first transfrom to become afailable" << flush;
  while (node_.ok() && !tf_.canTransform(framea, frameb, Time(), Duration(1.0)))
    cout << "." << flush;
  cout << endl;


  while (node_.ok()){
    tf_.lookupTransform(framea, frameb, Time(), tmp);
    double diff = (Time::now() - tmp.stamp_).toSec();
    avg_diff = lowpass * diff + (1-lowpass)*avg_diff;
    if (diff > max_diff) max_diff = diff;
    Duration(0.01).sleep();
    counter++;
    if (counter > 100){
      counter = 0;
      cout << "max = " << max_diff << "    avg = " << avg_diff << endl;
    }
  }

  return 0;
}
