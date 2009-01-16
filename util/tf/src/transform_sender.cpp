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

#include "tf/transform_broadcaster.h"

class TransformSender : public ros::Node
{
public:
  //constructor
  TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& parent_id) : 
    ros::Node("transform_sender", ros::Node::ANONYMOUS_NAME),broadcaster(*this), 
    transform_(btTransform(btQuaternion(yaw,pitch,roll), btVector3(x,y,z)), time, frame_id , parent_id){};
  //Clean up ros connections
  ~TransformSender() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;

  

  // A function to call to send data periodically
  void send () {
    broadcaster.sendTransform(transform_);
  };

private:
  tf::Stamped<tf::Transform> transform_;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv);

  if(argc != 10)
    {
      printf("A command line utility for manually sending a transform.\n");
      printf("It will periodicaly republish the given transform. \n");
      printf("Usage: transform_sender x y z yaw pitch roll frame_id parent_id  period(miliseconds) \n");
      ROS_ERROR("transform_sender exited due to not having the right number of arguments");
      return -1;
    }

  TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                            atof(argv[4]), atof(argv[5]), atof(argv[6]),
                            ros::Time::now(), 
                            argv[7], argv[8]);



  while(tf_sender.ok())
  {
    tf_sender.send();
    ROS_INFO("Sending transform from %s with parent %s\n", argv[7], argv[8]);
    usleep(atoi(argv[9])*1000);
  }
  ros::fini();

  return 0;
};

