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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include <ros/node.h>
#include <math.h>
#include <std_msgs/ImuData.h>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <iostream>

using namespace std;

static NEWMAT::Matrix cross(double x, double y, double z) {
  NEWMAT::Matrix c(3,3);
  c << 0  << -z <<  y
    << z  <<  0 << -x
    << -y <<  x << 0;
  return c;
}

class simple_estimator: public ros::node
{
public:
  std_msgs::ImuData reading;

  NEWMAT::Matrix orientation;

  simple_estimator() : ros::node("simple_estimator")
  {
    subscribe<std_msgs::ImuData>("imu_data", reading, &simple_estimator::imu_callback);

    orientation = NEWMAT::IdentityMatrix(3);
  }

  ~simple_estimator() { }

  void imu_callback()
  {
    double x = reading.angrate.x * 0.01;
    double y = reading.angrate.y * 0.01;
    double z = reading.angrate.z * 0.01;

    double theta = sqrt( pow(x, 2.0) +
                         pow(y, 2.0) +
                         pow(z, 2.0));

    x /= theta;
    y /= theta;
    z /= theta;

    NEWMAT::Matrix w = cross(x, y, z);

    NEWMAT::Matrix rot = NEWMAT::IdentityMatrix(3) + w * sin(theta) + w*w*(1 - cos(theta));

    orientation = orientation*rot;

    std::cout << orientation << std::endl;
  }

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  simple_estimator se;
  se.spin();

  ros::fini();

  return(0);
}
