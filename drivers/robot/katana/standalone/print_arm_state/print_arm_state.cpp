///////////////////////////////////////////////////////////////////////////////
// The katana package provides a simple wrapper library around the KNI library
// and a small suite of standalone executables and ROS nodes.
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

#include <cstdio>
#include "katana/katana.h"
using std::vector;

int main(int argc, char **argv)
{
  Katana *k = new Katana();
  vector<double> joints = k->get_joint_positions();
  vector<int> encoders = k->get_joint_encoders();
  vector<double> pose = k->get_pose();
 
	printf("encoders: ");
  for (size_t i = 0; i < encoders.size(); i++) {
    printf("%d  ", encoders[i]);
	}
	printf("\n");
  
  printf("joints (deg): ");
  for (size_t i = 0; i < joints.size(); i++) {
    printf("%f  ", joints[i]);
	}
	printf("\n");
  
  printf("joints (rad): ");
  for (size_t i = 0; i < joints.size(); i++) {
    printf("%f  ", joints[i]*3.14159/180.);
	}
	printf("\n");

	printf("pose: ");
  for (size_t i = 0; i < pose.size(); i++) {
    printf("%f ", pose[i]);
	}
	printf("\n"); 
  
  delete k;
 
 return 0;
}

