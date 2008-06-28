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
#include <iostream>
#include "katana/katana.h"
#include <math.h>
using std::vector;
using std::cout;
using std::endl;

#define PI 3.14159265358979

int main(int argc, char **argv)
{
  if (argc != 7)
  {
    printf("usage: get_joint_solution x y z theta psi max_theta_dev\n");
    return 1;
  }
  Katana *k = new Katana();

  double x = atof(argv[1]);
  double y = atof(argv[2]);
  double z = atof(argv[3]);
	double phi = 0;		// phi is determined by x and y....doesn't matter what value it's set to!

	cout << "phi = " << phi << endl;
  double theta = atof(argv[4]), theta_init = atof(argv[4]);
  double psi = atof(argv[5]);
	double max_theta_dev = atof(argv[6]);
	vector<double> jointAngles;
	
	if (k->ik_joint_solution(x,y,z,theta,psi,max_theta_dev,jointAngles)) {
		cout << "Received solution: ";
   	 	for (int i=0; i<jointAngles.size(); i++) {
   	   cout << jointAngles[i]*180./PI << " ";
   	 	}
   		cout << endl;
	}

  delete k;
  return 0;
}

