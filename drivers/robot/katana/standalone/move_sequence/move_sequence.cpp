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

/* Inputs:
* 		argv[1] = name of file containing sequence of joint angles (in radians)
*			argv[2] = desired time for movement between each joint angle position (in seconds)
*  Note: joint angles file should contain each joint angle configuration on a single line 
* 			(tab delimited).
*/

#include <cstdio>
#include <vector>
#include "katana/katana.h"
using std::vector;

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("usage: ./move_sequence jointAnglesFilename \n");
    return 1;
  }

  Katana *k = new Katana();
	vector<vector<double> > jointAngles;
	unsigned int numberOfMotors = k->get_number_of_motors();

	// Read in joint angle sequence from file.
	const char *jointsFilename = argv[1];
	FILE* f = fopen(jointsFilename, "r");
  if(f == NULL) {
		printf("Couldn't read joint angles file.\n");
		return 1;
	}
	char buf[512];
	char* p = NULL;
	char delim[] = "\t\n ";
	while (fgets(buf, 512, f) != NULL) {
		vector<double> joints;
		p = strtok(buf, delim);  
		joints.push_back(atof(p)*(180./3.14159));
//		joints.push_back(atof(p)*(3.14159/180.));
		for (unsigned int i=1; i<numberOfMotors; i++) {
			p = strtok(NULL,delim);
		  joints.push_back(atof(p)*(180./3.14159));
			//joints.push_back(atof(p)*(3.14159/180.));
		}
    jointAngles.push_back(joints);
	}

	for (unsigned int i=0; i<jointAngles.size(); i++) {
		printf("joints: %f %f %f %f %f\n",jointAngles[i][0],jointAngles[i][1],
					jointAngles[i][2],jointAngles[i][3],jointAngles[i][4]);
	}

	// Compute splines for trajectory and send motor commands to katana.
	//k->move_along_trajectory(jointAngles, atof(argv[2]));
  for (int i=0; i<jointAngles.size(); i++) {
    k->goto_joint_position_deg(jointAngles[i][0], jointAngles[i][1], 
      jointAngles[i][2], jointAngles[i][3], jointAngles[i][4]); 
  }

  delete k;

  return 0;
}

