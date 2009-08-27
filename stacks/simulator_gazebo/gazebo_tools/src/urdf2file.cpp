/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <gazebo_tools/urdf2gazebo.h>

using namespace urdf2gazebo;

void usage(const char *progname)
{
    printf("\nUsage: %s URDF.xml Gazebo.model\n", progname);
    printf("       where URDF.xml is the file containing a robot description in the Willow Garage format (URDF)\n");
    printf("       and Gazebo.model is the file where the Gazebo model should be written\n\n");
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        usage(argv[0]);
        exit(1);
    }
    
    // Experimental: take in optional argument to disable passing the limit for grasping test
    bool enforce_limits = true;
    if (argc == 4)
    {
        printf("Not enforcing limits\n");
        enforce_limits = false;
    }
    
    TiXmlDocument urdf_in(argv[1]), xml_out;
    urdf_in.LoadFile();
    
    // default initial pose is all 0's
    urdf::Vector3 initial_xyz;
    urdf::Vector3 initial_rpy;

    URDF2Gazebo u2g(std::string("pr2_model"));
    u2g.convert(urdf_in, xml_out, enforce_limits,initial_xyz,initial_rpy,true);
    
    if (!xml_out.SaveFile(argv[2]))
    {
        printf("Unable to save gazebo model in %s\n", argv[2]);  
        exit(3);
    }
    
    return 0;
}
