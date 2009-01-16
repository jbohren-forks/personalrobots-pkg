///////////////////////////////////////////////////////////////////////////////
// The deadreckon package provides a really dumb navigation style.
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
/////////////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include "ros/node.h"
#include "deadreckon/DriveDeadReckon.h"
using namespace std;
using namespace ros;

class DeadReckonTest : public ros::Node
{
public:
  DeadReckonTest() : Node("DeadReckonTest") { }
  void test_dr(float dist, float bearing, float relHeading)
  {
    deadreckon::DriveDeadReckon::request  req;
    deadreckon::DriveDeadReckon::response res;
    req.dist = dist;
    req.bearing = bearing;
    req.finalRelHeading = relHeading;
    bool ok = service::call("DriveDeadReckon", req, res);
    if (ok)
    {
      printf("drivedeadreckon succeeded. status: [%s]\n", res.status.c_str());
    }
    else
      printf("drivedeadreckon failed\n");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc < 4)
  {
    printf("usage: deadreckon_test DIST BEARING FINAL_HEADING\n");
    return 1;
  }
  DeadReckonTest drt;
  drt.test_dr(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  ros::fini();
}

