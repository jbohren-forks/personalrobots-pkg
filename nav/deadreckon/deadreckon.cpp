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

#include <cmath>
#include "ros/node.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/RobotBase2DOdom.h"
#include "deadreckon/DriveDeadReckon.h"
using namespace std;
using namespace ros;

class DeadReckon : public ros::Node
{
public:
  std_msgs::BaseVel velMsg;
  std_msgs::RobotBase2DOdom odomMsg;
  double maxTV, maxRV, distEps, headEps, finalEps, tgtX, tgtY, tgtTh;
  enum
  {
    DR_FACE_TGT,
    DR_DRIVE_TO_TGT,
    DR_FACE_FINAL_HEADING,
    DR_IDLE
  } drState;

  DeadReckon() : Node("DeadReckon"), drState(DR_IDLE), tgtX(0), tgtY(0),
                 tgtTh(0)
  {
    param("drMaxTV", maxTV, 0.3);
    param("drMaxRV", maxRV, 0.3);
    param("drDistEps", distEps, 0.05);
    param("drHeadEps", headEps, 0.1);
    param("drFinalEps", finalEps, 0.05);
    advertise_service("DriveDeadReckon", &DeadReckon::dr_cb);
    subscribe("odom", odomMsg, &DeadReckon::odom_cb);
    advertise<std_msgs::BaseVel>("cmd_vel");
  }
  bool dr_cb(deadreckon::DriveDeadReckon::request  &req,
             deadreckon::DriveDeadReckon::response &res)
  {
    printf("handling dead reckon request: %f %f %f\n",
           req.dist,
           req.bearing,
           req.finalRelHeading);
    odomMsg.lock();
    tgtX  = odomMsg.pos.x + req.dist * cos(odomMsg.pos.th + req.bearing);
    tgtY  = odomMsg.pos.y + req.dist * sin(odomMsg.pos.th + req.bearing);
    tgtTh = normalizeAngle(odomMsg.pos.th + req.finalRelHeading);
    odomMsg.unlock();
    drState = DR_FACE_TGT;
    while (drState != DR_IDLE && ok())
      usleep(100000);
    res.status = "ok";
    return (drState == DR_IDLE);
  }
  double normalizeAngle(double ang)
  {
    while (ang > M_PI)
      ang -= 2 * M_PI;
    while (ang < -M_PI)
      ang += 2 * M_PI;
    return ang;
  }
  inline double saturate(double val, double low, double high)
  {
    return (val < low ? low : (val > high ? high : val));
  }
  void odom_cb()
  {
    velMsg.vx = velMsg.vw = 0;
    double dx = tgtX - odomMsg.pos.x, dy = tgtY - odomMsg.pos.y;
    double distToGo = sqrt(dx*dx + dy*dy);
    double bearing = normalizeAngle(atan2(dy, dx) - odomMsg.pos.th);
    bool backup = false;
    if (fabs(bearing) > M_PI/2)
    {
      backup = true;
      bearing = normalizeAngle(bearing + M_PI);
    }
    double finalBearing = normalizeAngle(tgtTh - odomMsg.pos.th);
    switch(drState)
    {
      case DR_FACE_TGT:
        velMsg.vx = 0;
        velMsg.vw = saturate(bearing, -maxRV, maxRV);
        if (fabs(bearing) < headEps)
        {
          drState = DR_DRIVE_TO_TGT;
          printf("driving to target\n");
        }
        break;
      case DR_DRIVE_TO_TGT:
        velMsg.vw = 0;
        if (!backup)
        {
          velMsg.vx = saturate(distToGo, 0.2, maxTV);
          if (distToGo > 0.3)
            velMsg.vw = saturate(bearing, -maxRV, maxRV);
        }
        else
          velMsg.vx = saturate(-distToGo, -0.2, -maxTV);
        if (distToGo < distEps)
        {
          drState = DR_FACE_FINAL_HEADING;
          printf("facing final heading\n");
        }
        break;
      case DR_FACE_FINAL_HEADING:
        velMsg.vx = 0;
        velMsg.vw = saturate(finalBearing, -maxRV, maxRV);
        if (fabs(finalBearing) < finalEps)
        {
          printf("going back to idle\n");
          drState = DR_IDLE;
        }
        break;
      case DR_IDLE:
        velMsg.vx = velMsg.vw = 0;
    }
    publish("cmd_vel", velMsg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  DeadReckon dr;
  dr.spin();
  ros::fini();
  return 0;
}

