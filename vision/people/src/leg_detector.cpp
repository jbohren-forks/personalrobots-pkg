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

#include "laser_processor.h"
#include "calc_leg_features.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "rosrecord/Player.h"

#include "robot_msgs/PositionMeasurement.h"

using namespace std;
using namespace laser_processor;
using namespace ros;

class LegDetector : public node
{
public:
  std_msgs::LaserScan scan_;
  std_msgs::PointCloud cloud_;

  ScanMask mask_;

  int mask_count_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  LegDetector() : node("laser_processor"), mask_count_(0), connected_thresh_(0.08), feat_count_(0)
  {
    if (argc > 1) {
      forest.load(argv[1]);
      feat_count_ = forest.get_active_var_mask()->cols;
      printf("Loaded forest: %s\n", argv[1]);
    } else {
      printf("Please provide a trained random forests classifier as an input.\n");
      self_destruct();
    }

    subscribe("scan", scan_, &LegDetector::laserCallback, 10);
    advertise<std_msgs::PointCloud>("filt_cloud",10);
    advertise<robot_msgs::PositionMeasurement>("person_measurement",1);
  }

  void laserCallback()
  {
    cloud_.pts.clear();
    cloud_.chan.clear();
    cloud_.chan.resize(1);
    cloud_.chan[0].name = "rgb";

    ScanProcessor processor(scan_, mask_);

    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);

    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

    for (list<SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      vector<float> f = calcLegFeatures(*i, scan_);

      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)(f[k]);

      if (forest.predict( tmp_mat ) > 0)
      {
        (*i)->appendToCloud(cloud_, 255, 0, 0);

        robot_msgs::PositionMeasurement pos;
        pos.header.stamp = scan_.header.stamp;
        pos.header.frame_id = scan_.header.frame_id;
        pos.name = "leg_detector";
        pos.object_id = "unknown";
        pos.pos = (*i)->center();
        pos.reliability = 0.7;
        pos.covariance[0] = 1.0;
        pos.covariance[4] = 1.0;
        pos.covariance[8] = 0.1;
        pos.initialization = 0;
        
        publish("person_measurement", pos);
      }
    }

    cloud_.header = scan_.header;

    publish("filt_cloud", cloud_);

    cvReleaseMat(&tmp_mat);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  LegDetector ld;
  ld.spin();
  ros::fini();
  return 0;
}
