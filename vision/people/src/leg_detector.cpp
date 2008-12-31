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

#include "rostools/Header.h"

#include "tf/transform_listener.h"

using namespace std;
using namespace laser_processor;
using namespace ros;

class SavedFeature
{
public:
  int id_;
  string object_id;
  ros::Time time_;
  tf::Stamped<tf::Point> loc_;
  vector<float>    features_;
  vector<char>    color_;

  SavedFeature(int id, std_msgs::LaserScan& scan, tf::Stamped<tf::Point> loc, vector<float> features, char r, char g, char b)
  {
    id_ = id;
    object_id = "none";
    time_ = scan.header.stamp;
    loc_ = loc;
    features_ = features;
    color_.push_back(r);
    color_.push_back(g);
    color_.push_back(b);
  }

  void update(std_msgs::LaserScan& scan, tf::Stamped<tf::Point> loc, vector<float> features)
  {
    time_ = scan.header.stamp;
    loc_ = loc;
    features_ = features;
  }
};

class PropagatedFeature
{
public:
  list<SavedFeature>::iterator saved_;
  tf::Stamped<tf::Point> loc_;

  PropagatedFeature(list<SavedFeature>::iterator saved, tf::Stamped<tf::Point> loc) : saved_(saved), loc_(loc) { }
};

class MatchedFeature
{
public:
  list<SampleSet*>::iterator good_;
  list<PropagatedFeature>::iterator closest_;
  float distance_;

  MatchedFeature(list<SampleSet*>::iterator good, list<PropagatedFeature>::iterator closest, float distance)
  : good_(good)
  , closest_(closest)
  , distance_(distance)
  {}

  inline bool operator< (const MatchedFeature& b) const
  {
    return (distance_ <  b.distance_);
  }
};

class LegDetector : public node
{
public:
  tf::TransformListener tf;

  std_msgs::LaserScan scan_;
  std_msgs::PointCloud cloud_;

  ScanMask mask_;

  int mask_count_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  list<SavedFeature> saved_features_;
  boost::mutex saved_mutex_;

  int feature_id_;

  robot_msgs::PositionMeasurement people_meas_;

  LegDetector() : node("laser_processor"), tf(*this), mask_count_(0), connected_thresh_(0.06), feat_count_(0)
  {
    if (argc > 1) {
      forest.load(argv[1]);
      feat_count_ = forest.get_active_var_mask()->cols;
      printf("Loaded forest with %d features: %s\n", feat_count_, argv[1]);
    } else {
      printf("Please provide a trained random forests classifier as an input.\n");
      self_destruct();
    }

    subscribe("scan", scan_, &LegDetector::laserCallback, 10);
    advertise<std_msgs::PointCloud>("filt_cloud",10);
    advertise<robot_msgs::PositionMeasurement>("people_tracking_measurements",1);
    subscribe("people_tracker_filter", people_meas_, &LegDetector::peopleCallback, 10);

    feature_id_ = 0;
  }


  void peopleCallback()
  {
    tf::Point pt;

    tf::PointMsgToTF(people_meas_.pos, pt);

    tf::Stamped<tf::Point> loc(pt, people_meas_.header.stamp, people_meas_.header.frame_id);

    boost::mutex::scoped_lock lock(saved_mutex_);

    list<SavedFeature>::iterator closest = saved_features_.end();
    float closest_dist = 1.0;
      
    list<SavedFeature>::iterator it  = saved_features_.begin();
    list<SavedFeature>::iterator end = saved_features_.end();

    for (; it != end; ++it )
    {
      if (it->object_id == people_meas_.object_id)
        return;

      try
      {
        tf.transformPoint(it->loc_.frame_id_, it->loc_.stamp_,
                          loc, "odom_combined", loc);
      } 
      catch (tf::TransformException& ex)
      {
      }

      float dist = loc.distance(it->loc_);
      if ( dist < closest_dist )
      {
        closest = it;
        closest_dist = dist;
      }
    }

    if (closest_dist < 1.0)
      closest->object_id = people_meas_.object_id;
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

    ros::Time purge = scan_.header.stamp + ros::Duration().fromSec(-2.0);

    list<SavedFeature>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end())
    {
      if (sf_iter->time_ < purge)
        saved_features_.erase(sf_iter++);
      else
        ++sf_iter;
    }

    // Build up the set of "good" clusters
    list<SampleSet*> good;

    for (list<SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      vector<float> f = calcLegFeatures(*i, scan_);

      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)(f[k]);

      if (forest.predict( tmp_mat ) > 0)
      {
        good.push_back(*i);
      }
    }

    // Transform our saved features forward in time.
    list<PropagatedFeature> propagated;
    for (list<SavedFeature>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++)
    {
      tf::Stamped<tf::Point> sf_loc;
      if (tf.canTransform(scan_.header.frame_id, scan_.header.stamp,
                          sf_iter->loc_.frame_id_, sf_iter->loc_.stamp_,
                          "odom_combined"))
      {
        tf.transformPoint(scan_.header.frame_id, scan_.header.stamp,
                          sf_iter->loc_, "odom_combined", sf_loc);
      } else {
        sf_loc = sf_iter->loc_;
      }
      propagated.push_back(PropagatedFeature(sf_iter, sf_loc));
    }

    multiset<MatchedFeature> matches;

    //Find point which is closest to a saved feature, within threshold
    for (list<SampleSet*>::iterator i = good.begin();
         i != good.end();
         i++)
    {
      tf::Stamped<tf::Point> loc((*i)->center(), scan_.header.stamp, scan_.header.frame_id);

      list<PropagatedFeature>::iterator closest = propagated.end();
      float closest_dist = 1.0;
      
      for (list<PropagatedFeature>::iterator pf_iter = propagated.begin();
           pf_iter != propagated.end();
           pf_iter++)
      {
        // If it is close to a saved feature
        float dist = loc.distance(pf_iter->loc_);
        if ( dist < closest_dist )
        {
          closest = pf_iter;
          closest_dist = dist;
        }
      }
      
      if (closest == propagated.end())
      {
        vector<float> f = calcLegFeatures(*i, scan_);
        list<SavedFeature>::iterator new_saved = saved_features_.insert(saved_features_.end(), SavedFeature(feature_id_++, scan_, loc, f, rand()%255, rand()%255, rand()%255));

        (*i)->appendToCloud(cloud_, new_saved->color_[0], new_saved->color_[1], new_saved->color_[2]);
      }
      else
      {
        matches.insert(MatchedFeature(i,closest,closest_dist));
      }
    }

    while (matches.size() > 0)
    {
      multiset<MatchedFeature>::iterator matched_iter = matches.begin();
      bool found = false;
      list<PropagatedFeature>::iterator pf_iter = propagated.begin();
      while (pf_iter != propagated.end())
      {
        if (matched_iter->closest_ == pf_iter)
        {
          // It was actually the closest... do our thing
          vector<float> f = calcLegFeatures(*matched_iter->good_, scan_);
          tf::Stamped<tf::Point> loc((*matched_iter->good_)->center(), scan_.header.stamp, scan_.header.frame_id);
          matched_iter->closest_->saved_->update(scan_, loc, f);
          
          (*matched_iter->good_)->appendToCloud(cloud_, matched_iter->closest_->saved_->color_[0], matched_iter->closest_->saved_->color_[1], matched_iter->closest_->saved_->color_[2]);

          matches.erase(matched_iter);
          propagated.erase(pf_iter++);
          found = true;
          break;
        }
        else
        {
          pf_iter++;
        }
      }
      if (!found)
      {
        // Search for new closest:
        tf::Stamped<tf::Point> loc((*matched_iter->good_)->center(), scan_.header.stamp, scan_.header.frame_id);

        list<PropagatedFeature>::iterator closest = propagated.end();
        float closest_dist = 1.0;
      
        for (list<PropagatedFeature>::iterator remain_iter = propagated.begin();
             remain_iter != propagated.end();
             remain_iter++)
        {
          // If it is close to a saved feature
          float dist = loc.distance(remain_iter->loc_);
          if ( dist < closest_dist )
          {
            closest = remain_iter;
            closest_dist = dist;
          }
        }

        if (closest == propagated.end())
        {
          vector<float> f = calcLegFeatures(*matched_iter->good_, scan_);
          list<SavedFeature>::iterator new_saved = saved_features_.insert(saved_features_.end(), SavedFeature(feature_id_++, scan_, loc, f, rand()%255, rand()%255, rand()%255));

          (*matched_iter->good_)->appendToCloud(cloud_, new_saved->color_[0], new_saved->color_[1], new_saved->color_[2]);
          matches.erase(matched_iter);
        }
        else
        {
          matches.insert(MatchedFeature(matched_iter->good_,closest,closest_dist));
          matches.erase(matched_iter);
        }
      }
    }

    cloud_.header = scan_.header;
    publish("filt_cloud", cloud_);
    cvReleaseMat(&tmp_mat);

    for (list<SavedFeature>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++)
    {
      if (sf_iter->object_id != "none" && sf_iter->time_ == scan_.header.stamp)
      {
        robot_msgs::PositionMeasurement pos;

        pos.header.stamp = sf_iter->time_;
        pos.header.frame_id = sf_iter->loc_.frame_id_;
        pos.name = "leg_detector";
        pos.object_id = sf_iter->object_id;
        tf::PointTFToMsg(sf_iter->loc_,pos.pos);
        pos.reliability = 0.7;
        pos.covariance[0] = 1.0;
        pos.covariance[4] = 1.0;
        pos.covariance[8] = 0.1;
        pos.initialization = 0;
        
        publish("people_tracker_measurements", pos);        
      }
    }
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

