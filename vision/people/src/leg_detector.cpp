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
#include <tf/message_notifier.h>

#include "filter/tracker_kalman.h"
#include "filter/state_pos_vel.h"
#include "filter/state_vector.h"
#include "filter/rgb.h"

#include <algorithm>

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;

using namespace estimation;
using namespace BFL;

using namespace MatrixWrapper;

class SavedFeature
{
public:
  static int nextid;
  string id_;
  string object_id;
  ros::Time time_;
  ros::Time meas_time_;
  vector<char>     color_;

  tf::TransformListener& tfl_;

  BFL::StatePosVel sys_sigma_;
  TrackerKalman filter_;

  tf::Stamped<tf::Point> prop_loc_;

  SavedFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl)
    : tfl_(tfl),
      sys_sigma_(tf::Vector3(0.05/20.0,0.05/20.0,0.05/20.0), tf::Vector3(1.0/20.0,1.0/20.0,1.0/20.0)),
      filter_("tracker_name",sys_sigma_)
  {
    char id[100];
    snprintf(id,100,"legtrack%d", nextid++);
    id_ = std::string(id);

    object_id = "";
    time_ = loc.stamp_;
    meas_time_ = loc.stamp_;

    tf::Stamped<btTransform> pose( btTransform(tf::Quaternion(), loc), loc.stamp_, id_, loc.frame_id_);
    tfl_.setTransform(pose);

    StatePosVel prior_sigma(StateVector(0.1,0.1,0.1), StateVector(0.0000001, 0.0000001, 0.0000001));
    filter_.initialize(loc, prior_sigma, time_.toSec());    

    color_.push_back(rand()%255);
    color_.push_back(rand()%255);
    color_.push_back(rand()%255);
  }

  void propagate(ros::Time time)
  {
    time_ = time;

    filter_.updatePrediction(time.toSec());

    StatePosVel est;
    filter_.getEstimate(est);

    prop_loc_[0] = est.pos_[0];
    prop_loc_[1] = est.pos_[1];
    prop_loc_[2] = est.pos_[2];
    prop_loc_.stamp_ = time;
    prop_loc_.frame_id_ = "odom";

    /*
    tf::Stamped<tf::Point> orig_loc(tf::Point(0,0,0),  meas_time_, id_);
    tf::Stamped<tf::Point> inter_loc;

    tfl_.transformPoint("odom", orig_loc, inter_loc);
      
    inter_loc.stamp_ = time_;
      
    tfl_.transformPoint("odom", inter_loc, prop_loc_);
    */
    
  }

  void update(tf::Stamped<tf::Point> loc)
  {
    tf::Stamped<btTransform> pose( btTransform(tf::Quaternion(), loc), loc.stamp_, id_, loc.frame_id_);
    tfl_.setTransform(pose);

    meas_time_ = loc.stamp_;
    time_ = meas_time_;

    SymmetricMatrix cov(3);
    cov = 0.0;
    cov(1,1) = 0.0025;
    cov(2,2) = 0.0025;
    cov(3,3) = 0.0025;

    filter_.updateCorrection(loc, cov);
  }
};

int SavedFeature::nextid = 0;

class MatchedFeature
{
public:
  SampleSet* candidate_;
  SavedFeature* closest_;
  float distance_;

  MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance)
  : candidate_(candidate)
  , closest_(closest)
  , distance_(distance)
  {}

  inline bool operator< (const MatchedFeature& b) const
  {
    return (distance_ <  b.distance_);
  }
};


static const string fixed_frame = "odom";


class LegDetector : public node
{
public:
  tf::TransformListener tfl_;

  std_msgs::PointCloud filt_cloud_;

  ScanMask mask_;

  int mask_count_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  list<SavedFeature*> saved_features_;
  boost::mutex saved_mutex_;

  int feature_id_;

  float max_track_jump_;

  float max_meas_jump_;

  MessageNotifier<robot_msgs::PositionMeasurement>*  people_notifier_;
  MessageNotifier<std_msgs::LaserScan>*  laser_notifier_;

  LegDetector() : 
    node("laser_processor"), 
    tfl_(*this), 
    mask_count_(0), 
    connected_thresh_(0.06), 
    feat_count_(0), 
    max_track_jump_(1.0), 
    max_meas_jump_(1.0)
  {
    if (argc > 1) {
      forest.load(argv[1]);
      feat_count_ = forest.get_active_var_mask()->cols;
      printf("Loaded forest with %d features: %s\n", feat_count_, argv[1]);
    } else {
      printf("Please provide a trained random forests classifier as an input.\n");
      self_destruct();
    }

    // advertise topics
    advertise<std_msgs::PointCloud>("filt_cloud",10);
    advertise<std_msgs::PointCloud>("kalman_filt_cloud",10);
    advertise<robot_msgs::PositionMeasurement>("people_tracker_measurements",1);

    // subscribe to topics
    people_notifier_ = new MessageNotifier<robot_msgs::PositionMeasurement>(&tfl_, this,  
									    boost::bind(&LegDetector::peopleCallback, this, _1), 
									    "people_tracker_filter", fixed_frame, 10);
    laser_notifier_ = new MessageNotifier<std_msgs::LaserScan>(&tfl_, this,  
							       boost::bind(&LegDetector::laserCallback, this, _1), 
							       "scan", fixed_frame, 10);

    feature_id_ = 0;
  }


  ~LegDetector()
  {
    delete people_notifier_;
    delete laser_notifier_;
  }


  void peopleCallback(const MessageNotifier<robot_msgs::PositionMeasurement>::MessagePtr& people_meas)
  {
    tf::Point pt;

    tf::PointMsgToTF(people_meas->pos, pt);

    boost::mutex::scoped_lock lock(saved_mutex_);

    list<SavedFeature*>::iterator closest = saved_features_.end();
    float closest_dist = max_meas_jump_;
      
    list<SavedFeature*>::iterator it  = saved_features_.begin();
    list<SavedFeature*>::iterator end = saved_features_.end();

    for (; it != end; ++it )
    {
      tf::Stamped<tf::Point> loc(pt, people_meas->header.stamp, people_meas->header.frame_id);


      tfl_.transformPoint((*it)->id_, people_meas->header.stamp,
                            loc, fixed_frame, loc);
      loc[2] = 0.0;
      
      float dist = loc.length();
      if ( dist < closest_dist )
      {
        closest = it;
        closest_dist = dist;
      }

      // If we're already tracking the object
      if ((*it)->object_id == people_meas->object_id)
      {
        if (dist < max_meas_jump_)
          return; // This should instead check that it's still close
        else
        {
          (*it)->object_id = "";
        }
      }
    }

    if (closest_dist < max_meas_jump_)
      (*closest)->object_id = people_meas->object_id;
  }



  void laserCallback(const MessageNotifier<std_msgs::LaserScan>::MessagePtr& scan)
  {
    filt_cloud_.pts.clear();
    filt_cloud_.chan.clear();
    filt_cloud_.chan.resize(1);
    filt_cloud_.chan[0].name = "rgb";

    ScanProcessor processor(*scan, mask_);

    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);

    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-5.0);

    list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end())
    {
      if ((*sf_iter)->meas_time_ < purge)
      {
        delete (*sf_iter);
        saved_features_.erase(sf_iter++);
      }
      else
        ++sf_iter;
    }
    // Build up the set of "candidate" clusters
    list<SampleSet*> candidates;

    for (list<SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      vector<float> f = calcLegFeatures(*i, *scan);

      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)(f[k]);

      if (forest.predict( tmp_mat ) > 0)
      {
        candidates.push_back(*i);
      }
    }

    list<SavedFeature*> propagated;
    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++)
    {
      (*sf_iter)->propagate(scan->header.stamp);
      propagated.push_back(*sf_iter);
    }

    multiset<MatchedFeature> matches;

    //Find point which is closest to a saved feature, within threshold
    for (list<SampleSet*>::iterator cf_iter = candidates.begin();
         cf_iter != candidates.end();
         cf_iter++)
    {
      tf::Stamped<tf::Point> loc((*cf_iter)->center(), scan->header.stamp, scan->header.frame_id);
      tfl_.transformPoint("odom", loc, loc);

      list<SavedFeature*>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_;
      
      for (list<SavedFeature*>::iterator pf_iter = propagated.begin();
           pf_iter != propagated.end();
           pf_iter++)
      {
        // If it is close to a saved feature
        float dist = loc.distance((*pf_iter)->prop_loc_);
        if ( dist < closest_dist )
        {
          closest = pf_iter;
          closest_dist = dist;
        }
      }
      
      if (closest == propagated.end()) // Nothing close to it, start a new track
      {
        list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));

        (*cf_iter)->appendToCloud(filt_cloud_, (*new_saved)->color_[0], (*new_saved)->color_[1], (*new_saved)->color_[2]);
      }
      else
      {
        matches.insert(MatchedFeature(*cf_iter,*closest,closest_dist));
      }
    }

    while (matches.size() > 0)
    {
      multiset<MatchedFeature>::iterator matched_iter = matches.begin();
      bool found = false;
      list<SavedFeature*>::iterator pf_iter = propagated.begin();
      while (pf_iter != propagated.end())
      {
        if (matched_iter->closest_ == *pf_iter)
        {
          // It was actually the closest... do our thing
          tf::Stamped<tf::Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
          tfl_.transformPoint("odom", loc, loc);          

          matched_iter->closest_->update(loc);
          
          matched_iter->candidate_->appendToCloud(filt_cloud_, matched_iter->closest_->color_[0], matched_iter->closest_->color_[1], matched_iter->closest_->color_[2]);

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
        tf::Stamped<tf::Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
        tfl_.transformPoint("odom", loc, loc);

        list<SavedFeature*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_;
      
        for (list<SavedFeature*>::iterator remain_iter = propagated.begin();
             remain_iter != propagated.end();
             remain_iter++)
        {
          // If it is close to a saved feature
          float dist = loc.distance((*remain_iter)->prop_loc_);
          if ( dist < closest_dist )
          {
            closest = remain_iter;
            closest_dist = dist;
          }
        }

        if (closest == propagated.end())
        {
          list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));

          matched_iter->candidate_->appendToCloud(filt_cloud_, (*new_saved)->color_[0], (*new_saved)->color_[1], (*new_saved)->color_[2]);
          matches.erase(matched_iter);
        }
        else
        {
          matches.insert(MatchedFeature(matched_iter->candidate_,*closest,closest_dist));
          matches.erase(matched_iter);
        }
      }
    }

    filt_cloud_.header.stamp = scan->header.stamp;
    filt_cloud_.header.frame_id = scan->header.frame_id;
    publish("filt_cloud", filt_cloud_);
    cvReleaseMat(&tmp_mat);

    vector<std_msgs::Point32> filter_visualize(saved_features_.size());
    vector<float> weights(saved_features_.size());
    std_msgs::ChannelFloat32 channel;
    int i = 0;

    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++,i++)
    {
      // publish filter result
      StatePosVel est;
      (*sf_iter)->filter_.getEstimate(est);
      
      filter_visualize[i].x = est.pos_[0];
      filter_visualize[i].y = est.pos_[1];
      filter_visualize[i].z = est.pos_[2];

      int rgb = ((*sf_iter)->color_[0] << 16) | ((*sf_iter)->color_[1] << 8) | (*sf_iter)->color_[2];

      weights[i] = *(float*)&(rgb);

      if ((*sf_iter)->meas_time_ == scan->header.stamp)
      {
        if (est.vel_.length() > 0.5)
        {
          robot_msgs::PositionMeasurement pos;

          pos.header.stamp = (*sf_iter)->time_;
          pos.header.frame_id = "odom";
          pos.name = "leg_detector";
          pos.object_id = (*sf_iter)->object_id;
          pos.pos.x = est.pos_[0];
          pos.pos.y = est.pos_[1];
          pos.pos.z = est.pos_[2];
          pos.reliability = 0.80;
          pos.covariance[0] = 0.09;
          pos.covariance[1] = 0.0;
          pos.covariance[2] = 0.0;
          pos.covariance[3] = 0.0;
          pos.covariance[4] = 0.09;
          pos.covariance[5] = 0.0;
          pos.covariance[6] = 0.0;
          pos.covariance[7] = 0.0;
          pos.covariance[8] = 10000.0;
          pos.initialization = 0;
        
          publish("people_tracker_measurements", pos);               
        }
        else if ((*sf_iter)->object_id != "")
        {
          robot_msgs::PositionMeasurement pos;

          pos.header.stamp = (*sf_iter)->time_;
          pos.header.frame_id = "odom";
          pos.name = "leg_detector";
          pos.object_id = (*sf_iter)->object_id;
          pos.pos.x = est.pos_[0];
          pos.pos.y = est.pos_[1];
          pos.pos.z = est.pos_[2];
          pos.reliability = 0.5;
          pos.covariance[0] = 0.09;
          pos.covariance[1] = 0.0;
          pos.covariance[2] = 0.0;
          pos.covariance[3] = 0.0;
          pos.covariance[4] = 0.09;
          pos.covariance[5] = 0.0;
          pos.covariance[6] = 0.0;
          pos.covariance[7] = 0.0;
          pos.covariance[8] = 10000.0;
          pos.initialization = 0;
        
          publish("people_tracker_measurements", pos);        
        }
      }
    }

    // visualize all trackers
    channel.name = "rgb";
    channel.vals = weights;
    std_msgs::PointCloud  people_cloud; 
    people_cloud.chan.push_back(channel);
    people_cloud.header.frame_id = "odom";//scan_.header.frame_id;
    people_cloud.header.stamp = scan->header.stamp;
    people_cloud.pts  = filter_visualize;
    publish("kalman_filt_cloud", people_cloud);

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

