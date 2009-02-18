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



static const double no_observation_timeout_s = 0.5;
static const double max_track_jump_m         = 1.0; 
static const double max_meas_jump_m          = 1.0;
static const double leg_pair_separation_m    = 1.0;



#include "laser_processor.h"
#include "calc_leg_features.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "rosrecord/Player.h"

#include "robot_msgs/PositionMeasurement.h"
#include "laser_scan/LaserScan.h"
#include "roslib/Header.h"

#include "tf/transform_listener.h"
#include <tf/message_notifier.h>

#include "filter/tracker_kalman.h"
#include "filter/state_pos_vel.h"
#include "filter/rgb.h"

#include <algorithm>

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

class SavedTracker
{
public:
  static int nextid;
  string id_;
  string object_id;
  ros::Time time_;
  ros::Time meas_time_;
  vector<unsigned char>     color_;

  TransformListener& robot_state_;

  StatePosVel sys_sigma_;
  TrackerKalman filter_;

  Stamped<Point> prop_loc_;

  // one leg tracker
  SavedTracker(Stamped<Point> loc, TransformListener& tfl)
    : robot_state_(tfl),
      sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
      filter_("tracker_name",sys_sigma_)
  {
    char id[100];
    snprintf(id,100,"legtrack%d", nextid++);
    id_ = std::string(id);

    object_id = "";
    time_ = loc.stamp_;
    meas_time_ = loc.stamp_;

    Stamped<btTransform> pose( btTransform(Quaternion(), loc), loc.stamp_, id_, loc.frame_id_);
    robot_state_.setTransform(pose);

    StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
    filter_.initialize(loc, prior_sigma, time_.toSec());    

    color_.push_back((unsigned char)(rand()%255));
    color_.push_back((unsigned char)(rand()%255));
    color_.push_back((unsigned char)(rand()%255));
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
    Stamped<Point> orig_loc(Point(0,0,0),  meas_time_, id_);
    Stamped<Point> inter_loc;

    robot_state_.transformPoint("odom", orig_loc, inter_loc);
      
    inter_loc.stamp_ = time_;
      
    robot_state_.transformPoint("odom", inter_loc, prop_loc_);
    */
    
  }

  void update(Stamped<Point> loc)
  {
    Stamped<btTransform> pose( btTransform(Quaternion(), loc), loc.stamp_, id_, loc.frame_id_);
    robot_state_.setTransform(pose);

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

int SavedTracker::nextid = 0;




class MatchedTracker
{
public:
  SampleSet* measurement_;
  SavedTracker* closest_tracker_;
  float distance_;

  MatchedTracker(SampleSet* measurement, SavedTracker* closest_tracker, float distance)
  : measurement_(measurement)
  , closest_tracker_(closest_tracker)
  , distance_(distance)
  {}

  inline bool operator< (const MatchedTracker& b) const
  {
    return (distance_ <  b.distance_);
  }
};

int g_argc;
char** g_argv;

static const string fixed_frame = "odom";



// actual legdetector node
class LegDetector : public Node
{
public:
  TransformListener robot_state_;

  robot_msgs::PointCloud filt_cloud_;

  ScanMask mask_;

  int mask_count_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  list<SavedTracker*> saved_trackers_;
  boost::mutex saved_mutex_;

  int tracker_id_;

  MessageNotifier<robot_msgs::PositionMeasurement>*  people_notifier_;
  MessageNotifier<laser_scan::LaserScan>*  laser_notifier_;

  LegDetector() : 
    Node("laser_processor"), 
    robot_state_(*this), 
    mask_count_(0), 
    connected_thresh_(0.06), 
    feat_count_(0)
  {
    if (g_argc > 1) {
      forest.load(g_argv[1]);
      feat_count_ = forest.get_active_var_mask()->cols;
      printf("Loaded forest with %d trackers: %s\n", feat_count_, g_argv[1]);
    } else {
      printf("Please provide a trained random forests classifier as an input.\n");
      shutdown();
    }

    // advertise topics
    advertise<robot_msgs::PointCloud>("filt_cloud",10);
    advertise<robot_msgs::PointCloud>("kalman_filt_cloud",10);
    advertise<robot_msgs::PositionMeasurement>("people_tracker_measurements",1);

    // subscribe to topics
    people_notifier_ = new MessageNotifier<robot_msgs::PositionMeasurement>(&robot_state_, this,  
									    boost::bind(&LegDetector::peopleCallback, this, _1), 
									    "people_tracker_filter", fixed_frame, 10);
    laser_notifier_ = new MessageNotifier<laser_scan::LaserScan>(&robot_state_, this,  
							       boost::bind(&LegDetector::laserCallback, this, _1), 
							       "scan", fixed_frame, 10);

    tracker_id_ = 0;
  }


  ~LegDetector()
  {
    delete people_notifier_;
    delete laser_notifier_;
  }



  // Find the tracker that is closest to this person message
  // If a tracker was already assigned to a person, keep this assignment when the distance between them is not too large.
  void peopleCallback(const MessageNotifier<robot_msgs::PositionMeasurement>::MessagePtr& people_meas)
  {
    // If there are no legs, return.
    if (saved_trackers_.empty()) 
      return;

    Point pt;
    PointMsgToTF(people_meas->pos, pt);
    Stamped<Point> orig_loc(pt, people_meas->header.stamp, people_meas->header.frame_id);
    orig_loc[2] = 0.0; // Ignore the height of the person measurement.
    Stamped<Point> dest_loc(pt, people_meas->header.stamp, people_meas->header.frame_id);

    boost::mutex::scoped_lock lock(saved_mutex_);

    list<SavedTracker*>::iterator closest = saved_trackers_.end();
    list<SavedTracker*>::iterator closest1 = saved_trackers_.end();
    list<SavedTracker*>::iterator closest2 = saved_trackers_.end();
    float closest_dist = max_meas_jump_m;
    float closest_pair_dist = 2*max_meas_jump_m;
    float dist, dist1, dist2;

    list<SavedTracker*>::iterator it1 = saved_trackers_.begin();
    list<SavedTracker*>::iterator it2 = saved_trackers_.end();
    list<SavedTracker*>::iterator end = saved_trackers_.end();

    // If there's a pair of legs with the right label and within the max dist, return
    // If there's one leg with the right label and within the max dist, 
    //   find it a pair from the unlabeled legs.
    //   If no pairs, label just the one leg.
    // If there are no legs with the right label and within the max dist, 
    //   find a new unlabeled leg and assign the label.
    

    // Try to find a tracker with the same label and within the max distance of the person.
    cout << "Looking for two legs" << endl;
    for (; it1 != end; ++it1)
    {
      //      try {
      //robot_state_.transformPoint((*it1)->id_, people_meas->header.stamp,
      //                     orig_loc, fixed_frame, dest_loc);
      cout << "1. it1 " << (*it1)->id_ << endl;
      robot_state_.transformPoint((*it1)->id_, (*it1)->prop_loc_.stamp_, orig_loc, fixed_frame, dest_loc);
	//}
	//catch (tf::TransformException& ex) {
	//cout << ex.what() << endl;
	//continue;
    //}
      // Compute the distance between the person and this leg.
      dist = dest_loc.length();
      
      // If this leg belongs to the person...
      if ((*it1)->object_id == people_meas->object_id) 
      {
	// and their distance is close enough, assign it2 to this leg, and look for a second leg. Otherwise, remove the tracker's label.
	if (dist < max_meas_jump_m)
	{
	  if (it2 == end) 
	    it2 = it1;
	  else
	    break;
	}
	else
	  (*it1)->object_id = "";
      }
    }
    // If we found two legs with the right label and within the max distance, all is good, return.
    if (it1 != end && it2 != end) 
    {
      cout << "Found matching pair. The second distance was " << dist << endl;
      return;
    }
    cout << "Done looking for two legs" << endl;

    // If we found one leg, let's try to find a second leg that doesn't yet have a label and is within the max distance.
    cout << "Looking for one leg plus one new leg" << endl;
    float dist_between_legs, closest_dist_between_legs;
    if (it2 != end) 
    {
      closest_dist = max_meas_jump_m;
      closest = saved_trackers_.end();

      it1 = saved_trackers_.begin();
      for (; it1 != end; ++it1) 
      {
	if (it1 == it2)
	  continue;

	// Compute the distance between the person and this leg.
	//try {
	//robot_state_.transformPoint((*it1)->id_, people_meas->header.stamp,
	//	      orig_loc, fixed_frame, dest_loc);
	cout << "2. it1 " << (*it1)->id_ << endl;
	robot_state_.transformPoint((*it1)->id_, (*it1)->prop_loc_.stamp_, orig_loc, fixed_frame, dest_loc);
	  //}
	  //catch (tf::TransformException& ex) {
	  //cout << ex.what() << endl;
	  //continue;
	  //}
	dist = dest_loc.length();
	
	// Get the distance between the two legs
        printf("get the dist between the legs\n");
	//robot_state_.transformPoint((*it1)->id_, (*it2)->prop_loc_.stamp_, (*it2)->prop_loc_, fixed_frame, dest_loc);
	cout << "3. it1 " << (*it1)->id_ << " it2 " << (*it2)->prop_loc_.frame_id_ << endl;
	robot_state_.transformPoint((*it1)->id_, (*it1)->prop_loc_.stamp_, (*it2)->prop_loc_, fixed_frame, dest_loc); 
	printf("done get dist\n");
        dist_between_legs = dest_loc.length();

	// If this is the closest dist (and within range), and the legs are close together, and unlabeled, mark it.
	if ( (*it1)->object_id == "" && dist < closest_dist && dist_between_legs < leg_pair_separation_m )
	{
	  closest = it1;
	  closest_dist = dist;
	  closest_dist_between_legs = dist_between_legs;
	}
      }
      // If we found a close, unlabeled leg, set it's label.
      if (closest != end) 
      {
	cout << "Replaced one leg with a distance of " << closest_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
	(*closest)->object_id = people_meas->object_id;
      }
      else 
      {
	cout << "Returned one matched leg only" << endl;
      }
      
      // Regardless of whether we found a second leg, return.
      return;
    }

    cout << "Looking for a pair of new legs" << endl;
    // If we didn't find any legs, try to find two unlabeled legs that are close together and close to the tracker.
    it1 = saved_trackers_.begin();
    it2 = saved_trackers_.begin();
    closest = saved_trackers_.end();
    closest1 = saved_trackers_.end();
    closest2 = saved_trackers_.end();
    closest_dist = max_meas_jump_m;
    closest_pair_dist = 2*max_meas_jump_m;
    for (; it1 != end; ++it1) 
    {
      // Only look at trackers without ids.
      if ((*it1)->object_id != "")
	continue;

      // Get the distance between the leg and the person.
      //try {
      //	robot_state_.transformPoint((*it1)->id_, people_meas->header.stamp,
      //                    orig_loc, fixed_frame, dest_loc);

      cout << "4. it1 " << (*it1)->id_ << endl;
      robot_state_.transformPoint((*it1)->id_, (*it1)->prop_loc_.stamp_, orig_loc, fixed_frame, dest_loc);
	//}
	//catch (tf::TransformException& ex) {
	//cout << ex.what() << endl;
	//continue;
	//}
      dist1 = dest_loc.length();
      // Don't jump to a far-away tracker.
      if ( dist1 >= max_meas_jump_m ) 
	continue;

      // Keep the single closest leg around in case none of the pairs work out.
      if ( dist1 < closest_dist ) 
      {
	closest_dist = dist1;
	closest = it1;
      }

      // Find a second leg.
      it2 = it1;
      it2++;
      for (; it2 != end; ++it2) 
      {
	// Only look at trackers without ids.
	if ((*it2)->object_id != "") 
	  continue;

	// Get the distance between the leg and the person.
	//try {
	//robot_state_.transformPoint((*it2)->id_, people_meas->header.stamp, orig_loc, fixed_frame, dest_loc);
        cout << "5. it2 " << (*it2)->id_ <<endl;

	robot_state_.transformPoint((*it2)->id_, (*it2)->prop_loc_.stamp_, orig_loc, fixed_frame, dest_loc);  
	//}
	  //catch (tf::TransformException& ex) {
	  //cout << ex.what() << endl;
	  //continue;
	  //}
	dist2 = dest_loc.length();
	
	// Get the distance between the two legs
        printf("Get the dist between the legs, take 2\n");
	//robot_state_.transformPoint((*it1)->id_, (*it2)->prop_loc_.stamp_, (*it2)->prop_loc_, fixed_frame, dest_loc);
	cout << "6. it1 " << (*it1)->id_ << " it2 " << (*it2)->prop_loc_.frame_id_ << endl;
	robot_state_.transformPoint((*it1)->id_, (*it1)->prop_loc_.stamp_, (*it2)->prop_loc_, fixed_frame, dest_loc);
	printf("Done get dist, take 2\n");
        dist_between_legs = dest_loc.length();

	// Ensure that neither the second point, not the combination of points, is too far away.
	if ( dist2 < max_meas_jump_m && dist1+dist2 < closest_pair_dist && dist_between_legs < leg_pair_separation_m ) 
	{
	  closest_pair_dist = dist1+dist2;
	  closest1 = it1;
	  closest2 = it2;
	  closest_dist_between_legs = dist_between_legs;
	}
      }
    }
    // Found a pair of legs.
    if (closest1 != end && closest2 != end) 
    {
      (*closest1)->object_id = people_meas->object_id;
      (*closest2)->object_id = people_meas->object_id;
      cout << "Found a completely new pair with total distance " << closest_pair_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
      return;
    }

    cout << "Looking for just one leg" << endl;
    // No pair worked, try for just one leg.
    if (closest != end)
    {
      (*closest)->object_id = people_meas->object_id;
      cout << "Returned one new leg only" << endl;
      return;
    }

    cout << "Nothing matched" << endl;

    // Nothing matched.
 
    //////////////////////////////

//     // loop over trackers
//     list<SavedTracker*>::iterator it  = saved_trackers_.begin();
//     list<SavedTracker*>::iterator end = saved_trackers_.end();
//     for (; it != end; ++it )
//     {
//       // transform people position into local frame with the origin at the tracker position 
//       Stamped<Point> loc(pt, people_meas->header.stamp, people_meas->header.frame_id);
//       loc[2] = 0.0; // Ignore the height of the person measurement.
//       robot_state_.transformPoint((*it)->id_, people_meas->header.stamp,
//                             loc, fixed_frame, loc);
//       float dist = loc.length();
//       // distance between tracker and people position
//       if ( dist < closest_dist )
//       {
//         closest = it;
//         closest_dist = dist;
//       }

//       // If we're already tracking the object
//       if ((*it)->object_id == people_meas->object_id)
//       {
//         if (dist < max_meas_jump_m)
//           return; 
// 	// remove link between tracker and person
//         else
//           (*it)->object_id = "";
//       }
//     }

//     if (closest_dist < max_meas_jump_m)
//       (*closest)->object_id = people_meas->object_id;
  }





  void laserCallback(const MessageNotifier<laser_scan::LaserScan>::MessagePtr& scan)
  {
    filt_cloud_.pts.clear();
    filt_cloud_.chan.clear();
    filt_cloud_.chan.resize(1);
    filt_cloud_.chan[0].name = "rgb";


    // if no measurement matches to a tracker in the last no_observation_timeout  seconds: erase tracker
    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);
    list<SavedTracker*>::iterator sf_iter = saved_trackers_.begin();
    while (sf_iter != saved_trackers_.end())
    {
      if ((*sf_iter)->meas_time_ < purge)
      {
        delete (*sf_iter);
        saved_trackers_.erase(sf_iter++);
      }
      else
        ++sf_iter;
    }


    // System update of trackers, and copy updated ones in propagate list
    list<SavedTracker*> propagated;
    for (list<SavedTracker*>::iterator sf_iter = saved_trackers_.begin();
         sf_iter != saved_trackers_.end();
         sf_iter++)
    {
      (*sf_iter)->propagate(scan->header.stamp);
      propagated.push_back(*sf_iter);
    }


    // Detection step: build up the set of "measurement" clusters
    ScanProcessor processor(*scan, mask_);
    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);
    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);
    list<SampleSet*> measurements;
    for (list<SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      vector<float> f = calcLegFeatures(*i, *scan);

      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)(f[k]);

      if (forest.predict( tmp_mat ) > 0)
      {
        measurements.push_back(*i);
      }
    }


    // For each measurement, find the closest tracker (within threshold) and add to the match list
    // If no tracker is found, start a new one
    multiset<MatchedTracker> matches;
    for (list<SampleSet*>::iterator measurement_iter = measurements.begin();
         measurement_iter != measurements.end(); 
	 measurement_iter++)
      {
	// transform measurement point to fixed_frame
	Stamped<Point> loc((*measurement_iter)->center(), scan->header.stamp, scan->header.frame_id);
	robot_state_.transformPoint(fixed_frame, loc, loc);
	
	list<SavedTracker*>::iterator closest = propagated.end();
	float closest_dist = max_track_jump_m;
	
	for (list<SavedTracker*>::iterator pf_iter = propagated.begin();
	     pf_iter != propagated.end();
	     pf_iter++)
	  {
	    // find the closest distance between measurement and trackers
	    float dist = loc.distance((*pf_iter)->prop_loc_);
	    if ( dist < closest_dist )
	      {
		closest = pf_iter;
		closest_dist = dist;
	      }
	  }
	// Nothing close to it, start a new track
	if (closest == propagated.end()) 
	  {
	    list<SavedTracker*>::iterator new_saved = saved_trackers_.insert(saved_trackers_.end(), new SavedTracker(loc, robot_state_));
	    (*measurement_iter)->appendToCloud(filt_cloud_, (*new_saved)->color_[0], (*new_saved)->color_[1], (*new_saved)->color_[2]);
	  }
	// Add the measurement, the tracker and the distance to a match list
	else
	  matches.insert(MatchedTracker(*measurement_iter,*closest,closest_dist));
      }
    



    // loop through _sorted_ matches list
    // find the match with the shortest distance for each tracker
    while (matches.size() > 0)
    {
      multiset<MatchedTracker>::iterator matched_iter = matches.begin();
      bool found = false;
      list<SavedTracker*>::iterator pf_iter = propagated.begin();
      while (pf_iter != propagated.end())
      {
	// update the tracker with this measurement
        if (matched_iter->closest_tracker_ == *pf_iter)
        {
	  // Transform measurement to fixed frame
          Stamped<Point> loc(matched_iter->measurement_->center(), scan->header.stamp, scan->header.frame_id);
          robot_state_.transformPoint(fixed_frame, loc, loc);          

	  // Update the tracker with the measurement location
          matched_iter->closest_tracker_->update(loc);
          
	  // visualize
          matched_iter->measurement_->appendToCloud(filt_cloud_, matched_iter->closest_tracker_->color_[0], 
						  matched_iter->closest_tracker_->color_[1], matched_iter->closest_tracker_->color_[2]);
	  // remove this match and 
          matches.erase(matched_iter);
          propagated.erase(pf_iter++);
          found = true;
          break;
        }
	// still looking for the tracker to update
        else
        {
          pf_iter++;
        }
      }

      // didn't find tracker to update, because it was deleted above
      // try to assign the measurement to another tracker
      if (!found)
      {
	// transform matched point to fixed frame
        Stamped<Point> loc(matched_iter->measurement_->center(), scan->header.stamp, scan->header.frame_id);
        robot_state_.transformPoint(fixed_frame, loc, loc);

        list<SavedTracker*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;
      
        for (list<SavedTracker*>::iterator remain_iter = propagated.begin();
             remain_iter != propagated.end();
             remain_iter++)
        {
          float dist = loc.distance((*remain_iter)->prop_loc_);
          if ( dist < closest_dist )
          {
            closest = remain_iter;
            closest_dist = dist;
          }
        }

	// no tracker is within a threshold of this measurement
	// so create a new tracker for this measurement
        if (closest == propagated.end())
        {
          list<SavedTracker*>::iterator new_saved = saved_trackers_.insert(saved_trackers_.end(), new SavedTracker(loc, robot_state_));

          matched_iter->measurement_->appendToCloud(filt_cloud_, (*new_saved)->color_[0], (*new_saved)->color_[1], (*new_saved)->color_[2]);
          matches.erase(matched_iter);
        }
        else
        {
          matches.insert(MatchedTracker(matched_iter->measurement_,*closest,closest_dist));
          matches.erase(matched_iter);
        }
      }
    }

    // visualization
    filt_cloud_.header.stamp = scan->header.stamp;
    filt_cloud_.header.frame_id = scan->header.frame_id;
    publish("filt_cloud", filt_cloud_);
    cvReleaseMat(&tmp_mat); tmp_mat = 0;

    vector<robot_msgs::Point32> filter_visualize(saved_trackers_.size());
    vector<float> weights(saved_trackers_.size());
    robot_msgs::ChannelFloat32 channel;
    int i = 0;

    for (list<SavedTracker*>::iterator sf_iter = saved_trackers_.begin();
         sf_iter != saved_trackers_.end();
         sf_iter++,i++)
    {
      // publish filter result
      StatePosVel est;
      (*sf_iter)->filter_.getEstimate(est);
      filter_visualize[i].x = est.pos_[0];
      filter_visualize[i].y = est.pos_[1];
      filter_visualize[i].z = est.pos_[2];

      // reliability of tracker
      double reliability = fmin(1, fmax(0.1, est.vel_.length() / 0.5 ));

      // color depends on reliability
      weights[i] = *(float*)&(rgb[min(998, max(1, (int)(reliability * 999)))]);

      // publish trackers that are assigned to a person
      if ((*sf_iter)->object_id != "")
      {         
	robot_msgs::PositionMeasurement pos;

	pos.header.stamp = (*sf_iter)->time_;
	pos.header.frame_id = "odom";
	pos.name = "leg_detector";
	pos.object_id = (*sf_iter)->object_id;
	pos.pos.x = est.pos_[0];
	pos.pos.y = est.pos_[1];
	pos.pos.z = est.pos_[2];

        // define covariance based on reliability
	pos.covariance[0] = pow(0.03 / reliability,2);
	pos.covariance[1] = 0.0;
	pos.covariance[2] = 0.0;
	pos.covariance[3] = 0.0;
	pos.covariance[4] = pow(0.03 / reliability,2);
	pos.covariance[5] = 0.0;
	pos.covariance[6] = 0.0;
	pos.covariance[7] = 0.0;
	pos.covariance[8] = 10000.0;
	pos.initialization = 0;

        publish("people_tracker_measurements", pos);        
      }
    }


    // visualize all trackers
    channel.name = "rgb";
    channel.vals = weights;
    robot_msgs::PointCloud  people_cloud; 
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
  g_argc = argc;
  g_argv = argv;
  LegDetector ld;
  ld.spin();
  
  return 0;
}

