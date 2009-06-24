/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: scan_shadows_filter.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/*
\author Radu Bogdan Rusu <rusu@cs.tum.edu>


 */

#include <ros/node.h>
#include <robot_msgs/PointCloud.h>
#include <laser_scan/LaserScan.h>

#include <float.h>

// Laser projection
#include <laser_scan/laser_scan.h>

// TF
#include <tf/transform_listener.h>
#include "tf/message_notifier.h"

#include "laser_scan/point_cloud_footprint_filter.h"
#include "filters/filter_chain.h"

using namespace robot_msgs;

/** @b ScanShadowsFilter is a simple node that filters shadow points in a laser scan line and publishes the results in a cloud.
 */
class ScanShadowsFilter
{
  public:

    // ROS related
    laser_scan::LaserProjection projector_; // Used to project laser scans

    double laser_max_range_;           // Used in laser scan projection
    double min_angle_, max_angle_;          // Filter angle threshold
    int window_;
    
    bool high_fidelity_;                    // High fidelity (interpolating time across scan)
    bool preservative_;                     //keep max range values?
    std::string target_frame_;                   // Target frame for high fidelity result
    std::string scan_topic_, cloud_topic_;
    
    // TF
    tf::TransformListener* tf_;
    tf::MessageNotifier<laser_scan::LaserScan>* notifier_;
    filters::FilterChain<robot_msgs::PointCloud> filter_chain_;

    ////////////////////////////////////////////////////////////////////////////////
    ScanShadowsFilter () : laser_max_range_ (DBL_MAX), notifier_(NULL)
    {
      tf_ = new tf::TransformListener() ;

      ros::Node::instance()->param ("~filter_min_angle", min_angle_, 10.0);
      ros::Node::instance()->param ("~filter_max_angle", max_angle_, 170.0);
      ros::Node::instance()->param ("~filter_window", window_, 2);

      ros::Node::instance()->param ("~high_fidelity", high_fidelity_, false);
      ros::Node::instance()->param ("~target_frame", target_frame_, std::string ("base_link"));
      ros::Node::instance()->param ("~preservative", preservative_, false);
      ros::Node::instance()->param ("~scan_topic", scan_topic_, std::string("tilt_scan"));
      ros::Node::instance()->param ("~cloud_topic", cloud_topic_, std::string("tilt_laser_cloud_filtered"));
      ros::Node::instance()->param ("~laser_max_range", laser_max_range_, DBL_MAX);

      notifier_ = new tf::MessageNotifier<laser_scan::LaserScan>(tf_, ros::Node::instance(), 
          boost::bind(&ScanShadowsFilter::scanCallback, this, _1), scan_topic_, "base_link", 50);
      notifier_->setTolerance(ros::Duration(0.03));

      ros::Node::instance()->advertise<PointCloud> (cloud_topic_, 10);

      std::string filter_xml;
      ros::Node::instance()->param("~filters", filter_xml, std::string("<filters><!--Filter Parameter Not Set--></filters>"));
      ROS_INFO("Got parameter'~filters' as: %s\n", filter_xml.c_str());
      filter_chain_.configureFromXMLString(1, filter_xml);
    }

    ////////////////////////////////////////////////////////////////////////////////
    virtual ~ScanShadowsFilter () { if(notifier_) delete notifier_;}

    ////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Computes the angle between the two lines created from 2 points and the
     * viewpoint. Returns the angle (in degrees).
     * \param px X coordinate for the first point
     * \param py Y coordinate for the first point
     * \param pz Z coordinate for the first point
     * \param qx X coordinate for the second point
     * \param qy Y coordinate for the second point
     * \param qz Z coordinate for the second point
     */
    inline double
      getAngleWithViewpoint (float px, float py, float pz, float qx, float qy, float qz)
    {
      double dir_a[3], dir_b[3];
      dir_a[0] =    - px; dir_a[1] =    - py; dir_a[2] =    - pz;   // Assume viewpoint is 0,0,0
      dir_b[0] = qx - px; dir_b[1] = qy - py; dir_b[2] = qz - pz;

      // sqrt (sqr (x) + sqr (y) + sqr (z))
      double norm_a = sqrt (dir_a[0]*dir_a[0] + dir_a[1]*dir_a[1] + dir_a[2]*dir_a[2]);
      // Check for bogus 0,0,0 points
      if (norm_a == 0) return (0);
      double norm_b = sqrt (dir_b[0]*dir_b[0] + dir_b[1]*dir_b[1] + dir_b[2]*dir_b[2]);
      if (norm_b == 0) return (0);
      // dot_product (x, y)
      double dot_pr = dir_a[0]*dir_b[0] + dir_a[1]*dir_b[1] + dir_a[2]*dir_b[2];
      if (dot_pr != dot_pr)     // Check for NaNs
        return (0);

      return ( acos (dot_pr / (norm_a * norm_b) ) * 180.0 / M_PI);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Given a PointCloud representing a single laser scan (usually obtained
     * after LaserProjection's projectLaser(), and the index of the channel
     * representing the true measurement "index", create a complete PointCloud
     * representation which replaces the invalid measurements with 0 values.
     * \param c_idx the channel index for the "index"
     * \param cloud_in the input PointCloud message
     * \param cloud_out the output PointCloud message
     */
    void
      constructCompleteLaserScanCloud (int c_idx, PointCloud cloud_in, PointCloud &cloud_out)
    {
      // Use the index to revert to a full laser scan cloud (inefficient locally, but efficient globally)
      int idx = 0;
      for (unsigned int i = 0; i < cloud_out.pts.size (); i++)
      {
        unsigned int j = (int)cloud_in.chan[c_idx].vals[idx];  // Find out the true index value
        if (i == j)
        {
          // Copy relevant data
          cloud_out.pts[i].x = cloud_in.pts[idx].x;
          cloud_out.pts[i].y = cloud_in.pts[idx].y;
          cloud_out.pts[i].z = cloud_in.pts[idx].z;
          for (unsigned int d = 0; d < cloud_out.get_chan_size (); d++)
            cloud_out.chan[d].vals[i] = cloud_in.chan[d].vals[idx];

          idx++;                                        // Assume chan['index'] is sorted (which should be true)
          if (idx >= (int)cloud_in.chan[c_idx].vals.size ()) idx = cloud_in.chan[c_idx].vals.size () - 1;
        }
        else
        {
          // Bogus XYZ entry. No need to copy channels.
          cloud_out.pts[i].x = cloud_out.pts[i].y = cloud_out.pts[i].z = 0.0;
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
     * and window. {min,max}_angle specify the allowed angle interval (in degrees)
     * between the created lines (see getAngleWithViewPoint). Window specifies how many
     * consecutive measurements to take into account for one point.
     * \param cloud_in the input PointCloud message
     * \param cloud_out the output PointCloud message
     */
    void
      filterShadowPoints (PointCloud cloud_in, PointCloud &cloud_out)
    {
      // For each point in the current line scan
      int n_pts_filtered = 0;
      for (unsigned int i = 0; i < cloud_in.pts.size (); i++)
      {
        bool valid_point = true;
        for (int y = -window_; y < window_ + 1; y++)
        {
          int j = i + y;
          if ( j < 0 || j >= (int)cloud_in.pts.size () || (int)i == j ) // Out of scan bounds or itself
            continue;

          double angle = getAngleWithViewpoint (cloud_in.pts[i].x, cloud_in.pts[i].y, cloud_in.pts[i].z,
                                                cloud_in.pts[j].x, cloud_in.pts[j].y, cloud_in.pts[j].z);
          if (angle < min_angle_ || angle > max_angle_)
            valid_point = false;
        }

        // If point found as 'ok', copy the relevant data
        if (valid_point)
        {
          cloud_out.pts[n_pts_filtered].x = cloud_in.pts[i].x;
          cloud_out.pts[n_pts_filtered].y = cloud_in.pts[i].y;
          cloud_out.pts[n_pts_filtered].z = cloud_in.pts[i].z;

          for (unsigned int d = 0; d < cloud_out.get_chan_size (); d++)
            cloud_out.chan[d].vals[n_pts_filtered] = cloud_in.chan[d].vals[i];

          n_pts_filtered++;
        }
      }

      // Resize output vectors
      cloud_out.pts.resize (n_pts_filtered);
      for (unsigned int d = 0; d < cloud_out.get_chan_size (); d++)
        cloud_out.chan[d].vals.resize (n_pts_filtered);
    }

    ////////////////////////////////////////////////////////////////////////////////
    void
      scanCallback (const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& msg_in)
    {
      laser_scan::LaserScan& scan_msg = *msg_in;
      // Project laser into point cloud
      PointCloud scan_cloud;
      int n_scan = scan_msg.ranges.size ();      // Save the number of measurements

      //\TODO CLEAN UP HACK 
      // This is a trial at correcting for incident angles.  It makes many assumptions that do not generalise
      for (unsigned int i = 0; i < scan_msg.ranges.size(); i++)
      {
        double angle = scan_msg.angle_min + i * scan_msg.angle_increment;
        scan_msg.ranges[i] = scan_msg.ranges[i] + 0.03 * exp(-fabs(sin(angle)));

      };


      // Transform into a PointCloud message
      int mask = laser_scan::MASK_INTENSITY | laser_scan::MASK_DISTANCE | laser_scan::MASK_INDEX | laser_scan::MASK_TIMESTAMP;
      
      if (high_fidelity_)
      {
        try
        {
          projector_.transformLaserScanToPointCloud (target_frame_, scan_cloud, scan_msg, *tf_, mask);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN ("High fidelity enabled, but TF returned a transform exception to frame %s: %s", target_frame_.c_str (), ex.what ());
          projector_.projectLaser (scan_msg, scan_cloud, laser_max_range_, preservative_, mask);//, true);
        }
      }
      else
      {
        projector_.projectLaser (scan_msg, scan_cloud, laser_max_range_, preservative_, mask);//, true);
      }
      

      /// ---[ Perhaps unnecessary, but find out which channel contains the index
      int c_idx = -1;
      for (unsigned int d = 0; d < scan_cloud.get_chan_size (); d++)
      {
        if (scan_cloud.chan[d].name == "index")
        {
          c_idx = d;
          break;
        }
      }
      if (c_idx == -1 || scan_cloud.chan[c_idx].vals.size () == 0) return;
      /// ]--

      // Prepare the storage for the temporary array ([] and resize are faster than push_back)
      PointCloud scan_full_cloud (scan_cloud);
      scan_full_cloud.pts.resize (n_scan);
      for (unsigned int d = 0; d < scan_cloud.get_chan_size (); d++)
        scan_full_cloud.chan[d].vals.resize (n_scan);

      // Prepare data storage for the output array ([] and resize are faster than push_back)
      PointCloud filtered_cloud (scan_cloud);
      filtered_cloud.pts.resize (n_scan);
      for (unsigned int d = 0; d < scan_cloud.get_chan_size (); d++)
        filtered_cloud.chan[d].vals.resize  (n_scan);

      // Construct a complete laser cloud resembling the original LaserScan (0..LASER_MAX measurements)
      constructCompleteLaserScanCloud (c_idx, scan_cloud, scan_full_cloud);

      // Filter points
      filterShadowPoints (scan_full_cloud, filtered_cloud);

      PointCloud clear_footprint_cloud;
      filter_chain_.update (filtered_cloud, clear_footprint_cloud);

      // Set timestamp/frameid and publish
      ros::Node::instance()->publish (cloud_topic_, clear_footprint_cloud);
    }

} ;

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);
  ros::Node n("scan_shadows_filter");
  ScanShadowsFilter f;
  n.spin ();

  

  return (0);
}
/* ]--- */
