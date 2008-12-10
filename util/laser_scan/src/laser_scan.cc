/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#include "laser_scan/laser_scan.h"
#include <algorithm>

namespace laser_scan{

  
  void LaserProjection::projectLaser(const std_msgs::LaserScan& scan_in, std_msgs::PointCloud & cloud_out, double range_cutoff, bool preservative)
  {
    NEWMAT::Matrix ranges(2, scan_in.get_ranges_size());
    double * matPointer = ranges.Store();
    // Fill the ranges matrix
    for (unsigned int index = 0; index < scan_in.get_ranges_size(); index++)
      {
        matPointer[index] = (double) scan_in.ranges[index];
        matPointer[index+scan_in.get_ranges_size()] = (double) scan_in.ranges[index];
      }
    

    //Do the projection
    NEWMAT::Matrix output = NEWMAT::SP(ranges, getUnitVectors(scan_in.angle_min, scan_in.angle_max, scan_in.angle_increment));
    

    //Stuff the output cloud
    cloud_out.header = scan_in.header;
    cloud_out.set_pts_size(scan_in.get_ranges_size());
    if (scan_in.get_intensities_size() > 0)
      {
        cloud_out.set_chan_size(2);
        cloud_out.chan[0].name ="intensities";
        cloud_out.chan[0].set_vals_size(scan_in.get_intensities_size());

        cloud_out.chan[1].name = "index";
        cloud_out.chan[1].set_vals_size(scan_in.get_ranges_size());
      }

    double* outputMat = output.Store();

    if (range_cutoff < 0)
      range_cutoff = scan_in.range_max;
    else
      range_cutoff = std::min(range_cutoff, (double)scan_in.range_max); 
    
    unsigned int count = 0;
    for (unsigned int index = 0; index< scan_in.get_ranges_size(); index++)
    {
      if (!preservative){ //Default behaviour will throw out invalid data
        if ((matPointer[index] < range_cutoff) &&
            (matPointer[index] > scan_in.range_min)) //only valid
        {
          cloud_out.pts[count].x = outputMat[index];
          cloud_out.pts[count].y = outputMat[index + scan_in.get_ranges_size()];
          cloud_out.pts[count].z = 0.0;

          //write index to point cloud
          cloud_out.chan[1].vals[count] = index;

          if (scan_in.get_intensities_size() >= index){ /// \todo optimize and catch length difference better
            cloud_out.chan[0].vals[count] = scan_in.intensities[index];
          }
          count++;
        }
      }
      else { //Keep all points
        cloud_out.pts[count].x = outputMat[index];
        cloud_out.pts[count].y = outputMat[index + scan_in.get_ranges_size()];
        cloud_out.pts[count].z = 0.0;

        //write index to point cloud
        cloud_out.chan[1].vals[count] = index;

        if (scan_in.get_intensities_size() >= index){ /// \todo optimize and catch length difference better
          cloud_out.chan[0].vals[count] = scan_in.intensities[index];
        }

        count++;
      }
        
    }

    //downsize if necessary
    cloud_out.set_pts_size(count);
    cloud_out.chan[0].set_vals_size(count);
    cloud_out.chan[1].set_vals_size(count);
 
  };

  NEWMAT::Matrix& LaserProjection::getUnitVectors(float angle_min, float angle_max, float angle_increment)
  {
    //construct string for lookup in the map
    std::stringstream anglestring;
    anglestring <<angle_min<<","<<angle_max<<","<<angle_increment;
    std::map<string, NEWMAT::Matrix*>::iterator it;
    it = unit_vector_map_.find(anglestring.str());
    //check the map for presense
    if (it != unit_vector_map_.end())
      return *((*it).second);     //if present return
    //else calculate
    unsigned int length = (unsigned int) round((angle_max - angle_min)/angle_increment) + 1; ///\todo Codify how this parameter will be calculated in all cases
    NEWMAT::Matrix * tempPtr = new NEWMAT::Matrix(2,length);
    for (unsigned int index = 0;index < length; index++)
      {
        (*tempPtr)(1,index+1) = cos(angle_min + (double) index * angle_increment);
        (*tempPtr)(2,index+1) = sin(angle_min + (double) index * angle_increment);
      }
    //store 
    unit_vector_map_[anglestring.str()] = tempPtr;
    //and return
    return *tempPtr;
  };


  LaserProjection::~LaserProjection()
  {
    std::map<string, NEWMAT::Matrix*>::iterator it;
    it = unit_vector_map_.begin();
    while (it != unit_vector_map_.end())
      {
        delete (*it).second;
        it++;
      }
  };

void LaserProjection::transformLaserScanToPointCloud(const std::string & target_frame, std_msgs::PointCloud & cloudOut, const std_msgs::LaserScan & scanIn,
                                                       tf::Transformer& tf)
{
  cloudOut.header = scanIn.header;
  cloudOut.header.frame_id = target_frame;
  cloudOut.set_pts_size(scanIn.get_ranges_size());
  if (scanIn.get_intensities_size() > 0)
  {
    cloudOut.set_chan_size(2);
    cloudOut.chan[0].name ="intensities";
    cloudOut.chan[0].set_vals_size(scanIn.get_intensities_size());

    cloudOut.chan[1].name ="index";
    cloudOut.chan[1].set_vals_size(scanIn.get_ranges_size());
  }

  tf::Stamped<tf::Point> pointIn;
  tf::Stamped<tf::Point> pointOut;

  pointIn.frame_id_ = scanIn.header.frame_id;

  ///\todo this can be optimized
  std_msgs::PointCloud intermediate; //optimize out

  projectLaser(scanIn, intermediate, -1.0, true);

  // Extract transforms for the beginning and end of the laser scan
  ros::Time start_time = scanIn.header.stamp ;
  ros::Time end_time   = scanIn.header.stamp + ros::Duration().fromSec(scanIn.get_ranges_size()*scanIn.time_increment) ;

  tf::Stamped<tf::Transform> start_transform ;
  tf::Stamped<tf::Transform> end_transform ;
  tf::Stamped<tf::Transform> cur_transform ;

  tf.lookupTransform(target_frame, scanIn.header.frame_id, start_time, start_transform) ;
  tf.lookupTransform(target_frame, scanIn.header.frame_id, end_time, end_transform) ;


  unsigned int count = 0;  
  for (unsigned int i = 0; i < scanIn.get_ranges_size(); i++)
  {
    if (scanIn.ranges[i] < scanIn.range_max 
        && scanIn.ranges[i] > scanIn.range_min) //only when valid
    {
      // Looking up transforms in tree is too expensive. Need more optimized way
      /*
         pointIn = tf::Stamped<tf::Point>(btVector3(intermediate.pts[i].x, intermediate.pts[i].y, intermediate.pts[i].z), 
         ros::Time(scanIn.header.stamp.to_ull() + (uint64_t) (scanIn.time_increment * 1000000000)),
         pointIn.frame_id_ = scanIn.header.frame_id);///\todo optimize to no copy
         transformPoint(target_frame, pointIn, pointOut);
         */

      // Instead, assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
      btScalar ratio = i / ( (double) scanIn.get_ranges_size() - 1.0) ;

      //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)

      //Interpolate translation
      btVector3 v ;
      v.setInterpolate3(start_transform.getOrigin(), end_transform.getOrigin(), ratio) ;
      cur_transform.setOrigin(v) ;

      //Interpolate rotation
      btQuaternion q1, q2 ;
      start_transform.getBasis().getRotation(q1) ;
      end_transform.getBasis().getRotation(q2) ;

      // Compute the slerp-ed rotation
      cur_transform.setRotation( slerp( q1, q2 , ratio) ) ;

      // Apply the transform to the current point
      btVector3 pointIn(intermediate.pts[i].x, intermediate.pts[i].y, intermediate.pts[i].z) ;
      btVector3 pointOut = cur_transform * pointIn ;

      // Copy transformed point into cloud
      cloudOut.pts[count].x  = pointOut.x();
      cloudOut.pts[count].y  = pointOut.y();
      cloudOut.pts[count].z  = pointOut.z();

      //Copy index over from projected point cloud
      cloudOut.chan[1].vals[count] = intermediate.chan[1].vals[i];


      if (scanIn.get_intensities_size() >= i) /// \todo optimize and catch length difference better
        cloudOut.chan[0].vals[count] = scanIn.intensities[i];
      count++;
    }

  }
  //downsize if necessary
  cloudOut.set_pts_size(count);
  cloudOut.chan[0].set_vals_size(count);
  cloudOut.chan[1].set_vals_size(count);
}


} //laser_scan
