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

#include "laser_scan_utils/laser_scan.h"
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
        cloud_out.set_chan_size(1);
        cloud_out.chan[0].name ="intensities";
        cloud_out.chan[0].set_vals_size(scan_in.get_intensities_size());
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
          if (scan_in.get_intensities_size() >= index) /// \todo optimize and catch length difference better
            cloud_out.chan[0].vals[count] = scan_in.intensities[index];
          count++;
        }
      }
      else { //Keep all points
        cloud_out.pts[count].x = outputMat[index];
        cloud_out.pts[count].y = outputMat[index + scan_in.get_ranges_size()];
        cloud_out.pts[count].z = 0.0;
        if (scan_in.get_intensities_size() >= index) /// \todo optimize and catch length difference better
          cloud_out.chan[0].vals[count] = scan_in.intensities[index];
        count++;
      }
        
    }

    //downsize if necessary
    cloud_out.set_pts_size(count);
    cloud_out.chan[0].set_vals_size(count);
 
  };
    void LaserProjection::projectLaser(const std_msgs::LaserScan& scan_in, std_msgs::PointCloudFloat32 & cloud_out, double range_cutoff, bool preservative)
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
        cloud_out.set_chan_size(1);
        cloud_out.chan[0].name ="intensities";
        cloud_out.chan[0].set_vals_size(scan_in.get_intensities_size());
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
          if (scan_in.get_intensities_size() >= index) /// \todo optimize and catch length difference better
            cloud_out.chan[0].vals[count] = scan_in.intensities[index];
          count++;
        }
      }
      else { //Keep all points
        cloud_out.pts[count].x = outputMat[index];
        cloud_out.pts[count].y = outputMat[index + scan_in.get_ranges_size()];
        cloud_out.pts[count].z = 0.0;
        if (scan_in.get_intensities_size() >= index) /// \todo optimize and catch length difference better
          cloud_out.chan[0].vals[count] = scan_in.intensities[index];
        count++;
      }
        
    }

    //downsize if necessary
    cloud_out.set_pts_size(count);
    if (scan_in.intensities_size > 0)
     cloud_out.chan[0].set_vals_size(count);
 
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

  LaserMedianFilter::LaserMedianFilter(unsigned int filter_length, unsigned int num_ranges, MedianMode_t mode):
    current_packet_num_(0),
    range_data_(filter_length,num_ranges),
    intensity_data_(filter_length,num_ranges),
    filter_length_(filter_length),
    num_ranges_(num_ranges),
    mode_(mode)
  {
  };

  bool LaserMedianFilter::addScan(const std_msgs::LaserScan& scan_in)
  {
    temp_scan_ = scan_in; //HACK to store all metadata 

    /** \todo check for length of intensities too */
    unsigned int iterations = std::min(scan_in.get_ranges_size(), num_ranges_);
    for (unsigned int index = 0; index < iterations; index ++)
      {
        range_data_(current_packet_num_+1, index+1)= (double) scan_in.ranges[index];
        intensity_data_(current_packet_num_+1, index+1)= (double) scan_in.intensities[index];
      }
    current_packet_num_++;
    if (current_packet_num_ == filter_length_)
      {
        current_packet_num_ = 0;
        return true;
      }
    if (mode_ == MEDIAN_TRAILING)
      return true;
    return false;
  }

  void LaserMedianFilter::getMedian(std_msgs::LaserScan& scan_result)
  {
    scan_result = temp_scan_; //Fill in latest scan data
    
    NEWMAT::ColumnVector dColumn;
    NEWMAT::ColumnVector iColumn;


    unsigned int iterations = std::min(scan_result.get_ranges_size(), num_ranges_);
    /** \todo Resize output cloud/check length */
    for (unsigned int index = 0; index < iterations; index ++)
      {
        dColumn = range_data_.Column(index+1);
        NEWMAT::SortAscending(dColumn);
        scan_result.ranges[index] = (float) dColumn((size_t)ceil(filter_length_/2));
        iColumn = intensity_data_.Column(index+1);
        NEWMAT::SortAscending(iColumn);
        scan_result.intensities[index] = (float) iColumn((size_t)ceil(filter_length_/2));
      }
    //reset to beginning
    current_packet_num_ = 0;
  }
} //laser_scan
