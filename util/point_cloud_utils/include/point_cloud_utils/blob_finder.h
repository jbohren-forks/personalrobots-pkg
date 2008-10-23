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


#include "std_msgs/PointCloud.h"

namespace point_cloud_utils
{
/**
 * Used to find blobs in a point cloud.  This is done by searching for clusters of points. Thus, searching for
 * blobs in a dense point-cloud (such as a full snapshot from the tilt_scan) doesn't really make sense, since there
 * will often be a ridiculous number of clusters.  It might make more sense to use this on pre-filtered data, such as
 * looking for blobs in clouds storing only high intensity readings.
 * 
 * Note that this is called a a blob-finder as opposed to a blob-tracker, since it doesn't save blob-state across successive calculations
 * 
 * \section Efficiency
 * The current implementation is hugely inefficient.  It computes the pairwise distance between all points in the cloud, O(N^2),
 * in order to determine who it is neighbors with.  There is much room to improve this.
 * 
 * \todo Use a grid or a KD Tree or some other data type to do the neighbor search more efficiently
 **/
class BlobFinder
{
  public :
    BlobFinder() ;
    
    /**
     * \brief Extract blobs from a cloud given a set of blob parameters.
     * \param cloud Input cloud used to extract blobs
     * \param blobs stores the list of centroids for the extracted blobs
     **/
    void findBlobs(const std_msgs::PointCloud& cloud, std_msgs::PointCloud& blobs) const ;

    /**
     * \brief Specify the minimum number of points that must exist within a blob for it to be considered a blob
     * \param min_pts What the new number of minimum points-per-blob should be
     **/
    void setMinPointsInBlob(const unsigned int min_points) { min_points_ = min_points ; }

    /**
     * \brief Specify the max number of blobs we want to extract from the scene
     * \param max_blobs What the new maximum number of blobs should be
     **/
    void setMaxBlobs(const unsigned int max_blobs) {max_blobs_ = max_blobs ; }

    /**
     * \brief Specify the max radius of the blob
     * \param max_blobs What the new maximum radius of the each blob should be
     **/
    void setMaxBlobRadius(const double max_radius) {max_radius_ = max_radius ; }
    
  private :
    unsigned int min_points_ ;          //!< Minimum number of points that must exist within max_radius_ for a blob for it to be considered a blob
    unsigned int max_blobs_ ;           //!< The max number of blobs we want to extract from the scene
    double max_radius_ ;                //!< The max radius of each blob
    
    /**
     * \brief computes distance between 2 points
     * \todo We should probably be using a more systematic way to compute distance between points, as opposed to reimplementing it
     **/
    double dist(const std_msgs::Point32& A, const std_msgs::Point32& B) const ;
} ;

}
