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

#include "point_cloud_utils/blob_finder.h"

#include <vector>
#include <list>

using namespace std_msgs ;
using namespace point_cloud_utils ;
using namespace std ;


BlobFinder::BlobFinder()
{
  min_points_ = 0 ;                                               // With these initial value, the blob tracker code will run, but it won't find any blobs
  max_blobs_ = 0 ;
  max_radius_ = 0 ;
}

void BlobFinder::findBlobs(const PointCloud& cloud, PointCloud& blobs) const
{
  const unsigned int N = cloud.get_pts_size() ;
  
  vector<list<unsigned int> > neighbors ;                         // Stores a list of neighbors for each point
  neighbors.resize(N) ;
  
  for (unsigned int i=0; i < N; i++)                              // Compute distances between all pairs of points, and add to list if it's a small distance
  {
    for (unsigned int j=0; j < i; j++)
    {
      if (dist(cloud.pts[i], cloud.pts[j]) < max_radius_)         // Is i close enough to j?
      {
        neighbors[i].push_back(j) ;                               // j is a neighbor of i
        neighbors[j].push_back(i) ;                               // i is a neighbor of j
      }
    }
  }
  

  blobs.header = cloud.header ;
  blobs.set_pts_size(max_blobs_) ;

  unsigned int num_blobs = 0 ;
  bool found_blob = true ;
  while(found_blob && num_blobs < max_blobs_)
  {
    found_blob = false ;
    
    int best_index = -1 ;
    unsigned int best_num_neighbors = min_points_ ;               // Only consider something the best if it beats our minimum point count
    
    // Find our current best point
    for (unsigned int i=0; i < N; i++)
    {
      if (neighbors[i].size() >= best_num_neighbors)              // Need to beat our previous best
      {
        best_index = i ;
        best_num_neighbors = neighbors[i].size() ;
      }
    }
    
    if (best_index >= 0)                                          //  Make sure we actually found a blob (since best_index would otherwise equal -1)
    {
      found_blob = true ;
      
      // Find centroids
      float sumX = 0 ;
      float sumY = 0 ;
      float sumZ = 0 ;
  
      int num_neighbors = neighbors[best_index].size() ;
  
      list<unsigned int>::iterator it ;
      for (it=neighbors[best_index].begin() ; it != neighbors[best_index].end(); it++ )
      {
        sumX += cloud.pts[*it].x ;
        sumY += cloud.pts[*it].y ;
        sumZ += cloud.pts[*it].z ;
      }
      sumX += cloud.pts[best_index].x ;                            // Add self to the average
      sumY += cloud.pts[best_index].y ;
      sumZ += cloud.pts[best_index].z ;
      
      
      Point32 cur_blob ;
      
      cur_blob.x = sumX / (num_neighbors+1) ;                      // Add 1 to account for self
      cur_blob.y = sumY / (num_neighbors+1) ;
      cur_blob.z = sumZ / (num_neighbors+1) ;
      
      blobs.pts[num_blobs] = cur_blob ;
      num_blobs++ ;
      
      // Prune off the points we've already used
      for (it=neighbors[best_index].begin() ; it != neighbors[best_index].end(); it++ )
      {
        neighbors[*it].clear() ;
      }
      neighbors[best_index].clear() ;
    }
  }
  blobs.set_pts_size(num_blobs) ;                                  // Trim blobs's size to actually match how many blobs we found
}

double BlobFinder::dist(const Point32& A, const Point32& B) const
{
  float dx = A.x - B.x ;
  float dy = A.y - B.y ;
  float dz = A.z - B.z ;
  return sqrt(dx*dx+dy*dy+dz*dz) ;
}

