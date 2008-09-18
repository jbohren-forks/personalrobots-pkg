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
 *
 * Authors: E. Gil Jones, Conor McGann
 */

/*

  Map format is

(x,y)

       width (x)
  |--------------|
 -
 | (0,0) (1,0) ... (width-1,0)
h| (0,1)
e|   .
i|   .
g|   .
h|   .
t| (0, height-1) ... (width-1, height-1)
 -
(y)

*/

#include "costmap_2d/costmap_2d.h"

CostMap2D::CostMap2D(size_t width, size_t height, const unsigned char* data, 
		     double resolution, double window_length,  unsigned char threshold, double maxZ)
: width_(width), 
   height_(height),  
   resolution_(resolution), 
   tickLength_(window_length/WATCHDOG_LIMIT),
   threshold_(threshold),
   maxZ_(maxZ),
   staticData_(NULL), fullData_(NULL), obsWatchDog_(NULL), lastTimeStamp_(0.0)
{
  staticData_ = new unsigned char[width_*height_];
  fullData_ = new unsigned char[width_*height_];
  obsWatchDog_ = new TICK[width_*height_];
  memcpy(staticData_, data, width_*height_);
  memcpy(fullData_, staticData_, width_*height_);
  memset(obsWatchDog_, 0, width_*height_);

  // Iterate over the map and get the occupied cells
  for (unsigned int i=0;i<width_;i++){
    for (unsigned int j=0;j<height_;j++){
      size_t ind = getMapIndexFromCellCoords(i, j);
      if(staticData_[ind] >= threshold_)
	permanentlyOccupiedCells_.push_back(ind);
    }
  }
}

CostMap2D::~CostMap2D() {
  if(staticData_ != NULL) delete[] staticData_;
  if(fullData_ != NULL) delete[] fullData_;
  if(obsWatchDog_ != NULL) delete[] obsWatchDog_;
}

void CostMap2D::updateDynamicObstacles(double ts,
				       const std_msgs::PointCloudFloat32& cloud,
				       std::vector<unsigned int>& newObstacles, 
				       std::vector<unsigned int>& deletedObstacles)
{
  newObstacles.clear();

  for(size_t i = 0; i < cloud.get_pts_size(); i++) {
    if(cloud.pts[i].z > maxZ_)
      continue;

    unsigned int ind = getMapIndexFromWorldCoords(cloud.pts[i].x, cloud.pts[i].y);

    // If the cell is not occupied, then we have a new obstacle to report
    if(obsWatchDog_[ind] == 0 && staticData_[ind] < threshold_){
      newObstacles.push_back(ind);
      dynamicObstacles_.push_back(ind);
      fullData_[ind] = threshold_;
    }

    // Now pet the watchdog
    obsWatchDog_[ind] = WATCHDOG_LIMIT;
  }

  // We always process deletions too
  removeStaleObstacles(ts, deletedObstacles);
}

/**
 * This algorithm uses the concept of a watchdog timer to decide when an obstacle can be removed.
 */
void CostMap2D::removeStaleObstacles(double ts, std::vector<unsigned int>& deletedObstacles){
  deletedObstacles.clear();

  // Calculate elapsed time in terms of ticks
  TICK elapsedTime = getElapsedTime(ts);

  // Iterate over the set of dyanmic obstacles
  std::list<unsigned int>::iterator it = dynamicObstacles_.begin();

  while(it != dynamicObstacles_.end()){
    unsigned int ind = *it;
    TICK timeLeft = obsWatchDog_[ind];

    // Case 1: The watchdog has just been reset. Here we decrement the watchdog by 1 and move on
    if(timeLeft == WATCHDOG_LIMIT){
      obsWatchDog_[ind]--;
      ++it;
      continue;
    }

    // Case 2: The watchdog has timed out. Here we remove the obstacle, zero the watchdog timer, and move on.
    if(timeLeft <= elapsedTime){
      it = dynamicObstacles_.erase(it);
      obsWatchDog_[ind] = 0;
      fullData_[ind] = staticData_[ind];
      deletedObstacles.push_back(ind);
      continue;
    }

    // Otherwise: Just have to decrement the watchdog and move on
    obsWatchDog_[ind] = timeLeft - elapsedTime;
    ++it;
  }
}

const unsigned char* CostMap2D::getMap() const {
  return fullData_;
}

void CostMap2D::getOccupiedCellDataIndexList(std::vector<unsigned int>& results) const {
  results = permanentlyOccupiedCells_;

  for(std::list<unsigned int>::const_iterator it = dynamicObstacles_.begin(); it != dynamicObstacles_.end(); ++it)
    results.push_back(*it);
}

size_t CostMap2D::getMapIndexFromCellCoords(unsigned int x, unsigned int y) const 
{
  return(x+y*width_);
}

size_t CostMap2D::getMapIndexFromWorldCoords(double wx, double wy) const {
  size_t mx, my;
  convertFromWorldCoordToIndexes(wx, wy, mx, my);
  return getMapIndexFromCellCoords(mx, my);
}

void CostMap2D::convertFromWorldCoordToIndexes(double wx, double wy,
					     size_t& mx, size_t& my) const {
  
  if(wx < 0 || wy < 0) {
    //std::cerr << "CostMap2D::convertFromWorldCoordToIndex problem with " << wx << " or " << wy << std::endl;
    mx = 0;
    my = 0;
    return;
  }

  //not much for now
  mx = (int) (wx/resolution_);
  my = (int) (wy/resolution_);

  if(mx > width_) {
    std::cout << "CostMap2D::convertFromWorldCoordToIndex converted x " << wx << " greater than width " << width_ << std::endl;
    mx = 0;
    return;
  } 
  if(my > height_) {
    std::cout << "CostMap2D::convertFromWorldCoordToIndex converted y " << wy << " greater than height " << height_ << std::endl;
    my = 0;
    return;
  }
}

void CostMap2D::convertFromIndexesToWorldCoord(size_t mx, size_t my,
                                               double& wx, double& wy) const {
  wx = (mx*1.0)*resolution_;
  wy = (my*1.0)*resolution_;
}

void CostMap2D::convertFromMapIndexToXY(unsigned int ind,
                                        unsigned int& x,
                                        unsigned int& y) const {
  //depending on implicit casting to get floors
  y = ind/width_;
  x = ind-(y*width_);

  //std::cout << "Going from " << ind << " to " << x << " " << y << std::endl;
}

size_t CostMap2D::getWidth() const {
  return width_;
}

size_t CostMap2D::getHeight() const {
  return height_;
}

TICK CostMap2D::getElapsedTime(double ts) {
  //ROS_ASSERT(ts >= lastTimeStamp_);

  double count = (ts - lastTimeStamp_) / tickLength_;

  TICK result = (count > WATCHDOG_LIMIT ? WATCHDOG_LIMIT : (TICK) count);

  lastTimeStamp_ = ts;

  return result;
}

unsigned char CostMap2D::operator [](unsigned int ind) const{
  // ROS_ASSERT on index
  return fullData_[ind];
}
