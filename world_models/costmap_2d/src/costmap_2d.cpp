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
#include "rosconsole/rosconsole.h"
#include "ros/common.h"
#include <algorithm>
#include <set>
#include <sstream>
#include <assert.h>

namespace costmap_2d {

  unsigned int toCellDistance(double d, unsigned int maxD, double resolution){
    double a = std::max(0.0, ceil(d / resolution));
    if(a > maxD)
      return maxD;
    
    return (unsigned int) a;
  }

  CostMap2D::CostMap2D(unsigned int width, unsigned int height, const std::vector<unsigned char>& data,
		       double resolution, double window_length,  unsigned char threshold, 
		       double maxZ, double freeSpaceProjectionHeight,
		       double inflationRadius,	double circumscribedRadius, double inscribedRadius)
    : ObstacleMapAccessor(0, 0, width, height, resolution),
      tickLength_(window_length/WATCHDOG_LIMIT),
      maxZ_(maxZ),
      freeSpaceProjectionHeight_(freeSpaceProjectionHeight),
      inflationRadius_(toCellDistance(inflationRadius, (unsigned int) ceil(width * resolution), resolution)),
      circumscribedRadius_(toCellDistance(circumscribedRadius, inflationRadius_, resolution)),
      inscribedRadius_(toCellDistance(inscribedRadius, circumscribedRadius_, resolution)),
      staticData_(NULL), fullData_(NULL), obsWatchDog_(NULL), lastTimeStamp_(0.0), mx_(0), my_(0)
  {
    unsigned int i, j;
    staticData_ = new unsigned char[width_*height_];
    fullData_ = new unsigned char[width_*height_];
    obsWatchDog_ = new TICK[width_*height_];
    memset(fullData_, 0, width_*height_);
    memset(obsWatchDog_, 0, width_*height_);

    cachedDistances = new double*[inflationRadius_+1];
    for (i=0; i<=inflationRadius_; i++) {
      cachedDistances[i] = new double[inflationRadius_+1];
      for (j=0; j<=i; j++) {
	cachedDistances[i][j] = sqrt (pow(i,2) + pow(j,2));
	cachedDistances[j][i] = cachedDistances[i][j];
      }
    }

    // For the first pass, just clean up the data and get the set of original obstacles.
    std::vector<unsigned int> updates;
    for (unsigned int i=0;i<width_;i++){
      for (unsigned int j=0;j<height_;j++){
	unsigned int ind = MC_IND(i, j);
	staticData_[ind] = data[ind];

	// If the source value is greater than the threshold but less than the NO_INFORMATION LIMIT
	// then set it to the threshold. 
	if (staticData_[ind] != NO_INFORMATION && staticData_[ind] >= threshold)
	  staticData_[ind] = LETHAL_OBSTACLE;

	// Lethal obstacles will have to be inflated. We take the approach that they will all be treated initially
	// as dynamic obstacles, and will be faded out as such, but
	if(staticData_[ind] == LETHAL_OBSTACLE){
	  enqueue(ind, i, j);
	  staticObstacles_.push_back(ind);
	}
	else
	  fullData_[ind] = staticData_[ind];
      }
    }

    // Now propagate the queue. The updates must be reapplied to the static map, which will ensure they are
    // not staled out
    propagateCosts(updates);

    // Now for every original obstacle, go back and fill in the inflated obstacles
    for(std::vector<unsigned int>::const_iterator it = updates.begin(); it != updates.end(); ++it){
      unsigned int ind = *it;
      staticData_[ind] = fullData_[ind];
      obsWatchDog_[ind] = WATCHDOG_LIMIT - 1;
    }
  }

  CostMap2D::~CostMap2D() {
    if(staticData_ != NULL) delete[] staticData_;
    if(fullData_ != NULL) delete[] fullData_;
    if(obsWatchDog_ != NULL) delete[] obsWatchDog_;
  }

  void CostMap2D::updateDynamicObstacles(double ts,
					 const std_msgs::PointCloud& cloud,
					 std::vector<unsigned int>& updates)
  {
    updateDynamicObstacles(ts, 0, 0, cloud, updates);
  }

  void CostMap2D::updateDynamicObstacles(double ts,
					 double wx, double wy,
					 const std_msgs::PointCloud& cloud,
					 std::vector<unsigned int>& updates)
  {
    // Update current grid position
    WC_MC(wx, wy, mx_, my_);

    updates.clear();
    ROS_ASSERT(queue_.empty());
    std::vector<unsigned int> cellHits;

    for(size_t i = 0; i < cloud.get_pts_size(); i++) {
      // Filter out points too high
      if(cloud.pts[i].z > maxZ_)
	continue;

      // Queue cell for cost propagation
      unsigned int ind = WC_IND(cloud.pts[i].x, cloud.pts[i].y);

      // If we have already processed the cell for this point, skip it
      if(obsWatchDog_[ind] == MARKED_FOR_COST)
	continue;

      // Buffer for cost propagation
      unsigned int mx, my;
      IND_MC(ind, mx, my);
      enqueue(ind, mx, my);

      // Immediately update free space, which is dominated by propagated costs so they are applied afterwards.
      // This only applies for points with the projection height limit
      if(cloud.pts[i].z <= freeSpaceProjectionHeight_)
	cellHits.push_back(ind);
    }

    // Propagate costs
    propagateCosts(updates);

    // Propagate free space
    for(std::vector<unsigned int>::const_iterator it = cellHits.begin(); it != cellHits.end(); ++it)
      updateFreeSpace(*it, updates);

    // We always process deletions too
    removeStaleObstaclesInternal(ts, updates);
  }

  void CostMap2D::updateCellCost(unsigned int cell, unsigned char cellState, std::vector<unsigned int>& updates){
    // Will take the higher of the static value and the new value.
    // We should also ensure that dynamic obstacles do not take the value of no information. That is basically unhandled
    // and we should address what to do there once we have use cases to implement that exploit this unknown state
    unsigned char oldValue = fullData_[cell];
    unsigned char staticValue = (staticData_[cell] == NO_INFORMATION ? 0 : staticData_[cell]);
    fullData_[cell] = std::min(std::max(cellState, staticValue), LETHAL_OBSTACLE);

    if(fullData_[cell] != oldValue)
      addUpdate(cell, updates);
  }

  void CostMap2D::petWatchDog(unsigned int cell, unsigned char value){
    // If it is a new dynamic obstacle, record it
    if(obsWatchDog_[cell] == 0)
      dynamicObstacles_.push_back(cell);

    // Always pet the watchdog
    obsWatchDog_[cell] = value;
  }

  void CostMap2D::markFreeSpace(unsigned int cell, std::vector<unsigned int>& updates){
    // If not currently free space then buffer as an update
    if(fullData_[cell] != 0)
      addUpdate(cell, updates);

    // Assign to free space
    fullData_[cell] = 0;

    petWatchDog(cell, WATCHDOG_LIMIT);
  }

  void CostMap2D::updateDynamicObstacles(double ts, const std_msgs::PointCloud& cloud){
    std::vector<unsigned int> updates;
    updateDynamicObstacles(ts, cloud, updates);
  }

  /**
   * This algorithm uses the concept of a watchdog timer to decide when an obstacle can be removed.
   */
  void CostMap2D::removeStaleObstacles(double ts, std::vector<unsigned int>& updates){
    updates.clear();
    removeStaleObstaclesInternal(ts, updates);
  }

  void CostMap2D::removeStaleObstaclesInternal(double ts, std::vector<unsigned int>& updates){
    // Calculate elapsed time in terms of ticks
    TICK elapsedTime = getElapsedTime(ts);

    // Iterate over the set of dyanmic obstacles
    std::list<unsigned int>::iterator it = dynamicObstacles_.begin();

    while(it != dynamicObstacles_.end()){
      unsigned int ind = *it;
      TICK timeLeft = obsWatchDog_[ind];

      // Case 1: The watchdog has just been reset. Here we decrement the watchdog by 1 and move on
      if(timeLeft >= WATCHDOG_LIMIT){
	obsWatchDog_[ind] = WATCHDOG_LIMIT - 1;
	++it;
	continue;
      }

      // Case 2: The watchdog has timed out. Here we remove the obstacle (if not static), zero the watchdog timer, and move on.
      if(timeLeft <= elapsedTime){
	it = dynamicObstacles_.erase(it);
	obsWatchDog_[ind] = 0;
	unsigned char oldValue = fullData_[ind];
	fullData_[ind] = staticData_[ind];

	// If the new value differs from the old value then insert it
	if(fullData_[ind] != oldValue)
	  addUpdate(ind, updates);

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
    results.clear();
    unsigned int maxCellCount = getWidth() * getHeight();
    for(unsigned int i = 0; i < maxCellCount; i++){
      if(fullData_[i] == INSCRIBED_INFLATED_OBSTACLE || fullData_[i] == LETHAL_OBSTACLE)
	results.push_back(i);
    }
  }

  TICK CostMap2D::getElapsedTime(double ts) {

    double count = (ts - lastTimeStamp_) / tickLength_;

    TICK result = (count > WATCHDOG_LIMIT ? WATCHDOG_LIMIT : (TICK) count);

    lastTimeStamp_ = ts;

    return result;
  }

  unsigned char CostMap2D::operator [](unsigned int ind) const{
    return fullData_[ind];
  }

  unsigned char CostMap2D::getCost(unsigned int mx, unsigned int my) const{
    return fullData_[MC_IND(mx, my)];
  }

  std::string CostMap2D::toString() const {
    std::stringstream ss;
    ss << std::endl;
    for(unsigned j = 0; j < getHeight(); j++){
      for (unsigned int i = 0; i < getWidth(); i++){
	unsigned char cost = getCost(i, j);
	if(cost == LETHAL_OBSTACLE)
	  ss << "O";
	else if (cost == INSCRIBED_INFLATED_OBSTACLE)
	  ss << "I";
	else if (cost == CIRCUMSCRIBED_INFLATED_OBSTACLE)
	  ss << "C";
	else if (cost == NO_INFORMATION)
	  ss << "?";
	else if (cost > 0)
	  ss << "f";
	else
	  ss << "F";

	ss << ",";
      }

      ss << std::endl;
    }
    return ss.str();
  }

  void CostMap2D::propagateCosts(std::vector<unsigned int>& updates){
    while(!queue_.empty()){
      QueueElement* c = queue_.top();
      queue_.pop();
      unsigned char cost = computeCost(c->distance);      
      updateCellCost(c->ind, cost, updates);

      // If distance reached the inflation radius then skip further expansion
      if(c->distance < inflationRadius_)
	enqueueNeighbors(c->source, c->ind);

      delete c;
    }
  }

  void CostMap2D::enqueueNeighbors(unsigned int source, unsigned int ind){
    unsigned int mx, my;
    IND_MC(ind, mx, my);

    if(mx > 0) 
      enqueue(source, mx - 1, my);
    if(my > 0) 
      enqueue(source, mx, my - 1);
    if(mx < (getWidth() - 1)) 
      enqueue(source, mx + 1, my);
    if(my < (getHeight() - 1)) 
      enqueue(source, mx, my + 1);
  }

  void CostMap2D::enqueue(unsigned int source, unsigned int mx, unsigned int my){
    // If the cell is not marked for cost propagation
    unsigned int ind = MC_IND(mx, my);
    if(obsWatchDog_[ind] != MARKED_FOR_COST){
      QueueElement* c = new QueueElement(computeDistance(source, ind), source, ind);
      queue_.push(c);
      petWatchDog(ind, MARKED_FOR_COST);
    }
  }

  /**
   * Euclidean distance
   */
  double CostMap2D::computeDistance(unsigned int a, unsigned int b) const{
    unsigned int mx_a, my_a, mx_b, my_b;
    IND_MC(a, mx_a, my_a);
    IND_MC(b, mx_b, my_b);
    unsigned int dx = abs((int)(mx_a) - (int) mx_b);
    unsigned int dy = abs((int)(my_a) - (int) my_b);

    ROS_ASSERT((dx <= inflationRadius_) && (dy <= inflationRadius_));
    double distance = cachedDistances[dx][dy];

    return distance;
  }

  unsigned char CostMap2D::computeCost(double distance) const{
    unsigned char cost = 0;
    if(distance == 0)
      cost = LETHAL_OBSTACLE;
    else if(distance <= inscribedRadius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;
    else if (distance <= circumscribedRadius_)
      cost = CIRCUMSCRIBED_INFLATED_OBSTACLE;
    else
      cost = (unsigned char) (CIRCUMSCRIBED_INFLATED_OBSTACLE / pow(2, distance));

    return cost;
  }


  /**
   * Utilizes Eitan's implementation of Bresenhams ray tracing algorithm to iterate over the cells between the
   * current position (mx_, my_).
   */
  void CostMap2D::updateFreeSpace(unsigned int ind, std::vector<unsigned int>& updates){
    unsigned int x1, y1;
    IND_MC(ind, x1, y1);

    unsigned int deltax = abs(x1 - mx_);        // The difference between the x's
    unsigned int deltay = abs(y1 - my_);        // The difference between the y's
    unsigned int x = mx_;                       // Start x off at the first pixel
    unsigned int y = my_;                       // Start y off at the first pixel

    unsigned int xinc1, xinc2, yinc1, yinc2;
    unsigned int den, num, numadd, numpixels;

    if (x1 >= mx_)                 // The x-values are increasing
      {
	xinc1 = 1;
	xinc2 = 1;
      }
    else                          // The x-values are decreasing
      {
	xinc1 = -1;
	xinc2 = -1;
      }

    if (y1 >= my_)                 // The y-values are increasing
      {
	yinc1 = 1;
	yinc2 = 1;
      }
    else                          // The y-values are decreasing
      {
	yinc1 = -1;
	yinc2 = -1;
      }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
      {
	xinc1 = 0;                  // Don't change the x when numerator >= denominator
	yinc2 = 0;                  // Don't change the y for every iteration
	den = deltax;
	num = deltax / 2;
	numadd = deltay;
	numpixels = deltax;         // There are more x-values than y-values
      }
    else {
	xinc2 = 0;                  // Don't change the x for every iteration
	yinc1 = 0;                  // Don't change the y when numerator >= denominator
	den = deltay;
	num = deltay / 2;
	numadd = deltax;
	numpixels = deltay;         // There are more y-values than x-values
    }

    for (unsigned int curpixel = 0; curpixel <= numpixels; curpixel++){
      unsigned int ind = MC_IND(x, y);

      // If the cell is updated for cost, then it is not free space and we should not ray trace through it
      if(obsWatchDog_[ind] == MARKED_FOR_COST && fullData_[ind] < CIRCUMSCRIBED_INFLATED_OBSTACLE)
	return;

      // If already marked, ignore it
      if(obsWatchDog_[ind] < WATCHDOG_LIMIT)
	markFreeSpace(ind, updates);

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
	{
	  num -= den;               // Calculate the new numerator value
	  x += xinc1;               // Change the x as appropriate
	  y += yinc1;               // Change the y as appropriate
	}

      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }
  }

  CostMapAccessor::CostMapAccessor(const CostMap2D& costMap, double maxSize, double poseX, double poseY)
    : ObstacleMapAccessor(computeWX(costMap, maxSize, poseX, poseY),
			  computeWY(costMap, maxSize, poseX, poseY), 
			  computeSize(maxSize, costMap.getResolution()), 
			  computeSize(maxSize, costMap.getResolution()), 
			  costMap.getResolution()), costMap_(costMap),
      maxSize_(maxSize){

    // The origin locates this grid. Convert from world coordinates to cell co-ordinates
    // to get the cell coordinates of the origin
    costMap_.WC_MC(origin_x_, origin_y_, mx_0_, my_0_); 

    ROS_DEBUG_NAMED("costmap_2d", "Creating Local %d X %d Map\n", getWidth(), getHeight());
  }

  unsigned char CostMapAccessor::operator[](unsigned int ind) const {
    double wx, wy;
    IND_WC(ind, wx, wy);
    return costMap_[costMap_.WC_IND(wx, wy)];
  }

  unsigned char CostMapAccessor::getCost(unsigned int mx, unsigned int my) const {
    double wx, wy;
    MC_WC(mx, my, wx, wy);
    return costMap_[costMap_.WC_IND(wx, wy)];
  }

  void CostMapAccessor::updateForRobotPosition(double wx, double wy){
    if(wx < 0 || wx > costMap_.getResolution() * costMap_.getWidth())
      return;

    if(wy < 0 || wy > costMap_.getResolution() * costMap_.getHeight())
      return;

    origin_x_ = computeWX(costMap_, maxSize_, wx, wy);
    origin_y_ = computeWY(costMap_, maxSize_, wx, wy);
    costMap_.WC_MC(origin_x_, origin_y_, mx_0_, my_0_); 

    ROS_DEBUG_NAMED("costmap_2d", 
		    "Moving map to locate at <%f, %f> and size of %f meters for position <%f, %f>\n",
		    origin_x_, origin_y_, maxSize_, wx, wy);
  }

  double CostMapAccessor::computeWX(const CostMap2D& costMap, double maxSize, double wx, double wy){
    unsigned int mx, my;
    costMap.WC_MC(wx, wy, mx, my);

    unsigned int cellWidth = (unsigned int) (maxSize/costMap.getResolution());
    unsigned int origin_mx(0);

    if(mx > cellWidth/2)
      origin_mx = mx - cellWidth/2;

    if(origin_mx + cellWidth > costMap.getWidth())
      origin_mx = costMap.getWidth() - cellWidth;

    return origin_mx * costMap.getResolution();
  }

  double CostMapAccessor::computeWY(const CostMap2D& costMap, double maxSize, double wx, double wy){
    unsigned int mx, my;
    costMap.WC_MC(wx, wy, mx, my);

    unsigned int cellWidth = (unsigned int) (maxSize/costMap.getResolution());
    unsigned int origin_my(0);

    if(my > cellWidth/2)
      origin_my = my - cellWidth/2;

    if(origin_my + cellWidth > costMap.getHeight())
      origin_my = costMap.getHeight() - cellWidth;

    return origin_my * costMap.getResolution();
  }

  unsigned int CostMapAccessor::computeSize(double maxSize, double resolution){
    unsigned int cellWidth = (unsigned int) ceil(maxSize/resolution);
    ROS_DEBUG_NAMED("costmap_2d", "Given a size of %f and a resolution of %f, we have a cell width of %d\n", maxSize, resolution, cellWidth);
    return cellWidth;
  }




}
