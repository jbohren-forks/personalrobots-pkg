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
#include <algorithm>
#include <set>
#include <sstream>
#include <assert.h>

namespace costmap_2d {

  unsigned int toCellDistance(double d, unsigned int minD, double resolution){
    double a = ceil(d / resolution);
    if(a > minD)
      return (unsigned int) a;
    
    return (unsigned int) a;
  }

  CostMap2D::CostMap2D(unsigned int width, unsigned int height, const std::vector<unsigned char>& data,
		       double resolution, double window_length,  unsigned char threshold, 
		       double maxZ, double inflationRadius,
		       double circumscribedRadius, double inscribedRadius)
    : ObstacleMapAccessor(0, 0, width, height, resolution),
      tickLength_(window_length/WATCHDOG_LIMIT),
      maxZ_(maxZ),
      inflationRadius_(toCellDistance(inflationRadius, 0, resolution)),
      circumscribedRadius_(toCellDistance(circumscribedRadius, inflationRadius_, resolution)),
      inscribedRadius_(toCellDistance(inscribedRadius, circumscribedRadius_, resolution)),
      staticData_(NULL), fullData_(NULL), obsWatchDog_(NULL), lastTimeStamp_(0.0)
  {
    staticData_ = new unsigned char[width_*height_];
    fullData_ = new unsigned char[width_*height_];
    obsWatchDog_ = new TICK[width_*height_];
    memset(fullData_, 0, width_*height_);
    memset(obsWatchDog_, 0, width_*height_);

    // For the first pass, just clean up the data and get the set of original obstacles.
    QUEUE queue; // queue for propagating costs
    for (unsigned int i=0;i<width_;i++){
      for (unsigned int j=0;j<height_;j++){
	unsigned int ind = MC_IND(i, j);
	staticData_[ind] = data[ind];

	// If the source value is greater than the threshold but less than the NO_INFORMATION LIMIT
	// then set it to the threshold
	if(staticData_[ind] >= threshold  && staticData_[ind] != NO_INFORMATION)
	  staticData_[ind] = LETHAL_OBSTACLE;

	if(staticData_[ind] == LETHAL_OBSTACLE){
	  queue.push(std::make_pair(0, ind));
	  staticObstacles_.push_back(ind);
	}
      }
    }

    // Now propagate the queue. The updates must be reapplied to the static map, which will ensure they are
    // not staled out
    std::set<unsigned int> updates;
    propagate(queue, updates);


    // Now for every original obstacle, go back and fill in the inflated obstacles
    for(std::set<unsigned int>::const_iterator it = updates.begin(); it != updates.end(); ++it){
      unsigned int ind = *it;
      staticData_[ind] = fullData_[ind];
    }

    std::cout << "CostMap 2D created with " << staticObstacles_.size() << " static obstacles" << std::endl;
  }

  CostMap2D::~CostMap2D() {
    if(staticData_ != NULL) delete[] staticData_;
    if(fullData_ != NULL) delete[] fullData_;
    if(obsWatchDog_ != NULL) delete[] obsWatchDog_;
  }

  void CostMap2D::updateDynamicObstacles(double ts,
					 const std_msgs::PointCloudFloat32& cloud,
					 std::set<unsigned int>& updates)
  {
    double FAR_AWAY = getWidth() * getHeight() * getResolution();
    updateDynamicObstacles(ts, FAR_AWAY, FAR_AWAY, cloud, updates);
  }

  void CostMap2D::updateDynamicObstacles(double ts,
					 double wx, double wy,
					 const std_msgs::PointCloudFloat32& cloud,
					 std::set<unsigned int>& updates)
  {
    updates.clear();

    QUEUE queue;

    // Small clearance for what is 'in contact' which is a centimeter
    for(size_t i = 0; i < cloud.get_pts_size(); i++) {
      // Filter out points too high
      if(cloud.pts[i].z > maxZ_)
	continue;

      unsigned int ind = WC_IND(cloud.pts[i].x, cloud.pts[i].y);
      queue.push(std::make_pair(0, ind));
    }

    // Propagate costs
    propagate(queue, updates);

    // We always process deletions too
    removeStaleObstacles(ts, updates);

    if(!updates.empty())
      std::cout << updates.size() << " cells updated.\n";
  }

  bool CostMap2D::updateCell(unsigned int cell, unsigned char cellState, std::set<unsigned int>& updates){
    unsigned char oldValue = fullData_[cell];

    // If it is a new dynamic obstacle, record it
    if(obsWatchDog_[cell] == 0)
      dynamicObstacles_.push_back(cell);

    // Will take the higher of the static value and the new value.
    // We should also ensure that dynamic obstacles do not take the value of no information. That is basically unhandled
    // and we should address what to do there once we have use cases to implement that exploit this unknown state
    fullData_[cell] = std::min(std::max(cellState, staticData_[cell]), LETHAL_OBSTACLE);
  
    // Always pet the watchdog
    obsWatchDog_[cell] = WATCHDOG_LIMIT;

    if(fullData_[cell] != oldValue){
      //printf("Switching %d from %d to %d\n", cell, oldValue, fullData_[cell]);
      updates.insert(cell);
      return true;
    }

    return false;
  }

  void CostMap2D::updateDynamicObstacles(double ts, const std_msgs::PointCloudFloat32& cloud){
    std::set<unsigned int> updates;
    updateDynamicObstacles(ts, cloud, updates);
  }

  /**
   * This algorithm uses the concept of a watchdog timer to decide when an obstacle can be removed.
   */
  void CostMap2D::removeStaleObstacles(double ts, std::set<unsigned int>& updates){

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

      // Case 2: The watchdog has timed out. Here we remove the obstacle (if not static), zero the watchdog timer, and move on.
      if(timeLeft <= elapsedTime){
	it = dynamicObstacles_.erase(it);
	obsWatchDog_[ind] = 0;
	unsigned char oldValue = fullData_[ind];
	fullData_[ind] = staticData_[ind];

	// If the new value differs from the old value then insert it
	if(fullData_[ind] != oldValue)
	  updates.insert(ind);

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

  unsigned char CostMap2D::getCost(unsigned int mx, unsigned int my) const{
    return fullData_[MC_IND(mx, my)];
  }

  /**
   * @brief Returns the set of cell id's for cells that should be considered obstacles given
   * the inflation radius and the map resolution.
   *
   * @param ind The index of the new obstacle
   * @param inflation The resulting inflation set
   * @todo Consider using euclidean distance check for inflation, by testng if the origin of a cell
   * is within the inflation radius distance of the origitn of the given cell.
   */
  void CostMap2D::computeInflation(unsigned int ind, std::vector< std::pair<unsigned int, unsigned char> >& inflation) const {
    // Identify the bounds on the cells to inflate
    unsigned int mx, my;
    IND_MC(ind, mx, my);
    unsigned int cellRadius = static_cast<unsigned int>(ceil(inflationRadius_/resolution_));
    unsigned int x_min = static_cast<unsigned int>(std::max<int>(0, mx - cellRadius));
    unsigned int x_max = static_cast<unsigned int>(std::min<int>(getWidth() - 1, mx + cellRadius));
    unsigned int y_min = static_cast<unsigned int>(std::max<int>(0, my - cellRadius));
    unsigned int y_max = static_cast<unsigned int>(std::min<int>(getHeight() - 1, my + cellRadius));

    double wx, wy;
    IND_WC(ind, wx, wy);
    inflation.clear();
    for(unsigned int i = x_min; i <= x_max; i++)
      for(unsigned int j = y_min; j <= y_max; j++){
	double wx_cell, wy_cell;
	MC_WC(i, j, wx_cell, wy_cell);
	double distance = sqrt(pow(wx_cell - wx, 2) + pow(wy_cell - wy, 2)) - resolution_/2;
	unsigned char cost = 0;
	if(distance <= inscribedRadius_)
	  cost = INSCRIBED_INFLATED_OBSTACLE;
	else if (distance <= circumscribedRadius_)
	  cost = CIRCUMSCRIBED_INFLATED_OBSTACLE;
	else
	  cost = (unsigned char) (CIRCUMSCRIBED_INFLATED_OBSTACLE / pow(2, distance));

	inflation.push_back( std::pair<unsigned int, unsigned char>(MC_IND(i, j), cost));
      }
  }

  std::string CostMap2D::toString() const {
    std::stringstream ss;
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

  void CostMap2D::propagate(QUEUE& queue, std::set<unsigned int>& updates){
    updates.clear();
    while(!queue.empty()){
      unsigned int distance = queue.front().first;
      unsigned int ind = queue.front().second;
      unsigned char cost = computeCost(ind, distance);
      queue.pop();
      
      if(!marked(ind)){
	updateCell(ind, cost, updates);
	unsigned int mx, my;
	IND_MC(ind, mx, my);

	// If distance reached the inflation radius then skip further expansion
	if(distance >= inflationRadius_)
	  continue;

	// Otherwise we expand further
	std::vector<unsigned int> neighbors;
	unsigned int xMin = (mx > 0 ? mx - 1 : 0);
	unsigned int xMax = (mx < getWidth() - 1 ? mx + 1 : mx);
	unsigned int yMin = (my > 0 ? my - 1 : 0);
	unsigned int yMax = (my < getHeight() - 1 ? my + 1 : my);

	for(unsigned int i = xMin; i <= xMax; i++){
	  for(unsigned int j = yMin; j <= yMax; j++){
	    unsigned int cellId = MC_IND(i, j);
	    // If the cell is not marked, insert into queue
	    if(!marked(cellId))
	      queue.push(std::make_pair(distance + 1, cellId));
	  }
	}
      }
    }
  }

  bool CostMap2D::marked(unsigned int ind) const { return obsWatchDog_[ind] == WATCHDOG_LIMIT;}

  unsigned char CostMap2D::computeCost(unsigned int ind, unsigned int distance) const{
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

    printf("Creating Local %d X %d Map\n", getWidth(), getHeight());
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

    //printf("Moving map to locate at <%f, %f> and size of %f meters for position <%f, %f>\n",
    //     origin_x_, origin_y_, maxSize_, wx, wy);
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
    printf("Given a size of %f and a resolution of %f, we have a cell width of %d\n", maxSize, resolution, cellWidth);
    return cellWidth;
  }

}
