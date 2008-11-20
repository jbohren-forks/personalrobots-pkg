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

  // Just clean up outstanding observations
  ObservationBuffer::~ObservationBuffer(){
    while(!buffer_.empty()){
      std::list<Observation>::iterator it = buffer_.begin();
      const std_msgs::PointCloud* cloud = it->cloud_;
      delete cloud;
      buffer_.erase(it);
    }
  }

  // Only works if the observation is in the map frame - test for it. It should be transformed before
  // we enque it
  bool ObservationBuffer::buffer_observation(const Observation& observation){
    last_updated_ = observation.cloud_->header.stamp;

    if(observation.cloud_->header.frame_id != "map")
      return false;

    // If the duration is 0, then we just keep the latest one, so we clear out all existing observations
    while(!buffer_.empty()){
      std::list<Observation>::iterator it = buffer_.begin();
      // Get the current one, and check if still alive. if so
      Observation& obs = *it;
      if((last_updated_ - obs.cloud_->header.stamp) > keep_alive_){
        delete obs.cloud_;
        buffer_.erase(it);
      }
      else 
        break;
    }

    // Otherwise just store it and indicate success
    buffer_.push_back(observation);
    return true;
  }

  void ObservationBuffer::get_observations(std::vector<Observation>& observations){
    // Add all remaining observations to the output
    for(std::list<Observation>::const_iterator it = buffer_.begin(); it != buffer_.end(); ++it){
      observations.push_back(*it);
    }
  }

  CostMap2D::CostMap2D(unsigned int width, unsigned int height, const std::vector<unsigned char>& data,
      double resolution, unsigned char threshold, double maxZ, double zLB, double zUB,
      double inflationRadius,	double circumscribedRadius, double inscribedRadius, double weight, 
      double  obstacleRange, double raytraceRange)
    : ObstacleMapAccessor(0, 0, width, height, resolution),
    maxZ_(maxZ), zLB_(zLB), zUB_(zUB),
    inflationRadius_(toCellDistance(inflationRadius, (unsigned int) ceil(width * resolution), resolution)),
    circumscribedRadius_(toCellDistance(circumscribedRadius, inflationRadius_, resolution)),
    inscribedRadius_(toCellDistance(inscribedRadius, circumscribedRadius_, resolution)),
    weight_(std::max(0.0, std::min(weight, 1.0))), sq_obstacle_range_(obstacleRange * obstacleRange),
    sq_raytrace_range_((raytraceRange / resolution) * (raytraceRange / resolution)), 
    staticData_(NULL), xy_markers_(NULL)
  {
    if(weight != weight_){
      ROS_INFO("Warning - input weight %f is invalid and has been set to %f\n", weight, weight_);
    }

    unsigned int i, j;
    staticData_ = new unsigned char[width_*height_];
    xy_markers_ = new bool[width_*height_];
    memset(xy_markers_, 0, width_ * height_* sizeof(bool));

    cachedDistances = new double*[inflationRadius_+1];
    for (i=0; i<=inflationRadius_; i++) {
      cachedDistances[i] = new double[inflationRadius_+1];
      for (j=0; j<=i; j++) {
        cachedDistances[i][j] = sqrt (pow(i, 2) + pow(j, 2));
        cachedDistances[j][i] = cachedDistances[i][j];
      }
    }

    setCircumscribedCostLowerBound(computeCost(circumscribedRadius_));

    // For the first pass, just clean up the data and get the set of original obstacles.
    std::vector<unsigned int> updates;
    for (unsigned int i=0;i<width_;i++){
      for (unsigned int j=0;j<height_;j++){
        unsigned int ind = MC_IND(i, j);
        costData_[ind] = data[ind];

        // If the source value is greater than the threshold but less than the NO_INFORMATION LIMIT
        // then set it to the threshold. 
        if (costData_[ind] != NO_INFORMATION && costData_[ind] >= threshold)
          costData_[ind] = LETHAL_OBSTACLE;

        // Lethal obstacles will have to be inflated. We take the approach that they will all be treated initially
        // as dynamic obstacles, and will be faded out as such, but
        if(costData_[ind] == LETHAL_OBSTACLE){
          enqueue(ind, i, j);
        }
      }
    }

    // Now propagate the costs derived from static data
    propagateCosts();

    // Instantiate static data
    memcpy(staticData_, costData_, width_ * height_);
  }

  CostMap2D::~CostMap2D() {
    if(staticData_ != NULL) delete[] staticData_;
    if(xy_markers_ != NULL) delete[] xy_markers_;
    if(cachedDistances != NULL){
      for (unsigned int i=0; i<=inflationRadius_; i++)
        delete[] cachedDistances[i];
      delete[] cachedDistances;
    }
  }


  /**
   * @brief Updated dyanmic obstacles and compute a diff. Mainly for backward compatibility. This will go away soon.
   */
  void CostMap2D::updateDynamicObstacles(double wx, double wy,
      const std::vector<std_msgs::PointCloud*>& clouds,
      std::vector<unsigned int>& updates){
    updates.clear();

    // Store the current cost data
    unsigned char* oldValues = new unsigned char[width_ * height_];
    memcpy(oldValues, costData_, width_ * height_);
    updateDynamicObstacles(wx, wy, clouds);

    // Compute the diff. Naive implementation
    unsigned int i = width_*height_;
    while(i> 0){
      i--;
      if(oldValues[i] != costData_[i])
        updates.push_back(i);
    }

    delete[] oldValues;
  }

  void CostMap2D::updateDynamicObstacles(double wx, double wy,
      const std::vector<std_msgs::PointCloud*>& clouds)
  {
    std_msgs::Point origin;
    origin.x = wx;
    origin.y = wy;
    origin.z = zLB_;
    std::vector<Observation> observations;
    for(std::vector<std_msgs::PointCloud*>::const_iterator it = clouds.begin(); it != clouds.end(); ++it){
      Observation obs(origin, *it);
      observations.push_back(obs);
    }

    updateDynamicObstacles(wx, wy, observations);
  }

  /**
   * @brief Update the cost map based on aggregate observations
   * @todo Deprecate use of wx and wy here
   */
  void CostMap2D::updateDynamicObstacles(double wx, double wy, const std::vector<Observation>& observations)
  {
    // Revert to initial state
    memset(xy_markers_, 0, width_ * height_ * sizeof(bool));
    memcpy(costData_, staticData_, width_ * height_);


    // Now propagate free space. We iterate again over observations, process only those from an origin
    // within a specific range, and a point within a certain z-range. We only want to propagate free space
    // in 2D so keep point and its origin within expected range
    for(std::vector<Observation>::const_iterator it = observations.begin(); it!= observations.end(); ++it){
      const Observation& obs = *it;
      if(!in_projection_range(obs.origin_.z))
        continue;

      const std_msgs::PointCloud& cloud = *(obs.cloud_);
      for(size_t i = 0; i < cloud.get_pts_size(); i++) {
        if(!in_projection_range(cloud.pts[i].z))
          continue;

        updateFreeSpace(obs.origin_, cloud.pts[i].x, cloud.pts[i].y);
      }
    }

    // Propagation queue should be empty from completion of last propagation.
    ROS_ASSERT(queue_.empty());

    // First we propagate costs. For this we iterate over observations, and process the internal point clouds
    for(std::vector<Observation>::const_iterator it = observations.begin(); it!= observations.end(); ++it){
      const Observation& obs = *it;
      const std_msgs::PointCloud& cloud = *(obs.cloud_);
      for(size_t i = 0; i < cloud.get_pts_size(); i++) {
        // Filter out points too high (can use for free space propagation?)
        if(cloud.pts[i].z > maxZ_)
          continue;

        //compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (cloud.pts[i].x - obs.origin_.x) * (cloud.pts[i].x - obs.origin_.x) 
          + (cloud.pts[i].y - obs.origin_.y) * (cloud.pts[i].y - obs.origin_.y) 
          + (cloud.pts[i].z - obs.origin_.z) * (cloud.pts[i].z - obs.origin_.z);

        //Filter out points that are outside of the max range we'll consider
        if(sq_dist >= sq_obstacle_range_)
          continue;

        // Queue cell for cost propagation
        unsigned int ind = WC_IND(cloud.pts[i].x, cloud.pts[i].y);

        // If we have already processed the cell for this point, skip it
        if(marked(ind))
          continue;

        // Buffer for cost propagation. This will mark the cell
        unsigned int mx, my;
        IND_MC(ind, mx, my);
        enqueue(ind, mx, my);
      }
    }

    // Propagate costs
    propagateCosts();
  }

  void CostMap2D::getOccupiedCellDataIndexList(std::vector<unsigned int>& results) const {
    results.clear();
    unsigned int maxCellCount = getWidth() * getHeight();
    for(unsigned int i = 0; i < maxCellCount; i++){
      if(costData_[i] == INSCRIBED_INFLATED_OBSTACLE || costData_[i] == LETHAL_OBSTACLE)
        results.push_back(i);
    }
  }

  void CostMap2D::propagateCosts(){
    while(!queue_.empty()){
      QueueElement* c = queue_.top();
      queue_.pop();
      unsigned char cost = computeCost(c->distance);
      updateCellCost(c->ind, cost);  

      // If distance reached the inflation radius then skip further expansion
      if(c->distance < inflationRadius_)
        enqueueNeighbors(c->source, c->ind);

      delete c;
    }
  }

  /**
   * @brief It is arguable if this is the correct update rule. We are trying to avoid
   * tracing holes through walls by propagating adjacent cells that are in sensor range through
   * those cells in the map that are no out of range. If we do not use prior values then
   * we will trace holes through walls. 
   */
  void CostMap2D::updateCellCost(unsigned int ind, unsigned char cost){
    costData_[ind] = std::max(cost, costData_[ind]);
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
    if(!marked(ind)){
      QueueElement* c = new QueueElement(computeDistance(source, ind), source, ind);
      queue_.push(c);
      mark(ind);
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
    else {
      // The cost for a non-lethal obstacle should be in the range of [0, inscribed_inflated_obstacle - 1].
      cost = (unsigned char) ( (INSCRIBED_INFLATED_OBSTACLE -1) * weight_/ pow(distance - inscribedRadius_, 2));
    }
    return cost;
  }


  /**
   * Utilizes Eitan's implementation of Bresenhams ray tracing algorithm to iterate over the cells between the
   * origin and the sightline.
   */
  void CostMap2D::updateFreeSpace(const std_msgs::Point& origin, double wx, double wy){
    unsigned int x, y, x1, y1;
    WC_MC(origin.x, origin.y, x, y);
    WC_MC(wx, wy, x1, y1);

    //for determining where we should stop raytracing
    unsigned int startx = x;
    unsigned int starty = y;

    unsigned int deltax = abs(x1 - x);        // The difference between the x's
    unsigned int deltay = abs(y1 - y);        // The difference between the y's

    unsigned int xinc1, xinc2, yinc1, yinc2;
    unsigned int den, num, numadd, numpixels;

    if (x1 >= x)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y)                 // The y-values are increasing
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
      unsigned int index = MC_IND(x, y);

      // Update x and y values
      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }

      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate

      if(!marked(index))
        costData_[index] = 0;

      //we want to break out of the loop if we have raytraced far enough
      double sq_cell_dist = (x - startx) * (x - startx) + (y - starty) * (y - starty);
      if(sq_cell_dist >= sq_raytrace_range_)
        break;

    }
  }

  CostMapAccessor::CostMapAccessor(const CostMap2D& costMap, double maxSize, double poseX, double poseY)
    : ObstacleMapAccessor(computeWX(costMap, maxSize, poseX, poseY),
        computeWY(costMap, maxSize, poseX, poseY), 
        computeSize(maxSize, costMap.getResolution()), 
        computeSize(maxSize, costMap.getResolution()), 
        costMap.getResolution()), costMap_(costMap),
    maxSize_(maxSize){

      setCircumscribedCostLowerBound(costMap.getCircumscribedCostLowerBound());

      // Set robot position
      updateForRobotPosition(poseX, poseY);

      ROS_DEBUG_NAMED("costmap_2d", "Creating Local %d X %d Map\n", getWidth(), getHeight());
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

    // Now update all the cells from the cost map
    for(unsigned int x = 0; x < width_; x++){
      for (unsigned int y = 0; y < height_; y++){
        unsigned int ind = x + y * width_;
        unsigned int global_ind = mx_0_ + x + (my_0_ + y) * costMap_.getWidth();
        costData_[ind] = costMap_[global_ind];
      }
    }
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
