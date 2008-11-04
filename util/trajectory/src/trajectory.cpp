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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Sachin Chitta */

#include "trajectory/trajectory.h"

#define MAX_ALLOWABLE_TIME 1.0e8

using namespace trajectory;

Trajectory::Trajectory(int dimension): dimension_(dimension)
{
  interp_method_ = "linear";
  autocalc_timing_ = false;
}

void Trajectory::clear()
{
  tp_.resize(0);
  tc_.resize(0);
  min_limit_.resize(0);
  max_limit_.resize(0);
  max_rate_.resize(0);
  max_acc_.resize(0);
}

int Trajectory::setTrajectory(const std::vector<TPoint>& tp)
{
  if(tp.size() <= 1)
  {
    ROS_WARN("Trying to set trajectory with number of points <= 0");
    return -1;
  }
  if(tp.begin()->dimension_ != dimension_)
  {
    ROS_WARN("Dimension of trajectory point %d does not match dimension of trajectory %d",tp[0].dimension_, dimension_);
    return -1;
  }

  num_points_ = tp.size();

  tp_.resize(num_points_);

  for(int i=0; i<num_points_; i++)
  {
    tp_[i].setDimension(dimension_);
    tp_[i] = tp[i];
  }

  calcCoeff(interp_method_,autocalc_timing_);

  return 1;
}  

int Trajectory::setTrajectory(const std::vector<double> &p, int numPoints)
{
  num_points_ = numPoints;

  if((int) p.size() < num_points_*dimension_)
  {
    ROS_WARN("Input has only %d values, expecting %d values for a %d dimensional trajectory with %d number of points",p.size(), num_points_*dimension_, dimension_, num_points_);
    return -1;
  }   
  autocalc_timing_ = true;//Enable autocalc timing by default since no time information given in trajectory
  tp_.resize(num_points_);

  for(int i=0; i<num_points_;i++)
  {
    tp_[i].setDimension(dimension_);
    for(int j=0; j<dimension_; j++)
      tp_[i].q_[j] = p[i*dimension_+j];
  }
  calcCoeff(interp_method_,autocalc_timing_);


  return 1;
}  

int Trajectory::setTrajectory(const std::vector<double> &p, const std::vector<double> &time, int numPoints)
{
  num_points_ = numPoints;

  tp_.resize(num_points_);

  if((int) time.size() != num_points_)
  {
    ROS_WARN("Number of points in vector specifying time (%d)  does not match number of points %d",(int) time.size(), num_points_);
    return -1;
  }
  if((int) p.size() < num_points_*dimension_)
  {
    ROS_WARN("Input has only %d values, expecting %d values for a %d dimensional trajectory with %d number of points",p.size(), num_points_*dimension_, dimension_, num_points_);
    return -1;
  }   

  for(int i=0; i<num_points_;i++)
  {
    tp_[i].setDimension(dimension_);
    tp_[i].time_ = time[i];
    for(int j=0; j<dimension_; j++)
      tp_[i].q_[j] = p[i*(dimension_+1)+j+1];
  }

  calcCoeff(interp_method_,autocalc_timing_);

  return 1;
}

void Trajectory::addPoint(const TPoint tp)
{
  double time = tp.time_;

  int index = findTrajectorySegment(time);
  std::vector<TPoint>::iterator it = tp_.begin() + index;
  tp_.insert(it,tp);
  num_points_++;
}

inline int Trajectory::findTrajectorySegment(double time)
{
  int result = 0;

  while(time > tp_[result+1].time_)
    result++;

  return result;
}

inline double Trajectory::getTotalTime()
{
  if(tp_.size() == 0)
    return 0.0;

  return (tp_.back().time_ - tp_.front().time_);
}

void Trajectory::calcCoeff(std::string interp_method, bool autocalc_timing)
{
  if(interp_method == std::string("linear"))
     calculateLinearCoeff(autocalc_timing);
  else
    ROS_WARN("Unrecognized interp_method type: %s\n",interp_method.c_str());
}

int Trajectory::sample(TPoint &tp, double time)
{
  if(time > tp_.back().time_ || time < tp_.front().time_)
  {
    ROS_WARN("Invalid input sample time.");
    return -1;
  }
  if((int) tp.q_.size() != dimension_ || (int) tp.qdot_.size() != dimension_)
  {
    ROS_WARN("Dimension of sample point passed in = %d does not match dimension of trajectory = %d",tp.q_.size(),dimension_);
    return -1;
  } 
  int segment_index = findTrajectorySegment(time);
  if(interp_method_ == std::string("linear"))
  {
    sampleLinear(tp,time,tc_[segment_index],tp_[segment_index].time_);
  }
  return 1;
}

int Trajectory::setMaxRates(std::vector<double> max_rate)
{
  if((int) max_rate.size() != dimension_)
  {
    ROS_WARN("Input size: %d does not match dimension of trajectory = %d",max_rate.size(),dimension_);
    return -1;
  }
  max_rate.resize(dimension_);
  max_rate_ = max_rate; 
  return 1;
}

int Trajectory::calculateLinearCoeff(bool autocalc_timing)
{
  double dT(0);
  TCoeff tc;
  std::vector<std::vector<double> > coeff;

  std::vector<double> temp;
  temp.resize(2);

  if(autocalc_timing)
    tp_[0].time_ = 0.0;

  tc.degree_ = 1;
  tc.dimension_ = dimension_;

  if(autocalc_timing)
  {
    if(max_rate_.empty() || (int) max_rate_.size() < 0)
    {
      ROS_WARN("Trying to use autocalc_timing without setting max rate information. Use setMaxRate first");
      return -1;
    }
  }

  for(int i=1; i < (int) tp_.size() ; i++)
  {
    if(autocalc_timing)
    {
      dT = calculateMinimumTimeLinear(tp_[i-1],tp_[i]);
      tp_[i].time_ = tp_[i-1].time_ + dT;
      ROS_INFO("dT for segment %d = %f",(i-1),dT);
    }
    else
    {
      dT = tp_[i].time_ - tp_[i-1].time_;
    }

    tc.duration_ = dT;

    for(int j=0; j < dimension_; j++)
    {
      temp[0] = tp_[i-1].q_[j];
      temp[1] = (tp_[i].q_[j] - tp_[i-1].q_[j])/tc.duration_;  
      tc.coeff_.push_back(temp);
    }
    tc_.push_back(tc);
  }
  return 1;
}

void Trajectory::sampleLinear(TPoint &tp, double time, const TCoeff &tc, double segment_start_time)
{
  double segment_time = time - segment_start_time;
  for(int i =0; i < dimension_; i++)
  {
    tp.q_[i]    =  tc.coeff_[i][0] + segment_time * tc.coeff_[i][1];
    tp.qdot_[i] =  tc.coeff_[i][1];
  }
  tp.time_ = time;
  tp.dimension_ = dimension_;
}

int Trajectory::getNumberPoints()
{
  return num_points_;
}

int Trajectory::getDuration(std::vector<double> &duration)
{
  if((int) duration.size() != num_points_-1)
  {
    ROS_WARN("Size of duration vector %d does not match number of segments in trajectory %d", duration.size(), num_points_-1);
    return -1;
  }
  for(int i = 0; i < num_points_-1; i++)
    duration[i] = tc_[i].duration_;

  return 1;
}

int Trajectory::getDuration(int index, double &duration)
{
  if(index > num_points_ -1)
  {
    ROS_WARN("Index %d outside number of segments in the trajectory %d", index, num_points_-1);
    return -1;
  }

  duration = tc_[index].duration_;

  return 1;

}

double Trajectory::calculateMinimumTimeLinear(const TPoint &start, const TPoint &end)
{
  double minJointTime(0);
  double minTime(0);

  for(int i = 0; i < start.dimension_; i++)
  {
    if(max_rate_[i] > 0)
      minJointTime = fabs(end.q_[i]-start.q_[i]) / max_rate_[i];
    else
      minJointTime = MAX_ALLOWABLE_TIME;

    if(minTime < minJointTime)
      minTime = minJointTime;

  }

  return minTime;
}

void Trajectory::setInterpolationMethod(std::string interp_method)
{
  interp_method_ = interp_method;
}
