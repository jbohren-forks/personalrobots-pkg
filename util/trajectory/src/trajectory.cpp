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
#define EPS_TRAJECTORY 1.0e-8

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
  if(tp.size() < 2)
  {
    ROS_WARN("Trying to set trajectory with number of points <= 0");
    return -1;
  }
  if(tp.begin()->dimension_ != dimension_)
  {
    ROS_WARN("Dimension of trajectory point %d does not match dimension of trajectory %d",tp[0].dimension_, dimension_);
    return -1;
  }

//  ROS_INFO("Initializing trajectory with %d points",tp.size());

  num_points_ = tp.size();

  tp_.resize(num_points_);

  for(int i=0; i<num_points_; i++)
  {
    tp_[i].setDimension(dimension_);
    tp_[i] = tp[i];
//    ROS_INFO("Input point: %d is ",i);
//    for(int j=0; j < dimension_; j++)
//      ROS_INFO("%f ",tp_[i].q_[j]);

//    ROS_INFO(" ");
  }

  parameterize();
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
    tp_[i].time_ = 0.0;
    for(int j=0; j<dimension_; j++)
    {
      tp_[i].q_[j] = p[i*dimension_+j];
      tp_[i].qdot_[j] = 0.0;
    }

//    ROS_INFO("Input point: %d is ",i);
//    for(int j=0; j < dimension_; j++)
//      ROS_INFO("%f ",tp_[i].q_[j]);

//    ROS_INFO(" ");

  }
  parameterize();
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
    {
      tp_[i].q_[j] = p[i*(dimension_)+j];
    }
  }

  parameterize();

  return 1;
}

int Trajectory::setTrajectory(const std::vector<double> &p, const std::vector<double> &pdot, const std::vector<double> &time, int numPoints)
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
    {
      tp_[i].q_[j] = p[i*dimension_+j];
      tp_[i].qdot_[j] = pdot[i*dimension_+j];
    }
  }
  parameterize();
  return 1;
}

void Trajectory::addPoint(const TPoint tp)
{
  double time = tp.time_;

  int index = findTrajectorySegment(time);
  std::vector<TPoint>::iterator it = tp_.begin() + index;
  tp_.insert(it,tp);
  num_points_++;
  parameterize();
}

inline int Trajectory::findTrajectorySegment(double time)
{
  int result = 0;

  while(time > tp_[result+1].time_)
    result++;

  return result;
}

//inline double Trajectory::getTotalTime()
double Trajectory::getTotalTime()
{
  if(tp_.size() == 0)
    return 0.0;

  return (tp_.back().time_ - tp_.front().time_);
}

int Trajectory::sample(TPoint &tp, double time)
{
//  ROS_INFO("Trajectory has %d points",tp_.size());
//  ROS_INFO("Time: %f, %f, %f",time,tp_.front().time_,tp_.back().time_);
  if(time > tp_.back().time_)
  {
//    ROS_WARN("Invalid input sample time.");
    time = tp_.back().time_;
  }
  else if( time < tp_.front().time_)
  {
    time = tp_.front().time_;
//    ROS_WARN("Invalid input sample time.");
  }

  if((int) tp.q_.size() != dimension_ || (int) tp.qdot_.size() != dimension_)
  {
    ROS_WARN("Dimension of sample point passed in = %d does not match dimension of trajectory = %d",tp.q_.size(),dimension_);
    return -1;
  } 
  int segment_index = findTrajectorySegment(time);
//  ROS_INFO("segment index : %d",segment_index);
  if(interp_method_ == std::string("linear"))
    sampleLinear(tp,time,tc_[segment_index],tp_[segment_index].time_);
  else if(interp_method_ == std::string("cubic"))
    sampleCubic(tp,time,tc_[segment_index],tp_[segment_index].time_);
  else if(interp_method_ == std::string("blended_linear"))
    sampleBlendedLinear(tp,time,tc_[segment_index],tp_[segment_index].time_);
  else
    ROS_WARN("Unrecognized interp_method type: %s\n",interp_method_.c_str());

  return 1;
}

/*
int Trajectory::sample(std::vector<TPoint> &tp, double start_time, double end_time)
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
    sampleLinear(tp,time,tc_[segment_index],tp_[segment_index].time_);
  else if(interp_method_ == std::string("cubic"))
    sampleCubic(tp,time,tc_[segment_index],tp_[segment_index].time_);
  else if(interp_method_ == std::string("blended_linear"))
    sampleBlendedLinear(tp,time,tc_[segment_index],tp_[segment_index].time_);
  else
    ROS_WARN("Unrecognized interp_method type: %s\n",interp_method_.c_str());

  return 1;
}
*/

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

int Trajectory::setMaxAcc(std::vector<double> max_acc)
{
  if((int) max_acc.size() != dimension_)
  {
    ROS_WARN("Input size: %d does not match dimension of trajectory = %d",max_acc.size(),dimension_);
    return -1;
  }
  max_acc.resize(dimension_);
  max_acc_ = max_acc; 
  return 1;
}

int Trajectory::minimizeSegmentTimes()
{
  int error_code = -1;
  if(interp_method_ == std::string("linear"))
     error_code = minimizeSegmentTimesWithLinearInterpolation();
  else if(interp_method_ == std::string("cubic"))
     error_code = minimizeSegmentTimesWithCubicInterpolation();
  else if(interp_method_ == std::string("blended_linear"))
     error_code = minimizeSegmentTimesWithBlendedLinearInterpolation();
  else
    ROS_WARN("minimizeSegmentTimes:: Unrecognized interp_method type: %s\n",interp_method_.c_str());

  return error_code;
}

int Trajectory::minimizeSegmentTimesWithLinearInterpolation()
{
  double dT(0);
  TCoeff tc;
  std::vector<std::vector<double> > coeff;

  std::vector<double> temp;
  temp.resize(2);

  tc.degree_ = 1;
  tc.dimension_ = dimension_;

  tc_.clear();

  if(max_rate_.empty() || (int) max_rate_.size() < 0)
  {
    ROS_WARN("Trying to apply rate limits without setting max rate information. Use setMaxRate first");
    return -1;
  }

  for(int i=1; i < (int) tp_.size() ; i++)
  {
    tc.coeff_.clear();
    dT = calculateMinimumTimeLinear(tp_[i-1],tp_[i]);
    tp_[i].time_ = tp_[i-1].time_ + dT;

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

int Trajectory::minimizeSegmentTimesWithCubicInterpolation()
{
  double dT(0);
  TCoeff tc;

  std::vector<double> temp;

  temp.resize(4);

  tc.degree_ = 1;
  tc.dimension_ = dimension_;

  tc_.clear();

    if(max_rate_.empty() || (int) max_rate_.size() < 1)
    {
      ROS_WARN("Trying to apply rate limits without setting max rate information. Use setMaxRate first");
      return -1;
    }

  for(int i=1; i < (int) tp_.size() ; i++)
  {
    tc.coeff_.clear();
    dT = calculateMinimumTimeCubic(tp_[i-1],tp_[i]);
    tp_[i].time_ = tp_[i-1].time_ + dT;
    tc.duration_ = dT;

    for(int j=0; j < dimension_; j++)
    {
      temp[0] = tp_[i-1].q_[j];
      temp[1] = tp_[i-1].qdot_[j];
      temp[2] = (3*(tp_[i].q_[j]-tp_[i-1].q_[j])-(2*tp_[i-1].qdot_[j]+tp_[i].qdot_[j])*tc.duration_)/(tc.duration_*tc.duration_);
      temp[3] = (2*(tp_[i-1].q_[j]-tp_[i].q_[j])+(tp_[i-1].qdot_[j]+tp_[i].qdot_[j])*tc.duration_)/(pow(tc.duration_,3));

      tc.coeff_.push_back(temp);
    }
    tc_.push_back(tc);
  }
  return 1;
}


int Trajectory::minimizeSegmentTimesWithBlendedLinearInterpolation()
{
   double dT(0),acc(0.0),tb(0.0);
  TCoeff tc;

  std::vector<double> temp;

  temp.resize(5);

  tc.degree_ = 1;
  tc.dimension_ = dimension_;

  tc_.clear();

    if(max_rate_.empty() || (int) max_rate_.size() != dimension_ || max_acc_.empty() || (int) max_acc_.size() != dimension_)
    {
      ROS_WARN("Trying to apply rate and acc limits without setting them. Use setMaxRate and setMaxAcc first");
      return -1;
    }

  for(int i=1; i < (int) tp_.size() ; i++)
  {
    tc.coeff_.clear();
    dT = calculateMinimumTimeLSPB(tp_[i-1],tp_[i]);
    tp_[i].time_ = tp_[i-1].time_ + dT;
    tc.duration_ = dT;

    for(int j=0; j < dimension_; j++)
    {
       if(tp_[i].q_[j]-tp_[i-1].q_[j] > 0)
          acc = max_acc_[j];
       else
          acc = -max_acc_[j];
      tb =  blendTime(acc,-acc*tc.duration_,tp_[i].q_[j]-tp_[i-1].q_[j]);

      temp[0] = tp_[i-1].q_[j];
      temp[1] = 0;
      temp[2] = 0.5*acc;
      temp[3] = tb;
      temp[4] = std::max(tc.duration_-2*tb,0.0);
     
      tc.coeff_.push_back(temp);
    }
    tc_.push_back(tc);
  }
  return 1;
}

double Trajectory::blendTime(double aa,double bb,double cc)
{
   double disc = (pow(bb,2) - 4*aa*cc);
   if(disc < 0)
      return 0.0;

   double tb1 = (-bb + sqrt(disc))/(2*aa);
   double tb2 = (-bb - sqrt(disc))/(2*aa);
   if(isnan(tb1))
     tb1 = 0.0;
   if(isnan(tb2))
     tb2 = 0.0;
   return std::min(tb1,tb2);
}


void Trajectory::sampleLinear(TPoint &tp, double time, const TCoeff &tc, double segment_start_time)
{
  double segment_time = time - segment_start_time;
//  ROS_INFO("Coeff size: %d",tc.coeff_.size());
//  ROS_INFO("Coeff internal size: %d", tc.coeff_[0].size());
  for(int i =0; i < dimension_; i++)
  {
//    ROS_INFO("Coeffs: %f %f", tc.coeff_[i][0], tc.coeff_[i][1]);
    tp.q_[i]    =  tc.coeff_[i][0] + segment_time * tc.coeff_[i][1];
    tp.qdot_[i] =  tc.coeff_[i][1];
  }
  tp.time_ = time;
  tp.dimension_ = dimension_;
}


void Trajectory::sampleBlendedLinear(TPoint &tp, double time, const TCoeff &tc, double segment_start_time)
{
  double segment_time = time - segment_start_time;
  for(int i =0; i < dimension_; i++)
  {
    double taccend = tc.coeff_[i][3];
    double tvelend = tc.coeff_[i][3] + tc.coeff_[i][4];
    double tvel = tc.coeff_[i][4];
    double acc = tc.coeff_[i][2]*2;
    double v0 = tc.coeff_[i][1];
 
    if(segment_time <= taccend)
    {
      tp.q_[i]    =  tc.coeff_[i][0] + segment_time * v0 + 0.5 * segment_time * segment_time * acc;
      tp.qdot_[i] =  tc.coeff_[i][1] + segment_time * acc;
    }
    else if(segment_time >= tvelend)
    {
      double dT = segment_time - tvelend;
      tp.q_[i] = tc.coeff_[i][0] +  v0 * taccend + 0.5 * acc * taccend * taccend + acc * taccend * tvel + acc * taccend * dT - 0.5 * acc * dT * dT;
      tp.qdot_[i] = acc*taccend - acc*dT;
    }
    else
    {
      double dT = segment_time - taccend;
      tp.q_[i] = tc.coeff_[i][0] +  v0 * taccend + 0.5 * acc * taccend * taccend + acc * taccend * dT;
      tp.qdot_[i] = acc * taccend;
    }
  }
  tp.time_ = time;
  tp.dimension_ = dimension_;
}


void Trajectory::sampleCubic(TPoint &tp, double time, const TCoeff &tc, double segment_start_time)
{
  double segment_time = time - segment_start_time;
  for(int i =0; i < dimension_; i++)
  {
    tp.q_[i]    = tc.coeff_[i][0] + segment_time * tc.coeff_[i][1] + segment_time*segment_time*tc.coeff_[i][2] + segment_time*segment_time*segment_time*tc.coeff_[i][3];
    tp.qdot_[i] = tc.coeff_[i][1] + 2*segment_time*tc.coeff_[i][2] + 3*segment_time*segment_time*tc.coeff_[i][3];
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
  double minJointTime(MAX_ALLOWABLE_TIME);
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

double Trajectory::calculateMinimumTimeCubic(const TPoint &start, const TPoint &end)
{
  double minJointTime(MAX_ALLOWABLE_TIME);
  double minTime(0);

  for(int i = 0; i < start.dimension_; i++)
  {
    if(max_rate_[i] > 0)
      minJointTime = calculateMinTimeCubic(start.q_[i],end.q_[i],start.qdot_[i],end.qdot_[i],max_rate_[i]);
    else
      minJointTime = MAX_ALLOWABLE_TIME;

//    ROS_INFO("Min time: %f",minJointTime);

    if(minTime < minJointTime)
      minTime = minJointTime;

  }
  return minTime;
}

double Trajectory::calculateMinTimeCubic(double q0, double q1, double v0, double v1, double vmax)
{
  double t1(MAX_ALLOWABLE_TIME), t2(MAX_ALLOWABLE_TIME), result(MAX_ALLOWABLE_TIME);
  double dq = q1 - q0;
  double v(0.0);
  if(dq > 0)
     v = vmax;
  else
     v = -vmax;

  double a = 3.0*(v0+v1)*v - 3.0* (v0+v1)*v0 + pow((2.0*v0+v1),2.0);
  double b = -6.0*dq*v + 6.0 * v0 *dq - 6.0*dq*(2.0*v0+v1);
  double c = 9.0 * pow(dq,2);

  if (fabs(a) > EPS_TRAJECTORY)
  {
    if((pow(b,2)-4.0*a*c) >= 0)
    {
      t1 = (-b + sqrt(pow(b,2)-4.0*a*c))/(2.0*a);
      t2 = (-b - sqrt(pow(b,2)-4.0*a*c))/(2.0*a);
    }
  }
  else
  {
    t1 = -c/b;
    t2 = t1;
  }

  if(t1 < 0)
    t1 = MAX_ALLOWABLE_TIME;

  if(t2 < 0)
    t2 = MAX_ALLOWABLE_TIME;

  result = std::min(t1,t2);
  return result;
}

double Trajectory::calculateMinimumTimeLSPB(const TPoint &start, const TPoint &end)
{
  double minJointTime(MAX_ALLOWABLE_TIME);
  double minTime(0);

  for(int i = 0; i < start.dimension_; i++)
  {
    if(max_rate_[i] > 0)
      minJointTime = calculateMinTimeLSPB(start.q_[i],end.q_[i],max_rate_[i],max_acc_[i]);
    else
      minJointTime = MAX_ALLOWABLE_TIME;

    if(minTime < minJointTime)
      minTime = minJointTime;

  }

  return minTime;
}

double Trajectory::calculateMinTimeLSPB(double q0, double q1, double vmax, double amax)
{
  double tb = std::min(fabs(vmax/amax),sqrt(fabs(q1-q0)/amax));
  double acc(0);
  if((q1-q0)>0)
    acc = amax;
  else
    acc = -amax;
  double dist_tb = acc*tb*tb;
  double ts = (q1-q0 - dist_tb)/(acc*tb);
  if(ts < 0)
    ts = 0;
  return (2*tb+ts);
}


void Trajectory::setInterpolationMethod(std::string interp_method)
{
  interp_method_ = interp_method;
  ROS_INFO("Trajectory:: interpolation type %s",interp_method_.c_str());
}

int Trajectory::parameterize()
{
  int error_code = -1;
  if(interp_method_ == std::string("linear"))
     error_code = parameterizeLinear();
  else if(interp_method_ == std::string("cubic"))
     error_code = parameterizeCubic();
  else if(interp_method_ == std::string("blended_linear"))
     error_code = parameterizeBlendedLinear();
  else
  {
    ROS_WARN("Unrecognized interp_method type: %s\n",interp_method_.c_str());
  }
  return error_code;
}


int Trajectory::parameterizeLinear()
{
  double dT(0);
  TCoeff tc;

  std::vector<double> temp;

  temp.resize(2);

  tc.degree_ = 1;
  tc.dimension_ = dimension_;
  tc_.clear();

  if(autocalc_timing_)
  {
    if(max_rate_.empty() || (int) max_rate_.size() < 1)
    {
      ROS_WARN("Trying to apply rate limits without setting max rate information. Use setMaxRate first.");
      return -1;
    }
  }
  for(int i=1; i < (int) tp_.size() ; i++)
  {
    tc.coeff_.clear();
    dT = tp_[i].time_ - tp_[i-1].time_;
    if(autocalc_timing_) 
    {
      double dTMin = calculateMinimumTimeLinear(tp_[i-1],tp_[i]);
      if(dTMin > dT) // if minimum time required to satisfy limits is greater than time available, stretch this segment
        dT = dTMin;
    }

    tc.duration_ = dT;

    for(int j=0; j < dimension_; j++)
    {
      temp[0] = tp_[i-1].q_[j];
      temp[1] = (tp_[i].q_[j] - tp_[i-1].q_[j])/tc.duration_;  
      if(isnan(temp[1]))
        {
         temp[1] = 0.0;
//         ROS_WARN("Zero duration between two trajectory points");
        }
      tc.coeff_.push_back(temp);
    }
    tc_.push_back(tc);
  }

/*  for(int i=0; i<tc_.size(); i++)
  {
    for(int j=0; j<dimension_; j++)
      ROS_INFO("stored coeff: %d %d %f %f",i,j,tc_[i].coeff_[j][0],tc_[i].coeff_[j][1]);
  }
*/
  // Now modify all the times to bring them up to date
  for(int i=1; i < (int) tp_.size(); i++)
  {
    tp_[i].time_ = tp_[i-1].time_ + tc_[i-1].duration_;
//    ROS_INFO("Times: %d %f",i,tp_[i].time_);
  }
  return 1;
}


int Trajectory::parameterizeCubic()
{
  double dT(0);
  TCoeff tc;

  std::vector<double> temp;

  temp.resize(4);

  tc.degree_ = 1;
  tc.dimension_ = dimension_;
  tc_.clear();

  if(autocalc_timing_)
  {
    if(max_rate_.empty() || (int) max_rate_.size() < 1)
    {
      ROS_WARN("Trying to apply rate limits without setting max rate information. Use setMaxRate first.");
      return -1;
    }
  }

  for(int i=1; i < (int) tp_.size() ; i++)
  {
    tc.coeff_.clear();
    dT = tp_[i].time_ - tp_[i-1].time_;
    if(autocalc_timing_) 
    {
      double dTMin = calculateMinimumTimeCubic(tp_[i-1],tp_[i]);
      if(dTMin > dT) // if minimum time required to satisfy limits is greater than time available, stretch this segment
        dT = dTMin;
    }

    tc.duration_ = dT;

    for(int j=0; j < dimension_; j++)
    {
      temp[0] = tp_[i-1].q_[j];
      temp[1] = tp_[i-1].qdot_[j];
      temp[2] = (3*(tp_[i].q_[j]-tp_[i-1].q_[j])-(2*tp_[i-1].qdot_[j]+tp_[i].qdot_[j])*tc.duration_)/(tc.duration_*tc.duration_);
      temp[3] = (2*(tp_[i-1].q_[j]-tp_[i].q_[j])+(tp_[i-1].qdot_[j]+tp_[i].qdot_[j])*tc.duration_)/(pow(tc.duration_,3));
      if(isnan(temp[2]))
        temp[2] = 0.0;
      if(isnan(temp[3]))
        temp[3] = 0.0;

      tc.coeff_.push_back(temp);
    }
    tc_.push_back(tc);
  }

  // Now modify all the times to bring them up to date
  for(int i=1; i < (int) tp_.size(); i++)
    tp_[i].time_ = tp_[i-1].time_ + tc_[i-1].duration_;

  return 1;
}


int Trajectory::parameterizeBlendedLinear()
{
   double dT(0.0),acc(0.0),tb(0.0);
  TCoeff tc;

  std::vector<double> temp;

  temp.resize(5);

  tc.degree_ = 1;
  tc.dimension_ = dimension_;
  tc_.clear();

  if(autocalc_timing_)
  {
    if(max_rate_.empty() || (int) max_rate_.size() != dimension_ || max_acc_.empty() || (int) max_acc_.size() != dimension_)
    {
      ROS_WARN("Trying to apply rate and acc limits without setting max rate or acc information. Use setMaxRate and setMaxAcc first.");
      return -1;
    }
  }

  for(int i=1; i < (int) tp_.size() ; i++)
  {
    tc.coeff_.clear();
    dT = tp_[i].time_ - tp_[i-1].time_;
    if(autocalc_timing_) 
    {
      double dTMin = calculateMinimumTimeLSPB(tp_[i-1],tp_[i]);
      if(dTMin > dT)
        dT = dTMin;
    }

    tc.duration_ = dT;

    for(int j=0; j < dimension_; j++)
    {
       if(tp_[i].q_[j]-tp_[i-1].q_[j] > 0)
          acc = max_acc_[j];
       else
          acc = -max_acc_[j];

      tb =  blendTime(acc,-acc*tc.duration_,tp_[i].q_[j]-tp_[i-1].q_[j]);

      temp[0] = tp_[i-1].q_[j];
      temp[1] = 0;
      temp[2] = 0.5*acc;
      temp[3] = tb;
      temp[4] = std::max(tc.duration_-2*tb,0.0);

      //ROS_DEBUG("coeff: %d %d %f %f %f %f %f %f\n", i,j,tc.duration_,temp[0],temp[1],temp[2],temp[3],temp[4]);
      tc.coeff_.push_back(temp);
    }
    tc_.push_back(tc);
  }

  // Now modify all the times to bring them up to date
  for(int i=1; i < (int) tp_.size(); i++)
    tp_[i].time_ = tp_[i-1].time_ + tc_[i-1].duration_;

  return 1;
}

int Trajectory::write(std::string filename, double dT)
{
  FILE *f = fopen(filename.c_str(),"w");
  double time = tp_.front().time_;
  TPoint tp;
  tp.setDimension(dimension_);

  while(time < tp_.back().time_)
  {
    sample(tp,time);
    fprintf(f,"%f ",time);
    for(int j=0; j < dimension_; j++)
    {
      fprintf(f,"%f ",tp.q_[j]);
    }
    for(int j=0; j < dimension_; j++)
    {
      fprintf(f,"%f ",tp.qdot_[j]);
    }
    fprintf(f,"\n");
    time += dT;
//    printf("%f \n",time);
  }
  fclose(f);
  return 1;
}
