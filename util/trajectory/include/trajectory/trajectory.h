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

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <sstream>
#include <map>

#include <ros/node.h>

namespace trajectory
{

  class Trajectory
  {
    public:
  
    struct TPoint
    {

        TPoint() {}

        TPoint(int dimension){setDimension(dimension);};

        TPoint(const std::vector<double>& q, double time);
            
        std::vector<double> q_;

        std::vector<double> qdot_;

        double time_;

        int dimension_;

        friend class Trajectory;

        void setDimension(int dimension){
          dimension_ = dimension;
          q_.resize(dimension_);
          qdot_.resize(dimension_);
        }
    };

    struct TCoeff
    {

        TCoeff() {}

        inline double get_coefficient(int degree, int dim_index);

        private: 

        int degree_;

        int dimension_;

        double duration_;

        std::vector<std::vector<double> > coeff_;

        friend class Trajectory;
    };
  
    Trajectory(int dimension);

    virtual ~Trajectory() {}

    void clear();

    void addPoint(const TPoint);

    int setTrajectory(const std::vector<TPoint>& tp);

    int setTrajectory(const std::vector<double> &p, int numFrames);

    inline double getTotalTime();

    int sample(TPoint &tp, double time);

//  void sample(std::vector<TPoint> &tp, double dT);

//  void sample(std::vector<TPoint> &tp, double start_time, double end_time, double dT); 

//  std::vector<TPoint>& getPoints() const;

    void setInterpolationMethod(std::string interp_method);
 
    double calculateMinimumTimeLinear(const TPoint &start, const TPoint &end);

    int setMaxRate(std::vector<double> max_rate);

    bool autocalc_timing_;

    private:

    int num_points_;

    int dimension_;

    std::string interp_method_;

    std::vector<TPoint> tp_;

    std::vector<TCoeff> tc_;

    std::vector<double> max_limit_;

    std::vector<double> min_limit_;

    std::vector<double> max_rate_;

    std::vector<double> max_acc_;

    void calcCoeff(std::string interp_method, bool autocalc_timing);

    void calculateLinearCoeff(bool autocalc_timing);

    void sampleLinear(TPoint &tp, double time, const TCoeff &tc, double segment_start_time);

    inline int findTrajectorySegment(double time);

  };
}

#endif
