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

#ifndef VISUAL_NAV_TRANSFORM_H
#define VISUAL_NAV_TRANSFORM_H

#include <iostream>
#include <tf/transform_datatypes.h>

namespace visual_nav
{

using std::ostream;
using std::istream;

typedef tf::Stamped<tf::Pose> StampedPose;

struct Pose {
  Pose (double x=0.0, double y=0.0f, double theta=0.0) : x(x), y(y), theta(theta) {}
  Pose (const StampedPose& tf_pose);
  double x, y, theta;
};

bool operator== (const Pose& p1, const Pose& p2);

struct Transform2D {
  Transform2D(double dx=0.0, double dy=0.0, double theta=0.0) : dx(dx), dy(dy), theta(theta) {}
  double dx, dy, theta;
};


/// \return \a pose transformed by \a trans
Pose transform (const Transform2D& trans, const Pose& pose);

/// \return The unique transform that sends \a pose1 to \a pose2
Transform2D getTransformBetween (const Pose& pose1, const Pose& pose2);

/// \return Inverse of \a trans
Transform2D inverse (const Transform2D& trans);


ostream& operator<< (ostream& str, const Pose& c);
ostream& operator<< (ostream& str, const Transform2D& c);
istream& operator>> (istream& str, Pose& c);
istream& operator>> (istream& str, Transform2D& c);


} //namespace

#endif
