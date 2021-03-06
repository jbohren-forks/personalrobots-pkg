/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "tf_conversions/tf_eigen.h"

namespace tf {

  void VectorTFToEigen(const tf::Vector3& t, Eigen::Vector3d& k)
  {
    k(0) = t[0];
    k(1) = t[1];
    k(2) = t[2];
  };

  void RotationTFToEigen(const tf::Quaternion& t, Eigen::Quaterniond& k)
  {
    Eigen::Quaterniond m(t[3],t[0],t[1],t[2]);
    k = m;
  };

  void TransformTFToEigen(const tf::Transform &t, Eigen::Transform3d &k)
  {
    for(int i=0; i<3; i++)
    {
      k.matrix()(i,3) = t.getOrigin()[i];
      for(int j=0; j<3; j++)
      {
        k.matrix()(i,j) = t.getBasis()[i][j];
      }
    }
  };

  void TransformEigenToTF(const Eigen::Transform3d &k, tf::Transform &t)
  {
    t.setOrigin(tf::Vector3(k.matrix()(0,3), k.matrix()(1,3), k.matrix()(2,3)));
    t.setBasis(btMatrix3x3(k.matrix()(0,0), k.matrix()(0,1),k.matrix()(0,2),k.matrix()(1,0), k.matrix()(1,1),k.matrix()(1,2),k.matrix()(2,0), k.matrix()(2,1),k.matrix()(2,2)));
  };
}
