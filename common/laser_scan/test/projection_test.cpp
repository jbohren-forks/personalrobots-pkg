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

#include <gtest/gtest.h>
#include <sys/time.h>

#include "laser_scan/laser_scan.h"
#include <math.h>


#include "angles/angles.h"

void test_getUnitVectors (float angle_min, float angle_max, float angle_increment)
{
  double tolerance = 1e-6;
  laser_scan::LaserProjection projector;
  
  
  const boost::numeric::ublas::matrix<double> & mat = projector.getUnitVectors(angle_min, angle_max, angle_increment);
  
  

  for (unsigned int i = 0; i < mat.size2(); i++)
  {
    EXPECT_NEAR(angles::normalize_angle(atan2(mat(1,i), mat(0,i))),
                angles::normalize_angle(angle_min + i * angle_increment),
                tolerance);
  }
}

TEST(laser_scan, getUnitVectors)
{
  test_getUnitVectors(0, M_PI, M_PI/2.0);
  test_getUnitVectors(-M_PI, M_PI, M_PI/100.0);
  test_getUnitVectors(M_PI, 2.0 * M_PI, M_PI/40.0);
}

/*TEST(laser_scan, projectLaser)
{
  EXPECT_TRUE(true);
}
*/


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
