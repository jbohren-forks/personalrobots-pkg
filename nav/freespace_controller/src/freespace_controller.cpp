/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

//check the orientation of a pt c with respect to the vector a->b
// orient(a, b, c) < 0 -----> Left
// orient(a, b, c) > 0 -----> Right
double orient(const std_msgs::Point2DFloat32& a, const std_msgs::Point2DFloat32& b, const std_msgs::Point2DFloat32& c){
  double acx = a.x - c.x;
  double bcx = b.x - c.x;
  double acy = a.y - c.y;
  double bcy = b.y - c.y;
  return acx * bcy - acy * bcx;
}

//check if a point is in a polygon
//a point is in a polygon iff the orientation of the point
//with respect to sides of the polygon is the same for every
//side of the polygon
bool ptInPolygon(const std_msgs::Point2DFloat32& pt, const vector<std_msgs::Point2DFloat32>& poly){
  bool all_left = false;
  bool all_right = false
  for(unsigned int i = 0; i < poly.size() - 1; ++i){
    Point a = poly[i];
    Point b = poly[i + 1];
    //if pt left of a->b
    if(orient(a, c, pt) < 0){
      if(all_right)
        return false;
      all_left = true;
    }
    //if pt right of a->b
    else{
      if(all_left)
        return false;
      all_right = true;
    }
  }
  return true;
}
