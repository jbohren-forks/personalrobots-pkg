/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/

/** \author Mrinal Kalakrishnan */


#ifndef SPLINE_SMOOTHER_UTILS_H_
#define SPLINE_SMOOTHER_UTILS_H_

namespace spline_smoother
{

template <typename T>
void differentiate(const std::vector<T>& x, std::vector<T>& xd);

/**
 * \brief Solves the tridiagonal system of equations, Ax = d
 * A is an n by n square matrix which consists of:
 *      diagonal b (0 ... n-1)
 *      upper diagonal c (0 ... n-2)
 *      lower diagonal a (0 ... n-1)
 *
 * The solution goes into x. Time complexity: O(n)
 *
 * WARNING: modifies input arrays!!
 */
template <typename T>
void tridiagonalSolve(std::vector<T>& a,
    std::vector<T>& b,
    std::vector<T>& c,
    std::vector<T>& d,
    std::vector<T>& x);

/////////////////////////// inline implementations follow //////////////////////////////

template <typename T>
inline void differentiate(const std::vector<T>& x, std::vector<T>& xd)
{
  int size = x.size();
  xd.resize(size-1);
  for (int i=0; i<size-1; ++i)
  {
    xd[i] = x[i+1] - x[i];
  }
}

template <typename T>
void tridiagonalSolve(std::vector<T>& a,
    std::vector<T>& b,
    std::vector<T>& c,
    std::vector<T>& d,
    std::vector<T>& x)
{
  int n = (int)d.size();

  x.resize(n);

  // forward elimination
  for (int i=1; i<n; i++)
  {
    double m = a[i] / b[i-1];
    b[i] -= m*c[i-1];
    d[i] -= m*d[i-1];
  }

  // backward substitution
  x[n-1] = d[n-1]/b[n-1];
  for (int i=n-2; i>=0; i--)
  {
    x[i] = (d[i] - c[i]*x[i+1])/b[i];
  }
}

}

#endif /* SPLINE_SMOOTHER_UTILS_H_ */
