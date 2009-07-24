#ifndef __MEANSHIFT_GENERIC_H__
#define __MEANSHIFT_GENERIC_H__
/*********************************************************************
 * Software License Agreement (LGPL License)
 *
 *  Copyright (c) 2009, Willow Garage
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License version 3 as published by the Free Software Foundation.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110
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

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <vector>

#include <ANN/ANN.h>

// --------------------------------------------------------------
/*!
 * \brief The namespace for clustering algorithms
 */
// --------------------------------------------------------------
namespace clustering
{
  // --------------------------------------------------------------
  /*!
   * \brief Performs mean-shift clustering on data in arbitrary feature space
   *
   * Mean-shift algorithm described in:
   * [1] K. Fukunaga, L. D. Hostetler, "The Estimation of the Gradient of a
   * Density Function, with Applications in Pattern Recognition", IEEE T-IT 1975
   * [2] Y. Cheng, "Mean-shift, mode seeking, and clustering", IEEE T-PAMI 1995
   * [3] D. Comaniciu, P. Meer, "Mean Shift: A Robust Approach Toward Feature
   * Space Analysis", IEEE T-PAMI 2002
   *
   * The radius or bandwidth is tied to the 'width' of the distribution and is
   * data dependent.  Note that the data should be normalized first so that
   * all the dimensions have the same bandwidth.  The rate determines how
   * large the gradient decent steps are.  The smaller the rate, the more
   * iterations are needed for convergence, but the more likely minima are not
   * overshot.  A reasonable value for the rate is .2.  Low value of the rate
   * may require an increase in maxIter.  Increase maxIter until convergence
   * occurs regularly for a given data set (versus the algorithm being cut off
   * at maxIter).
   *
   * \param data    p x n column matrix of data points
   * \param p       dimension of data points
   * \param n       number of data points
   * \param radius  radius of search window
   * \param rate    gradient descent proportionality factor
   * \param maxIter max allowed number of iterations
   * \param labels  vector of length n with cluster label for each point
   *
   * \return 0 on success, otherwise negative value on error
   */
  // --------------------------------------------------------------
  void meanshiftGeneric(double *data,
                        int p,
                        int n,
                        double radius,
                        double rate,
                        int maxIter,
                        std::vector<double>& labels);
}

#endif
