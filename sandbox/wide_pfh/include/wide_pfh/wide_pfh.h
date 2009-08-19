/*********************************************************************
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
 *********************************************************************/


#ifndef WIDE_PFH_H_
#define WIDE_PFH_H_

#include <vector>

struct CvMat;
struct CvRect;

/**
* \brief Compute the Wide (large, sparse) PFH (WPFH) feature for a point cloud grid
* @param xyzd_  Pointer to a 2D float CvMat grid of X,Y,Z,D channels where D = 0 => invalid point
* @param xyzn    Pointer to same size and type matrix as xyzd_. Dense normals will be computed here.
* @param wpfh    Return: Will fill this with a 512 long histogram of alpha*8^0 + phi*8^1 + theta*8^2 entries
* @param roi       CvRect. If set, use only that part of xyzd. If NULL (Default), use all xyzd
* @return: <=0 => error, else good
*/
int calcWPFH(const CvMat *xyzd_, CvMat *xyzn, std::vector<float> &wpfh, CvRect *roi = NULL);



/**
 * \brief Compare 2 vector histograms together by Min intersect, Bhattacharrya or Chi Sqr (DEFAULT)
 * @param H1           vector Histogram 1
 * @param H2           vector Histogram 2
 * @param method    0 Min Intersection, 1 Bhattacharrya, 2 Chi Sqr (DEFAULT)
 * @return    Score, low is better. > 1.0 => error
 */
float minH4(std::vector<float> &H1, std::vector<float> &H2, int method = 2);




#endif /* WIDE_PFH_H_ */
