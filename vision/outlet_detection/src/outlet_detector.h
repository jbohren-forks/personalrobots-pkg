/*
 *  outlet_detector.h
 *  outlet_sample
 *
 *  Created by Victor  Eruhimov on 2/19/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_OUTLET_DETECTOR_H)
#define _OUTLET_DETECTOR_H

#include <vector>
using namespace std;

#include "outlet_model.h"


// detect_outlet_tuple: high-level function for detecting 4 orange outlets
// Input parameters:
//	src: input color image
//	intrinsic_matrix: camera matrix of intrinsic parameters
//	distortion_params: vector of distortion parameters
//	outlets: output vector of outlets
//	output_path, filename: optional path and filename for logging the results
// Return value: 1 in case of success (found 4 outlets), 0 otherwise.
int detect_outlet_tuple(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params, vector<outlet_t>& outlets, 
	const char* output_path = 0, const char* filename = 0);

#endif //_OUTLET_DETECTOR_H
