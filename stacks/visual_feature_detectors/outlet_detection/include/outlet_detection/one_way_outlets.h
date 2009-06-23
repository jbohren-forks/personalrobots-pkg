/*
 *  one_way_outlets.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_ONE_WAY_OUTLET)
#define _ONE_WAY_OUTLET

#include <vector>
using namespace std;

#include <cv.h>
#include "features.h"
#include "one_way_descriptor.h"

void detect_outlets_2x1_one_way(IplImage* img, const CvOneWayDescriptorBase* descriptors, 
                                vector<feature_t>& features, const char* output_path = 0, const char* output_filename = 0);


#endif //_ONE_WAY_OUTLET