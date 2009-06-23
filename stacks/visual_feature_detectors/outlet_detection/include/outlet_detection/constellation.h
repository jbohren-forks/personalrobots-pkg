/*
 *  constellation.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/3/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_CONSTELLATION_H)
#define _CONSTELLATION_H

#include <vector>
using namespace std;

#include <cv.h>
#include "features.h"

float calc_set_std(const vector<feature_t>& features, const vector<int>& indices = vector<int>());
void DetectObjectConstellation(const vector<feature_t>& train, const vector<feature_t>& input, CvMat* homography, vector<int>& indices);
void InferMissingObjects(const vector<feature_t>& train, const vector<feature_t>& input, CvMat* homography, const vector<int>& indices, 
                         vector<feature_t>& full);
void FilterOutletFeatures(const vector<feature_t>& src_features, vector<feature_t>& dst_features, float max_dist);
void ClusterOutletFeatures(const vector<feature_t>& src_features, vector<feature_t>& clusters, float max_dist);
void SelectNeighborFeatures(const vector<feature_t>& src_features, CvPoint center, vector<feature_t>& dst_features, float max_dist);



#endif // _CONSTELLATION_H
