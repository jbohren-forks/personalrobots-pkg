/*
 *  affine_transform.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_AFFINE_TRANSFORM_H)
#define _AFFINE_TRANSFORM_H

#include <vector>
using namespace std;

#include <cv.h>

#include "outlet_detection/features.h"

void FindAffineTransform(const vector<CvPoint>& p1, const vector<CvPoint>& p2, CvMat* affine);
void MapVectorAffine(const vector<CvPoint>& p1, vector<CvPoint>& p2, CvMat* transform);
void MapFeaturesAffine(const vector<feature_t>& features, vector<feature_t>& mapped_features, CvMat* transform);
float CalcAffineReprojectionError(const vector<CvPoint>& p1, const vector<CvPoint>& p2, CvMat* transform);


#endif //_AFFINE_TRANSFORM_H
