/*
 *  pca_features.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/15/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_PCA_FEATURES_H)
#define _PCA_FEATURES_H

#include <cv.h>
#include "features.h"

void savePCAFeatures(const char* filename, CvMat* avg, CvMat* eigenvectors);
void calcPCAFeatures(vector<IplImage*>& patches, const char* filename);
void loadPCAFeatures(const char* path, vector<IplImage*>& patches);
void readPCAFeatures(const char* filename, CvMat** avg, CvMat** eigenvectors);
void eigenvector2image(CvMat* eigenvector, IplImage* img);


IplImage* loadImageRed(const char* filename);


#endif //_PCA_FEATURES_H