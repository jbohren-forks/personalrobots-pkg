//Created by Alexey Latyshev
// Set of functions for Generic Hough Transform (GHT) for outlets detection
#ifndef _G_HOUGH_H
#define _G_HOUGH_H

#include <cv.h>
#include <highgui.h>
#include "outlet_detection/features.h"
#include "outlet_detection/affine_transform.h"

// Builds 6-dimension histogram [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
CvSparseMat* buildHoughHist(vector<feature_t>& input, const vector<feature_t>& train_features, int* hist_size, float** ranges);
CvMatND* buildHoughHistSparse(vector<feature_t>& input, const vector<feature_t>& train_features, int* hist_size, float** ranges);

void releaseHistMat(CvMatND** hist, int* hist_size);

//// Calculates maximums of histogram. 
//// Returns one of the maximums
//float** getMaxHistValues(const CvSparseMat* hist, int* hist_size);
////Returns all maximums with >= MIN_VOTES
void getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float**& maxs, int& count, int MIN_VOTES);
////Returns all maximums (count number)
// Return value: max votes
int getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float**& maxs, int& count);

// Calculates maximums of histogram. 
//Returns all maximums with >= MIN_VOTES
void getMaxSparseHistValues(const CvMatND* hist, int* hist_size, float** ranges, float**& maxs, int& count, int MIN_VOTES);
//Returns all maximums (count number)
void getMaxSparseHistValues(const CvMatND* hist, int* hist_size, float** ranges, float**& maxs, int& count);

// Calculates outlet features from given train outlet and affine transform
// Affine transform is array [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
void calcOutletPosition(const vector<feature_t>& train_features, float* affine_transform, vector<feature_t>& features);

void calcExactLocation1(vector<feature_t>& features, vector<feature_t>& outlet);
void calcExactLocation2(vector<feature_t>& features, vector<feature_t>& outlet, int accuracy = 2);
//Calculates more accurate outlet position by following algorithm: for each outlet point we select the closest feature point and movement vector to this point
// Then we try to move outlet to the new location for each movement vector and calculate number of holes not far from the nearest feature point than accuracy  
// Than we select movement vector with the biggest number of votes and move outlet
// INPUT: features - all found features on the image
//		  outlet - test outlet for which we try to calculate exact location
// OUTPUT: outlet
void calcExactLocation3(vector<feature_t>& features, vector<feature_t>& outlet, int accuracy = 5);
//Calculates more accurate outlet position by following algorithm: for each outlet point we select the closest feature point and movement vector to this point
// Then we select group of approximately the same movement vectors and move all these outlet points to the new location
// Other oultet points are on the same place
// INPUT: features - all found features on the image
//		  outlet - test outlet for which we try to calculate exact location
// OUTPUT: outlet
void calcExactLocation4(vector<feature_t>& features, vector<feature_t>& outlet);
//Calculates more accurate outlet position by following algorithm: for each outlet point we select the closest feature point (with distance less than accuracy parameter otherwise we say that there is no the closest feature)
// Then we calculates affine transform between train features and selected features and apply found transform to the train outlet
// If we cannot do this then we clear outlet that means there is no outlet on the image
// INPUT: features - all found features on the image
//		  train_features - train outlet 
//		  src_outlet - test outlet for which we try to calculate exact location
//		  accuracy - max distance for feature selecting
// OUTPUT: dst_outlet
//		   reprojectionError - reprojection error for affine transform
void calcExactLocation(vector<feature_t>& features,const vector<feature_t>& train_features, vector<feature_t>& src_outlet, vector<feature_t>& dst_outlet, float& reprojectionError, int accuracy = 10);
void calcExactLocation_(vector<feature_t>& features,const vector<feature_t>& train_features, vector<feature_t>& src_outlet, vector<feature_t>& dst_outlet, float& reprojectionError, int accuracy = 10);

#endif