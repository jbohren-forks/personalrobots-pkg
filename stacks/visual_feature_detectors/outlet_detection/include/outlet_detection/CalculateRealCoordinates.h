#ifndef CalculateRealCoordinates_H
#define CalculateRealCoordinates_H

#ifdef WIN32
#include "cv.h"
#include "highgui.h"
#else
#include "opencv/cv.h"
#include "opencv/highgui.h"
#endif
#include <stdio.h>
#include <ctype.h>


// INPUT: - Image with chessboard and points in it
//		  - Camera matrix
//	      - Distortion coefficients
//		  -	Chessboard inner corners count
//	      - Chessboard square width and height (in millimeters)
//		  - Image coordinates of outlets' holes with subpixel accuracy (CvPoint2D32f)
//		  - Number of points 
//		  - MAX_REPROJECTION_ERROR for chessboard corners (4 corners checking)
// OUTPUT: Array (CvPoint2D32f) with relative holes coordinates in millimeters. The zero is first point
//			[Optional] pointsCameraCoords - address of array of outlets points in camera coordinate system	
CvPoint2D32f* CalculateRealCoordinates(const IplImage* img, const CvMat* cameraMatrix, const CvMat* distortionCoeffs,
									   CvSize innerCornersCount, float squareWidth, float squareHeight,
									   const CvPoint2D32f* imagePoints, int pointsCount, float maxReprojectionError=0,
									   CvPoint3D32f** pointsCameraCoords = NULL);
#endif