/*
* Function for outlets size estimation
* Author: Alexey Latyshev
*/

#ifdef WIN32
#include "cv.h"
#include "highgui.h"
#else
#include "opencv/cv.h"
#include "opencv/highgui.h"
#endif
#include <stdio.h>
#include <ctype.h>
#include "outlet_detection/outlet_detector.h"
#include "outlet_detection/CalculateRealCoordinates.h"


// INPUT: - Image with chessboard and outlets in it
//		  - Camera matrix
//	      - Distortion coefficients
//		  -	Chessboard inner corners count
//	      - Chessboard square width and height (in millimeters)
//		  - Image coordinates of outlets' holes with subpixel accuracy (CvPoint2D32f)
//		  - Number of outlets 
//		  - Number of holes in every outlet (default=3)
// OUTPUT: Array (CvPoint2D32f) with relative holes coordinates in millimeters. The zero is center of upper-left outlet
//			[Optional] pointsCameraCoords - address of array of outlets points in camera coordinate system	

CvPoint2D32f* CalculateRealCoordinates(const IplImage* img, const CvMat* cameraMatrix, const CvMat* distortionCoeffs,
									   CvSize innerCornersCount, float squareWidth, float squareHeight,
									   const CvPoint2D32f* imagePoints, int pointsCount,float maxReprojectionError,
									   CvPoint3D32f** pointsCameraCoords/*,CvRNG rng=0*/)
{
	CvPoint2D32f* corners = new CvPoint2D32f[innerCornersCount.height*innerCornersCount.width];
	int count;

	//Finding chessboard corners
	if (cvFindChessboardCorners(img,innerCornersCount,corners,&count)==0)
	{
		delete[] corners;
		return NULL;
	}
	//Finding corners with subpixel accuracy
	IplImage* view_gray = cvCreateImage( cvGetSize(img), 8, 1 );
	cvCvtColor(img, view_gray, CV_BGR2GRAY );
	cvFindCornerSubPix( view_gray, corners, count, cvSize(11,11),cvSize(-1,-1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	cvReleaseImage( &view_gray );

	CvMat* oldPoints = cvCreateMat( pointsCount,1, CV_32FC2 );
	CvMat* newPoints = cvCreateMat( pointsCount,1, CV_32FC2 );

	CvMat* oldPointsChessboard = cvCreateMat( innerCornersCount.height*innerCornersCount.width,1, CV_32FC2 );
	CvMat* newPointsChessboard = cvCreateMat( innerCornersCount.height*innerCornersCount.width,1, CV_32FC2 );
	CvMat* realPointsChessboard = cvCreateMat( innerCornersCount.height*innerCornersCount.width,1, CV_32FC2 );

	for (int i=0;i<innerCornersCount.height*innerCornersCount.width;i++)
	{
		oldPointsChessboard->data.fl[2*i]=corners[i].x;
		oldPointsChessboard->data.fl[2*i+1]=corners[i].y;
		realPointsChessboard->data.fl[2*i] = (i%innerCornersCount.width)*squareWidth;
		realPointsChessboard->data.fl[2*i+1] = (i/innerCornersCount.width)*squareHeight;
	}

	for (int i=0;i<pointsCount;i++)
	{
		oldPoints->data.fl[2*i]=imagePoints[i].x;
		oldPoints->data.fl[2*i+1]=imagePoints[i].y;
	}

	CvMat* newCameraMatrix = cvCloneMat(cameraMatrix);
	newCameraMatrix->data.db[2] = (img->width - 1)*0.5;
	newCameraMatrix->data.db[5] = (img->height - 1)*0.5;

	cvUndistortPoints(oldPoints,newPoints,cameraMatrix,distortionCoeffs,0,cameraMatrix);
	cvUndistortPoints(oldPointsChessboard,newPointsChessboard,cameraMatrix,distortionCoeffs,0,cameraMatrix);

	CvPoint2D32f* relCoords = new CvPoint2D32f[pointsCount];

	CvMat* homography = cvCreateMat(3,3,CV_64F);
	cvFindHomography(newPointsChessboard,realPointsChessboard,homography);

	bool isLargeError = false;

	if (maxReprojectionError > 0)
	{
		CvPoint2D32f reprojectedCorners[4];
		// Calculating chessboard coordinates in chessboard coordinate system
		float z;
		z=newPointsChessboard->data.fl[0]*(homography->data.db[6])+newPointsChessboard->data.fl[1]*(homography->data.db[7])+(homography->data.db[8]);
		reprojectedCorners[0].x=(newPointsChessboard->data.fl[0]*(homography->data.db[0])+newPointsChessboard->data.fl[1]*(homography->data.db[1])+(homography->data.db[2]))/z;
		reprojectedCorners[0].y=(newPointsChessboard->data.fl[0]*(homography->data.db[3])+newPointsChessboard->data.fl[1]*(homography->data.db[4])+(homography->data.db[5]))/z;
		reprojectedCorners[0].x-=realPointsChessboard->data.fl[0];
		reprojectedCorners[0].y-=realPointsChessboard->data.fl[1];
		
		z=newPointsChessboard->data.fl[2*innerCornersCount.width-2]*(homography->data.db[6])+newPointsChessboard->data.fl[2*innerCornersCount.width-1]*(homography->data.db[7])+(homography->data.db[8]);
		reprojectedCorners[1].x=(newPointsChessboard->data.fl[2*innerCornersCount.width-2]*(homography->data.db[0])+newPointsChessboard->data.fl[2*innerCornersCount.width-1]*(homography->data.db[1])+(homography->data.db[2]))/z;
		reprojectedCorners[1].y=(newPointsChessboard->data.fl[2*innerCornersCount.width-2]*(homography->data.db[3])+newPointsChessboard->data.fl[2*innerCornersCount.width-1]*(homography->data.db[4])+(homography->data.db[5]))/z;
		reprojectedCorners[1].x-=realPointsChessboard->data.fl[2*innerCornersCount.width-2];
		reprojectedCorners[1].y-=realPointsChessboard->data.fl[2*innerCornersCount.width-1];

		z=newPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)]*(homography->data.db[6])+newPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)+1]*(homography->data.db[7])+(homography->data.db[8]);
		reprojectedCorners[2].x=(newPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)]*(homography->data.db[0])+newPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)+1]*(homography->data.db[1])+(homography->data.db[2]))/z;
		reprojectedCorners[2].y=(newPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)]*(homography->data.db[3])+newPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)+1]*(homography->data.db[4])+(homography->data.db[5]))/z;
		reprojectedCorners[2].x-=realPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)];
		reprojectedCorners[2].y-=realPointsChessboard->data.fl[2*innerCornersCount.width*(innerCornersCount.height-1)+1];

		z=newPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-2]*(homography->data.db[6])+newPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-1]*(homography->data.db[7])+(homography->data.db[8]);
		reprojectedCorners[3].x=(newPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-2]*(homography->data.db[0])+newPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-1]*(homography->data.db[1])+(homography->data.db[2]))/z;
		reprojectedCorners[3].y=(newPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-2]*(homography->data.db[3])+newPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-1]*(homography->data.db[4])+(homography->data.db[5]))/z;
		reprojectedCorners[3].x-=realPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-2];
		reprojectedCorners[3].y-=realPointsChessboard->data.fl[2*innerCornersCount.width*innerCornersCount.height-1];

		float dist = 0;
		for (int j=0;j<4;j++)
		{
			dist = sqrt(reprojectedCorners[j].x*reprojectedCorners[j].x+reprojectedCorners[j].y*reprojectedCorners[j].y);
			if (dist > maxReprojectionError)
			{
				isLargeError = true;
				break;
			}
		}
	}


	if (!isLargeError)
	{

		// Calculating outlets' holes coordinate in chessboard coordinate system
		for (int i=0;i<pointsCount;i++)
		{
			float z=newPoints->data.fl[2*i]*(homography->data.db[6])+newPoints->data.fl[2*i+1]*(homography->data.db[7])+(homography->data.db[8]);
			relCoords[i].x=(newPoints->data.fl[2*i]*(homography->data.db[0])+newPoints->data.fl[2*i+1]*(homography->data.db[1])+(homography->data.db[2]))/z;
			relCoords[i].y=(newPoints->data.fl[2*i]*(homography->data.db[3])+newPoints->data.fl[2*i+1]*(homography->data.db[4])+(homography->data.db[5]))/z;
		}

		// [Optional] Calculating outlets' holes coordinates in camera coordinate system
		if (pointsCameraCoords)
		{
			CvMat* image_points = cvCreateMat(2,innerCornersCount.height*innerCornersCount.width,CV_64FC1);
			CvMat* object_points = cvCreateMat(3,innerCornersCount.height*innerCornersCount.width,CV_64FC1);

			// Sets object points and image points
			for (int i=0; i< innerCornersCount.height;i++)
				for (int j=0; j < innerCornersCount.width;j++)
				{
					object_points->data.db[(i*innerCornersCount.width+j)]=j*squareWidth;
					object_points->data.db[(i*innerCornersCount.width+j)+innerCornersCount.width*innerCornersCount.height]=i*squareHeight;
					object_points->data.db[(i*innerCornersCount.width+j)+2*innerCornersCount.width*innerCornersCount.height]=0.0f;

					image_points->data.db[(i*innerCornersCount.width+j)]=corners[(i*innerCornersCount.width+j)].x;
					image_points->data.db[(i*innerCornersCount.width+j)+innerCornersCount.width*innerCornersCount.height]=corners[(i*innerCornersCount.width+j)].y;
				}

				CvMat* R = cvCreateMat(3, 3, CV_64FC1);
				CvMat* T = cvCreateMat(3, 1, CV_64FC1);
				CvMat* rotation_vector = cvCreateMat(3,1,CV_64FC1);
				//Calculating Exrinsic camera parameters
				cvFindExtrinsicCameraParams2(object_points,image_points,cameraMatrix, distortionCoeffs,rotation_vector,T);
				cvRodrigues2(rotation_vector,R);

				*pointsCameraCoords = new CvPoint3D32f[pointsCount];

				CvMat* point = cvCreateMat(3,1,CV_64FC1);

				for (int i=0; i<pointsCount; i++)
				{
					point->data.db[0] = relCoords[i].x;
					point->data.db[1] = relCoords[i].y;
					point->data.db[2] = 0.0f;

					cvGEMM(R,point,1.0f,T,1.0,point);

					(*pointsCameraCoords)[i].x = point->data.db[0];
					(*pointsCameraCoords)[i].y = point->data.db[1];
					(*pointsCameraCoords)[i].z = point->data.db[2];
				}

				cvReleaseMat(&point);
				cvReleaseMat(&rotation_vector);
				cvReleaseMat(&R);
				cvReleaseMat(&T);
				cvReleaseMat(&image_points);
				cvReleaseMat(&object_points);
		}


		// Recalculating coordinates in outlets coordinate system (zero is in the center of the first outlet)
		float zerox = relCoords[0].x;
		float zeroy = relCoords[0].y;
		/*float zerox = 0.0f;
		float zeroy = 0.0f;*/
		//for (int i=0;i<holesCount;i++)
		//{
		//	zerox+=relCoords[i].x;	
		//	zeroy+=relCoords[i].y;
		//}
	//zerox/=holesCount;
		//zeroy/=holesCount;

		for (int i=0;i<pointsCount;i++)
		{
			relCoords[i].x=(relCoords[i].x-zerox);
			relCoords[i].y=(relCoords[i].y-zeroy);
		}
	}

	delete[] corners;
	cvReleaseMat(&newPoints);
	cvReleaseMat(&oldPoints);
	cvReleaseMat(&newPointsChessboard);
	cvReleaseMat(&oldPointsChessboard);
	cvReleaseMat(&realPointsChessboard);
	cvReleaseMat(&homography);

	if (isLargeError)
		return NULL;

	return relCoords;
}
