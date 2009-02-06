/*
 *  outlet_tuple.cpp
 *  rectify_outlets
 *
 *  Created by Victor  Eruhimov on 1/25/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include <vector>
using namespace std;

#include "outlet_tuple.h"
#include <highgui.h>


CvPoint2D32f calc_center(CvSeq* seq)
{
	CvMoments moments;
	cvMoments(seq, &moments);
	CvPoint2D32f center;
	center.x = cvGetSpatialMoment(&moments, 1, 0);
	center.y = cvGetSpatialMoment(&moments, 0, 1);
	float area = cvGetSpatialMoment(&moments, 0, 0);
	center.x /= area;
	center.y /= area;
	
	return center;
}

int find_dir(const CvPoint2D32f* dir, int xsign, int ysign)
{
	for(int i = 0; i < 4; i++)
	{
		if(dir[i].x*xsign > 0 && dir[i].y*ysign > 0)
		{
			return i;
		}
	}
	return -1;
}

void order_tuple(CvPoint2D32f* centers)
{
	CvPoint2D32f ordered[4];
	int idx[4];
	
	CvPoint2D32f center = cvPoint2D32f(0.0f, 0.0f);
	for(int i = 0; i < 4; i++)
	{
		center.x += centers[i].x;
		center.y += centers[i].y;
	}
	center.x *= 0.25f;
	center.y *= 0.25f;
	
	CvPoint2D32f dir[4];
	for(int i = 0; i < 4; i++)
	{
		dir[i].x = centers[i].x - center.x;
		dir[i].y = centers[i].y - center.y;
	}
	
	idx[0] = find_dir(dir, -1, -1);
	idx[1] = find_dir(dir, 1, -1);
	idx[2] = find_dir(dir, 1, 1);
	idx[3] = find_dir(dir, -1, 1);
	
	for(int i = 0; i < 4; i++)
	{
		ordered[i] = centers[idx[i]];
	}
	
	for(int i = 0; i < 4; i++)
	{
		centers[i] = ordered[i];
	}
	
}

void find_outlet_centroids(IplImage* img, CvPoint2D32f* centers)
{
	IplImage* grey = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	IplImage* binary = cvCloneImage(grey);
	cvCvtColor(img, grey, CV_RGB2GRAY);
	cvAdaptiveThreshold(grey, binary, 255, CV_ADAPTIVE_THRESH_MEAN_C, 
						CV_THRESH_BINARY_INV, 11, 0);
	
	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first = 0;
	cvFindContours(binary, storage, &first, sizeof(CvContour), CV_RETR_LIST);
	vector<CvSeq*> candidates;
	
	//IplImage* img1 = cvCloneImage(img);
	
	IplImage* mask = cvCloneImage(grey);
	for(CvSeq* seq = first; seq != NULL; seq = seq->h_next)
	{
		CvRect rect = cvBoundingRect(seq);
		if(rect.width < 30 || rect.width > 150 || rect.height < 15 || rect.height > 50)
		{
			continue;
		}
		
		//cvSetImageROI(img, rect);
		cvSetZero(mask);
		cvDrawContours(mask, seq, cvScalar(255), cvScalar(255), 0);
		
		CvScalar mean = cvAvg(img, mask);
		//cvResetImageROI(img);
		if(mean.val[2]/mean.val[1] < 1.0f)
		{
			continue;
		}
		
		candidates.push_back(seq);
		//		cvDrawContours(img1, seq, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0);
	}
	
#if 0
	cvNamedWindow("1", 1);
	cvShowImage("1", img1);
	cvWaitKey(0);
	cvReleaseImage(&img1);
#endif
	
	int tuple[3];
	int tuple_count = 0;
	int i = 0;
	for(i = 0; i < candidates.size() && tuple_count < 3; i++)
	{
		CvRect rect1 = cvBoundingRect(candidates[i]);
		tuple_count = 0;
		for(int j = 0; j < candidates.size() && tuple_count < 3; j++)
		{
			if(j <= i) continue;
			CvRect rect2 = cvBoundingRect(candidates[j]);
			
			// test the pair
			CvPoint center1 = rect_center(rect1);
			CvPoint center2 = rect_center(rect2);
			
			if(2.0f*(rect1.width - rect2.width)/(rect1.width + rect2.width) > 0.3f ||
			   2.0f*(rect1.height - rect2.height)/(rect1.height + rect2.height) > 0.3f)
			{
				continue;
			}
			
			float dist = sqrt(float(center1.x - center2.x)*(center1.x - center2.x) + 
							  (center1.y - center2.y)*(center1.y - center2.y));
			if(dist > 2*rect1.width)
			{
				continue;
			}
			
			// found pair, add rect2 to the tuple
			tuple[tuple_count] = j;
			tuple_count++;				
		}
	}
	
	i--;
	if(tuple_count == 3)
	{
#if 0
		cvDrawContours(img, candidates[i], CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0);
		for(int j = 0; j < 3; j++)
		{
			cvDrawContours(img, candidates[tuple[j]], CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0);
		}
#endif
		// found the tuple!
		centers[0] = calc_center(candidates[i]);
		for(int j = 0; j < 3; j++)
		{
			centers[j + 1] = calc_center(candidates[tuple[j]]);
		}
		
		order_tuple(centers);
		
#if 0
		cvCircle(img, cvPoint(centers[0]), 10, CV_RGB(0, 255, 0));
		cvCircle(img, cvPoint(centers[1]), 10, CV_RGB(0, 0, 255));			
		cvCircle(img, cvPoint(centers[2]), 10, CV_RGB(255, 255, 255));		
		cvCircle(img, cvPoint(centers[3]), 10, CV_RGB(0, 255, 255));		
#endif
		
	}
	
#if 0
	cvNamedWindow("1", 1);
	cvShowImage("1", img);
	cvWaitKey(0);
#endif
	
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&binary);
	cvReleaseImage(&grey);
	
}

void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix)
{
	CvPoint2D32f rectified[4];
	rectified[0] = centers[0];
	rectified[1] = cvPoint2D32f(centers[0].x + 200, centers[0].y);
	rectified[2] = cvPoint2D32f(centers[0].x + 200, centers[0].y + 100);
	rectified[3] = cvPoint2D32f(centers[0].x, centers[0].y + 100);
	cvGetPerspectiveTransform(centers, rectified, map_matrix);
}
