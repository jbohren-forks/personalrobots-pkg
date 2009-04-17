/*
 *  outlet_tuple.cpp
 *  rectify_outlets
 *
 *  Created by Victor  Eruhimov on 1/25/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include <vector>
#include <algorithm>
using namespace std;

#include "outlet_detection/outlet_tuple_coarse.h"
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



bool test_contours(CvSeq* c1, CvSeq* c2)
{
	CvRect rect1 = cvBoundingRect(c1);
	CvRect rect2 = cvBoundingRect(c2);
	// test the pair
	CvPoint center1 = rect_center(rect1);
	CvPoint center2 = rect_center(rect2);

	// check to see if more than 30% difference in width or height
	if(2.0f*(rect1.width - rect2.width)/(rect1.width + rect2.width) > 0.3f ||
	   2.0f*(rect1.height - rect2.height)/(rect1.height + rect2.height) > 0.3f)
	{
		return false;
	}

	float dist = sqrt(float(center1.x - center2.x)*(center1.x - center2.x) +
					  (center1.y - center2.y)*(center1.y - center2.y));
	if(dist > 2*rect1.width || dist < 0.5*rect1.width)
	{
		return false;
	}

	return true;
}



float sqdist(CvPoint2D32f p1, CvPoint2D32f p2)
{
	return ((p1.x-p2.x)*(p1.x-p2.x))+((p1.y-p2.y)*(p1.y-p2.y));
}


int aprox_equal(int a, int b, float eps)
{
	if (a<b) {
		swap(a,b);
	}

	if (b==0 && a!=0) {
		return false;
	}

	if (((float)(a-b))/b > eps) {
		return false;
	}
	return true;
}

//#define _SHOW_CONTOURS
//#define TUPLE_VERBOUSE

bool find_outlet_centroids(IplImage* img, CvPoint2D32f* centers, int use_edge_detection)
{
	IplImage* grey = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	IplImage* binary = cvCloneImage(grey);
	if (img->nChannels>1) {
		cvCvtColor(img,grey,CV_RGB2GRAY);
	}
	else {
		cvCopy(img,grey);
	}

	if (use_edge_detection) {
		cvCanny(grey, binary,20,40);
	}
	else {
		cvAdaptiveThreshold(grey, binary, 255, CV_ADAPTIVE_THRESH_MEAN_C,
				CV_THRESH_BINARY_INV, 11, 0);
	}
	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first = 0;
	cvFindContours(binary, storage, &first, sizeof(CvContour), CV_RETR_LIST);
	vector<CvSeq*> candidates;



	int width = img->width;
	int height = img->height;

	int max_width = 0.4*width;
	int min_width = 0.19*width;

	int max_height = 0.4*height;
	int min_height = 0.19*height;

#ifdef _SHOW_CONTOURS
	IplImage* img1 = cvCloneImage(img);
 	IplImage* mask = cvCloneImage(grey);
#endif
	for(CvSeq* seq = first; seq != NULL; seq = seq->h_next)
	{
#ifdef _SHOW_CONTOURS
		cvDrawContours(img1, seq, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0);
		cvNamedWindow("1", 0);
		cvShowImage("1", img1);
		cvWaitKey(0);
#endif


		CvRect rect = cvBoundingRect(seq);
		if(rect.width < min_width || rect.width > max_width || rect.height < min_height || rect.height > max_height)
		{
			continue;
		}

		candidates.push_back(seq);
#ifdef _SHOW_CONTOURS
		cvDrawContours(img1, seq, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0);
		cvNamedWindow("1", 0);
		cvShowImage("1", img1);
		cvWaitKey(0);
#endif
	}
#ifdef _SHOW_CONTOURS
//	cvNamedWindow("1", 1);
//	cvShowImage("1", img1);
//	cvWaitKey(0);
	cvReleaseImage(&img1);
#endif

	if (candidates.size()<4) {
#ifdef TUPLE_VERBOUSE
		printf("Cannot find 4 outlet blobs\n");
#endif
		cvReleaseMemStorage(&storage);
		cvReleaseImage(&binary);
		cvReleaseImage(&grey);
		return false;
	}

	int tuple[4];
	int tuple_count = 0;
	int prev_count = 0;
	int i_count = 0;
	vector<int> votes(candidates.size());
	size_t i;
	for(i = 0; i < candidates.size() && tuple_count < 3; i++)
	{
		i_count = 0;
		prev_count = tuple_count;
		for(size_t j = i+1; j < candidates.size() && tuple_count < 3; j++)
		{
			if (find(tuple,tuple+3, j)!=(tuple+3)) {
				continue;
			}
			if (!test_contours(candidates[i],candidates[j])) {
				continue;
			}
			// found pair, add j to the tuple


			tuple[tuple_count] = j;
			tuple_count++;
			i_count++;
		}
		if (i_count<2 && tuple_count<3) {
			i_count = 0;
			tuple_count = prev_count;
		}
	}
	if (tuple_count<3) {
#ifdef TUPLE_VERBOUSE
		printf("Cannot find 4 outlet blobs after filtering\n");
#endif
		cvReleaseMemStorage(&storage);
		cvReleaseImage(&binary);
		cvReleaseImage(&grey);
		return false;
	}

	bool tuple_found = false;
	for (int i=0;i<candidates.size();++i) {

		if (i==tuple[0] || i==tuple[1] || i==tuple[2]) continue;

		for (int j=0;j<3;++j) {
			if (test_contours(candidates[i], candidates[tuple[j]])) {
				tuple_found = true;
				tuple[3] = j;
				break;
			}
		}
		if (tuple_found) break;
	}

#if 0
	for(int j = 0; j < 4; j++)
	{
		cvDrawContours(img, candidates[tuple[j]], CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0);
	}
#endif
	// found the tuple!
	for(int j = 0; j < 4; j++)
	{
		centers[j] = calc_center(candidates[tuple[j]]);
	}

	order_tuple(centers);



#if 0
	cvCircle(img, cvPoint(centers[0]), 10, CV_RGB(0, 255, 0));
	cvCircle(img, cvPoint(centers[1]), 10, CV_RGB(0, 0, 255));
	cvCircle(img, cvPoint(centers[2]), 10, CV_RGB(255, 255, 255));
	cvCircle(img, cvPoint(centers[3]), 10, CV_RGB(0, 255, 255));
	cvNamedWindow("1", 0);
	cvShowImage("1", img);
	cvWaitKey(0);
#endif

//
//	int e1 = sqdist(centers[0], centers[1]);
//	int e2 = sqdist(centers[1], centers[2]);
//	int e3 = sqdist(centers[2], centers[3]);
//	int e4 = sqdist(centers[3], centers[0]);
//
//
//	if (!aprox_equal(e2,e4,1)) {
//#ifdef TUPLE_VERBOUSE
//		printf("Failed sides dimensions check\n");
//#endif
//		cvReleaseMemStorage(&storage);
//		cvReleaseImage(&binary);
//		cvReleaseImage(&grey);
//		return false;
//	}
//	if (!aprox_equal(e1,e3,1)) {
//#ifdef TUPLE_VERBOUSE
//		printf("Failed sides dimensions check\n");
//#endif
//		cvReleaseMemStorage(&storage);
//		cvReleaseImage(&binary);
//		cvReleaseImage(&grey);
//		return false;
//	}


	cvReleaseMemStorage(&storage);
	cvReleaseImage(&binary);
	cvReleaseImage(&grey);

	return true;

}

void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix)
{
	CvPoint2D32f rectified[4];
	rectified[0] = centers[0];
	rectified[1] = cvPoint2D32f(centers[0].x, centers[0].y);
	rectified[2] = cvPoint2D32f(centers[0].x, centers[3].y);
	rectified[3] = cvPoint2D32f(centers[1].x, centers[3].y);
	cvGetPerspectiveTransform(centers, rectified, map_matrix);
}
