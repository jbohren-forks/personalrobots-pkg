/*
 *  outlet_tuple.h
 *  rectify_outlets
 *
 *  Created by Victor  Eruhimov on 1/25/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_OUTLET_TUPLE_H)
#define _OUTLET_TUPLE_H

#include <cv.h>

inline CvPoint cvPoint(CvPoint2D32f point)
{
	return cvPoint(int(point.x), int(point.y));
}

inline CvPoint rect_center(CvRect rect)
{
	return cvPoint(rect.x + rect.width/2, rect.y + rect.height/2);
}

CvPoint2D32f calc_center(CvSeq* seq);
int find_dir(const CvPoint2D32f* dir, int xsign, int ysign);
void order_tuple(CvPoint2D32f* centers);
void find_outlet_centroids(IplImage* img, CvPoint2D32f* centers);
void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix);




#endif //_OUTLET_TUPLE_H