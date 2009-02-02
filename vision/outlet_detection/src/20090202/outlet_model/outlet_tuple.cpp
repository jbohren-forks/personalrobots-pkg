/*
 *  outlet_tuple.cpp
 *  rectify_outlets
 *
 *  Created by Victor  Eruhimov on 1/25/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#include <math.h>

#include <vector>
#include <algorithm>
using namespace std;

#include "outlet_tuple.h"
#include <highgui.h>

const float pi = 3.1415926f;

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

int order_tuple(CvPoint2D32f* centers)
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
	
	// check if any of the tuples are not found
	int found[4] = {-1,-1,-1,-1};
	int count_lost = 0;
	int idx_lost = 0;
	for(int i = 0; i < 4; i++) 
	{
		if(idx[i] != -1)
		{
			found[idx[i]] = 1;
		}
		else
		{
			idx_lost = i;
			count_lost++;
		}
	}
	
	if(count_lost > 1)
	{
		printf("%d outlets cannot be ordered, not enough for a tuple\n", count_lost);
		return 0;
	}
	
	for(int i = 0; i < 4; i++)
	{
		if(found[i] == -1)
		{
			idx[idx_lost] = i;
		}
	}
	
	for(int i = 0; i < 4; i++)
	{
		ordered[i] = centers[idx[i]];
	}
	
	for(int i = 0; i < 4; i++)
	{
		centers[i] = ordered[i];
	}
	
	return 1;
}

typedef struct 
{
	float angle;
	int idx;
} sort_helper_t;

bool helper_pred_greater(sort_helper_t h1, sort_helper_t h2)
{
	return h1.angle < h2.angle;
}
		
int order_tuple2(CvPoint2D32f* centers)
{
	CvPoint2D32f ordered[4];
	
	CvPoint2D32f center = cvPoint2D32f(0.0f, 0.0f);
	for(int i = 0; i < 4; i++)
	{
		center.x += centers[i].x;
		center.y += centers[i].y;
	}
	center.x *= 0.25f;
	center.y *= 0.25f;
	
	CvPoint2D32f dir[4];
	vector<sort_helper_t> helper_vec;
	for(int i = 0; i < 4; i++)
	{
		dir[i].x = centers[i].x - center.x;
		dir[i].y = centers[i].y - center.y;

		sort_helper_t helper;
		helper.angle = atan2(dir[i].y, dir[i].x);
		helper.idx = i;
		helper_vec.push_back(helper);
	}
	sort(helper_vec.begin(), helper_vec.end(), helper_pred_greater);
	
	int start_idx = -1;
	float min_angle_diff = 1e10;
	for(int i = 0; i < 4; i++)
	{
		float angle_diff = fabs(helper_vec[i].angle + 3*pi/4);
		if(angle_diff < min_angle_diff)
		{
			min_angle_diff = angle_diff;
			start_idx = i;
		}
	}
	
	for(int i = 0; i < 4; i++)
	{
		int _i = (i + start_idx)%4;
		ordered[i] = centers[helper_vec[_i].idx];
	}
	for(int i = 0; i < 4; i++)
	{
		centers[i] = ordered[i];
	}
	
	return 1;
}

int find_outlet_centroids(IplImage* img, CvPoint2D32f* centers)
{
	IplImage* grey = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	IplImage* binary = cvCloneImage(grey);
	cvCvtColor(img, grey, CV_RGB2GRAY);
	cvAdaptiveThreshold(grey, binary, 255, CV_ADAPTIVE_THRESH_MEAN_C, 
						CV_THRESH_BINARY_INV, 11, 0);
	cvErode(binary, binary, 0, 1);
	cvDilate(binary, binary, 0, 1);

#if 0
	cvNamedWindow("1", 1);
	cvShowImage("1", binary);
	cvWaitKey(0);
#endif
	
	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first = 0;
	cvFindContours(binary, storage, &first, sizeof(CvContour), CV_RETR_LIST);
	vector<CvSeq*> candidates;
	
//	IplImage* img1 = cvCloneImage(img);
	
	IplImage* mask = cvCloneImage(grey);
	for(CvSeq* seq = first; seq != NULL; seq = seq->h_next)
	{
		CvRect rect = cvBoundingRect(seq);
#if !defined(_OUTLET_HR)
		const int xmin = 30;
		const int xmax = 150;
		const int ymin = 15;
		const int ymax = 50;
		const int min_width = 10;
#else
		const int xmin = 50;
		const int xmax = 200;
		const int ymin = 30; 
		const int ymax = 80;
		const int min_width = 10;
#endif //OUTLET_HR
		
		if(rect.width < xmin || rect.width > xmax || rect.height < ymin || rect.height > ymax)
		{
			continue;
		}
		
		float area = fabs(cvContourArea(seq));
		float perimeter = fabs(cvArcLength(seq));

		if(area/perimeter < min_width)
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
	int ret = 1;
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
		
		ret = order_tuple2(centers);
		
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
	
	return tuple_count == 3 && ret == 1 ? 1 : 0;
	
}

const int outlet_width = 50;
const int outlet_height = 25;
void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix, CvMat* inverse_map_matrix)
{
	CvPoint2D32f rectified[4];
	
	rectified[0] = centers[0];
	rectified[1] = cvPoint2D32f(centers[0].x + outlet_width, centers[0].y);
	rectified[2] = cvPoint2D32f(centers[0].x + outlet_width, centers[0].y + outlet_height);
	rectified[3] = cvPoint2D32f(centers[0].x, centers[0].y + outlet_height);
	cvGetPerspectiveTransform(centers, rectified, map_matrix);
	
	if(inverse_map_matrix)
	{
		cvGetPerspectiveTransform(rectified, centers, inverse_map_matrix);
	}
}

void map_image_corners(CvSize src_size, CvMat* map_matrix, CvMat* corners, CvMat* dst)
{
	corners->data.fl[0] = 0;
	corners->data.fl[1] = 0;
	corners->data.fl[2] = src_size.width;
	corners->data.fl[3] = 0;
	corners->data.fl[4] = src_size.width;
	corners->data.fl[5] = src_size.height;
	corners->data.fl[6] = 0;
	corners->data.fl[7] = src_size.height;
	cvPerspectiveTransform(corners, dst, map_matrix);
}

//void calc_image_homography(const CvPoint2D32f* centers, CvSize src_size, CvMat** xmap, CvMat** ymap, CvSize* dst_size)
void calc_image_homography(const CvPoint2D32f* centers, CvSize src_size, CvMat* map_matrix, CvSize* dst_size)
{
//	CvMat* map_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* inverse_map_matrix = cvCreateMat(3, 3, CV_32FC1);
	calc_outlet_homography(centers, map_matrix, inverse_map_matrix);
	
	CvMat* corners = cvCreateMat(1, 4, CV_32FC2);
	CvMat* dst = cvCreateMat(1, 4, CV_32FC2);
	map_image_corners(src_size, map_matrix, corners, dst);

	float xmin = 1e10, ymin = 1e10, xmax = -1e10, ymax = -1e10;
	float src_xmin, src_xmax, src_ymin, src_ymax;
	for(int i = 0; i < 4; i++)
	{
		if(xmin > dst->data.fl[2*i])
		{
			xmin = dst->data.fl[2*i];
			src_ymin = corners->data.fl[2*i + 1];
		}
		if(xmax < dst->data.fl[2*i])
		{
			xmax = dst->data.fl[2*i];
//			src_xmax = corners->data.fl[2*i];
		}
		if(ymin > dst->data.fl[2*i + 1])
		{
			ymin = dst->data.fl[2*i + 1];
			src_xmin = corners->data.fl[2*i];
		}
		if(ymax < dst->data.fl[2*i + 1])
		{
			ymax = dst->data.fl[2*i + 1];
//			src_ymax = corners->data.fl[2*i + 1];
		}
	}
	
	dst_size->width = ceil(xmax - xmin) + 1;
	dst_size->height = ceil(ymax - ymin) + 1;
/*
	CvMat* src_points = cvCreateMat(1, dst_size->width*dst_size->height, CV_32FC2);
	CvMat* dst_points = cvCreateMat(1, dst_size->width*dst_size->height, CV_32FC2);
	for(int r = 0; r < dst_size->height; r++)
	{
		for(int c = 0; c < dst_size->width; c++)
		{
			dst_points->data.fl[r*dst_size->width + c*2] = c;// - xmin;
			dst_points->data.fl[r*dst_size->width + c*2 + 1] = r;// - ymin;
		}
	}
	cvPerspectiveTransform(dst_points, src_points, inverse_map_matrix);
	
	*xmap = cvCreateMat(dst_size->height, dst_size->width, CV_32FC1);
	*ymap = cvCreateMat(dst_size->height, dst_size->width, CV_32FC1);
	for(int r = 0; r < dst_size->height; r++)
	{
		for(int c = 0; c < dst_size->width; c++)
		{
			cvmSet(*xmap, r, c, src_points->data.fl[r*dst_size->width + c*2]);
			cvmSet(*ymap, r, c, src_points->data.fl[r*dst_size->width + c*2 + 1]);
		}
	}
			
	cvReleaseMat(&src_points);
	cvReleaseMat(&dst_points);
 */
	cvReleaseMat(&corners);
	cvReleaseMat(&dst);
}

int calc_image_homography(IplImage* src, CvMat* map_matrix, CvSize* dst_size, CvPoint2D32f* hor_dir, CvPoint3D32f* origin, 
						  CvPoint2D32f* scale)
{
	CvPoint2D32f centers[4];
	int ret = find_outlet_centroids(src, centers);
	if(!ret)
	{
		printf("Centroids not found\n");
		return 0;
	}
	
	if(hor_dir)
	{
		hor_dir->x = centers[1].x - centers[0].x;
		hor_dir->y = centers[1].y - centers[0].y;
	}
		
	calc_image_homography(centers, cvSize(src->width, src->height), map_matrix, dst_size);
	
	float scalex, scaley;
	if(origin)
	{
		CvMat* _src = cvCreateMat(1, 3, CV_32FC2);
		CvMat* _dst = cvCreateMat(1, 3, CV_32FC2);	
		
		_src->data.fl[0] = centers[0].x;
		_src->data.fl[1] = centers[0].y;
		_src->data.fl[2] = centers[1].x;
		_src->data.fl[3] = centers[1].y;
		_src->data.fl[4] = centers[2].x;
		_src->data.fl[5] = centers[2].y;
		
		cvPerspectiveTransform(_src, _dst, map_matrix);
		
		origin->x = _dst->data.fl[0];
		origin->y = _dst->data.fl[1];
		origin->z = 0;
		
		const float xsize = 44.67f;
		const float ysize = 38.7;
		
		scalex = xsize/(_dst->data.fl[2] - _dst->data.fl[0]);
		scaley = ysize/(_dst->data.fl[5] - _dst->data.fl[3]);
		
		cvReleaseMat(&_src);
		cvReleaseMat(&_dst);
	}
	
	if(scale)
	{
		*scale = cvPoint2D32f(scalex, scaley);
	}

	return 1;
}

IplImage* find_templates(IplImage* img, IplImage* templ)
{
	IplImage* templr = cvCreateImage(cvSize(outlet_width, outlet_height), IPL_DEPTH_8U, 3);
	cvResize(templ, templr);
	
	IplImage* dist = cvCreateImage(cvSize(img->width - templr->width + 1, img->height - templr->height + 1), IPL_DEPTH_32F, 1);
	cvMatchTemplate(img, templr, dist, CV_TM_SQDIFF_NORMED);
	
	double max_dist, min_dist;
	cvMinMaxLoc(dist, &min_dist, &max_dist);
	
	IplImage* mask = cvCreateImage(cvSize(dist->width, dist->height), IPL_DEPTH_8U, 1);
	
	double thresh = min_dist*1.3f;
	cvThreshold(dist, mask, thresh, 255, CV_THRESH_BINARY_INV);
	
	for(int r = 0; r < dist->height; r++)
	{
		for(int c = 0; c < dist->width; c++)
		{
			if(mask->imageData[r*mask->widthStep + c] == 0)
			{
				continue;
			}
			
			int color = (thresh - *(float*)(dist->imageData + r*dist->widthStep + c*sizeof(float)))/(thresh - min_dist)*255;
//			cvCircle(img, cvPoint(c + templr->width/2, r + templr->height/2), 20, CV_RGB(color, color, 0), 3);
//			cvRectangle(img, cvPoint(c, r), cvPoint(c + templr->width, r + templr->height), CV_RGB(color, color, 0), 2);
			cvRectangle(img, cvPoint(c, r), cvPoint(c + templr->width, r + templr->height), CV_RGB(0, 0, 255), 2);
		}
	}
	
	cvReleaseImage(&templr);

	return mask;
}