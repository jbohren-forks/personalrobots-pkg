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

#include <limits.h>
#include <math.h>
#include <stdio.h>

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

CvPoint2D32f calc_center(const vector<CvPoint2D32f>& points)
{
	CvPoint2D32f center = cvPoint2D32f(0, 0);
	for(unsigned int i = 0; i < points.size(); i++)
	{
		center.x += points[i].x;
		center.y += points[i].y;
	}
	
	center.x /= points.size();
	center.y /= points.size();
	
	return(center);
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

bool helper_pred_greater(outlet_elem_t h1, outlet_elem_t h2)
{
	return h1.angle < h2.angle;
}

int find_start_idx(const vector<outlet_elem_t>& helper_vec)
{
	int start_idx = -1;
	float min_angle_diff = 1e10;
	for(int i = 0; i < 4; i++)
	{
		float angle_diff = fabs(helper_vec[i].angle + 4*pi/5);
		if(angle_diff < min_angle_diff)
		{
			min_angle_diff = angle_diff;
			start_idx = i;
		}
	}
	
	return start_idx;
	
}


int find_start_idx2(const vector<outlet_elem_t>& helper_vec)
{
	for(int i = 0; i < 4; i++)
	{
		// find the fartherst point
		float max_dist = 0;
		int max_j = -1;
		for(int j = 0; j < 4; j++)
		{
			if(j == i) continue;
			
			float dist = length(helper_vec[i].center - helper_vec[j].center);
			if(dist > max_dist)
			{
				max_dist = dist;
				max_j = j;
			}			
		}
		
		if(helper_vec[i].center.x < helper_vec[max_j].center.x)
		{
			return i;
		}
	}
	
	assert(0);
	return -1; // should never reach this point
}
		
int find_start_idx3(const vector<outlet_elem_t>& helper_vec)
{
	const float mean_angle = -3*pi/4;
	float max_angle = pi/2*0.8f;;
	for(int i = 0; i < 4; i++)
	{
		float angle = helper_vec[i].angle - mean_angle;
		if(angle > pi) angle -= 2*pi;
		if(fabs(angle) > max_angle)
		{
			continue;
		}
		
		int prev_idx = (i + 3)%4;
		int post_idx = (i + 1)%4;
		CvPoint2D32f prev_vec = helper_vec[prev_idx].center - helper_vec[i].center;
		CvPoint2D32f post_vec = helper_vec[post_idx].center - helper_vec[i].center;
		float l1 = length(prev_vec);
		float l2 = length(post_vec);
		if(l2 > l1 && post_vec.x > 0)
		{
			return i;
		}
	}
	
	// should not get past this point
	return -1;
	
}

int order_tuple2(vector<outlet_elem_t>& tuple)
{
	vector<outlet_elem_t> ordered;
	
	CvPoint2D32f center = cvPoint2D32f(0.0f, 0.0f);
	for(int i = 0; i < 4; i++)
	{
		center.x += tuple[i].center.x;
		center.y += tuple[i].center.y;
	}
	center.x *= 0.25f;
	center.y *= 0.25f;
	
	CvPoint2D32f dir[4];
	for(int i = 0; i < 4; i++)
	{
		dir[i].x = tuple[i].center.x - center.x;
		dir[i].y = tuple[i].center.y - center.y;

		tuple[i].angle = atan2(dir[i].y, dir[i].x);
		tuple[i].idx = i;
	}
	sort(tuple.begin(), tuple.end(), helper_pred_greater);
	
#if 0
	int start_idx = find_start_idx(helper_vec);
#else
	int start_idx = find_start_idx3(tuple);
#endif
	
	ordered = tuple;
	for(int i = 0; i < 4; i++)
	{
		int _i = (i + start_idx)%4;
		ordered[i] = tuple[_i];
	}

	tuple = ordered;
	return 1;
}

int find_outlet_centroids(IplImage* img, outlet_tuple_t& outlet_tuple, const char* output_path, const char* filename)
{
	IplImage* grey = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
	IplImage* binary = cvCloneImage(grey);
	cvCvtColor(img, grey, CV_RGB2GRAY);
	cvSmooth(grey, grey);
	cvAdaptiveThreshold(grey, binary, 255, CV_ADAPTIVE_THRESH_MEAN_C, 
						CV_THRESH_BINARY_INV, 15, 0);
	cvErode(binary, binary, 0, 1);
	cvDilate(binary, binary, 0, 1);

#if defined(_VERBOSE_TUPLE)
	cvNamedWindow("1", 1);
	cvShowImage("1", binary);
	cvWaitKey(0);
	cvSaveImage("mask.jpg", binary);
#endif
	
	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first = 0;
	cvFindContours(binary, storage, &first, sizeof(CvContour), CV_RETR_CCOMP);
	vector<CvSeq*> candidates;

#if defined(_VERBOSE_TUPLE)
	IplImage* img1 = cvCloneImage(img);
#endif //_VERBOSE_TUPLE
	
	IplImage* mask = cvCloneImage(grey);
	for(CvSeq* seq = first; seq != NULL; seq = seq->h_next)
	{
		CvRect rect = cvBoundingRect(seq);
#if !defined(_OUTLET_HR)
		const int xmin = 30;
		const int xmax = 150;
		const int ymin = 15;
		const int ymax = 50;
		const int min_width = 5;
#else
		const int xmin = 50;
		const int xmax = 400;
		const int ymin = 30; 
		const int ymax = 400;
		const int min_width = 5;
#endif //OUTLET_HR
		
#if 0
		if(abs(rect.x - 1387) < 20 && abs(rect.y - 127) < 20)
		{
			int w = 1;
		}
#endif
		
		if(rect.width < xmin || rect.width > xmax || rect.height < ymin || rect.height > ymax)
		{
			continue;
		}
		
		float area = fabs(cvContourArea(seq));
		float perimeter = fabs(cvArcLength(seq));

		if(area/perimeter*2 < min_width)
		{
			continue;
		}
		
		//cvSetImageROI(img, rect);
		cvSetZero(mask);
		cvDrawContours(mask, seq, cvScalar(255), cvScalar(255), 0, CV_FILLED);
		
		CvScalar mean = cvAvg(img, mask);
		//cvResetImageROI(img);
		if(mean.val[2]/mean.val[1] < 2.0f)
		{
			continue;
		}
		
		candidates.push_back(seq);
		
#if defined(_VERBOSE_TUPLE)
		cvDrawContours(img1, seq, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0, 2);
#endif //_VERBOSE_TUPLE
	}
	
#if defined(_VERBOSE_TUPLE)
	cvNamedWindow("1", 1);
	cvShowImage("1", img1);
	cvWaitKey(0);
	cvReleaseImage(&img1);
#endif //_VERBOSE_TUPLE
	
	int found_tuple = 0;
	vector<outlet_elem_t> tuple;
	for(unsigned int i = 0; i < candidates.size(); i++)
	{
		vector<outlet_elem_t> tuple_candidates;
		outlet_elem_t outlet_elem;
		outlet_elem.seq = candidates[i];
		outlet_elem.center = calc_center(candidates[i]);
		tuple_candidates.push_back(outlet_elem);
		
		CvRect rect1 = cvBoundingRect(candidates[i]);
		for(unsigned int j = 0; j < candidates.size(); j++)
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
			if(dist > 4*rect1.width)
			{
				continue;
			}
			
			// found pair, add rect2 to the tuple
			outlet_elem_t outlet_elem;
			outlet_elem.seq = candidates[j];
			outlet_elem.center = calc_center(candidates[j]);
			tuple_candidates.push_back(outlet_elem);
		}
		
		// find the tuple
 		found_tuple = find_tuple(tuple_candidates, outlet_tuple.centers);
		if(found_tuple == 1)
		{
			// found the tuple!
			tuple = tuple_candidates;
			break;
		}
	}
	
#if defined(_VERBOSE_TUPLE) || defined(_VERBOSE)
	IplImage* img2 = cvCloneImage(img);
#endif //_VERBOSE_TUPLE
	
	if(found_tuple == 1)
	{
		// draw the mask
		if(outlet_tuple.tuple_mask)
		{
			cvSetZero(outlet_tuple.tuple_mask);
			for(int i = 0; i < 4; i++)
			{
				cvDrawContours(outlet_tuple.tuple_mask, tuple[i].seq, cvScalar(i + 1), cvScalar(i + 1), 0, CV_FILLED);
			}
		}
		
		// calculate the tuple roi
		CvRect tuple_roi[4];
		for(int i = 0; i < 4; i++)
		{
			tuple_roi[i] = cvBoundingRect(tuple[i].seq);
		}
		calc_bounding_rect(4, tuple_roi, outlet_tuple.roi);
		
#if defined(_VERBOSE_TUPLE) || defined(_VERBOSE)
		cvCircle(img2, cvPoint(outlet_tuple.centers[0]), 10, CV_RGB(0, 255, 0));
		cvCircle(img2, cvPoint(outlet_tuple.centers[1]), 10, CV_RGB(0, 0, 255));			
		cvCircle(img2, cvPoint(outlet_tuple.centers[2]), 10, CV_RGB(255, 255, 255));		
		cvCircle(img2, cvPoint(outlet_tuple.centers[3]), 10, CV_RGB(0, 255, 255));		
#endif //VERBOSE_TUPLE
	
        // save outlet borders 
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < tuple[i].seq->total; j++)
            {
                CvPoint* p = (CvPoint*)cvGetSeqElem(tuple[i].seq, j);
                outlet_tuple.borders[i].push_back(cvPoint2D32f(p->x, p->y));
            }
        }
    }
	
#if defined(_VERBOSE_TUPLE)
	cvNamedWindow("1", 1);
	cvShowImage("1", img2);
	cvWaitKey(0);
#endif //_VERBOSE_TUPLE
	
#if defined(_VERBOSE)
	if(output_path && filename)
	{
		char buf[1024];
		sprintf(buf, "%s/warped/%s", output_path, filename);
		cvSaveImage(buf, img2);
	}
#endif //_VERBOSE
	
#if defined(_VERBOSE) || defined(_VERBOSE_TUPLE)
	cvReleaseImage(&img2);
#endif

	cvReleaseMemStorage(&storage);
	cvReleaseImage(&binary);
	cvReleaseImage(&grey);
	
	return found_tuple;
	
}

void calc_bounding_rect(int count, const CvRect* rects, CvRect& bounding_rect)
{
	CvPoint tl = cvPoint(INT_MAX, INT_MAX); // top left point
	CvPoint br = cvPoint(INT_MIN, INT_MIN); //bottom right point
	for(int i = 0; i < count; i++)
	{
		tl.x = MIN(tl.x, rects[i].x);
		tl.y = MIN(tl.y, rects[i].y);
		br.x = MAX(br.x, rects[i].x + rects[i].width);
		br.y = MAX(br.y, rects[i].y + rects[i].height);
	}
	
	bounding_rect.x = tl.x;
	bounding_rect.y = tl.y;
	bounding_rect.width = br.x - tl.x;
	bounding_rect.height = br.y - tl.y;
}

int find_tuple(vector<outlet_elem_t>& candidates, CvPoint2D32f* centers)
{
	if(candidates.size() < 4)
	{
		// should be at least 4 candidates for a tuple
		return 0;
	}
	
	if(candidates.size() > 15)
	{
		// too many candidates -- the algorithm will be slow
		return 0;		
	}
	
	if(candidates.size() == 4)
	{
		// we've got a tuple!
		order_tuple2(candidates);
		for(int i = 0; i < 4; i++) 
		{
			centers[i] = candidates[i].center;
		}
		return 1;
	}
	
	printf("find_tuple: The case of more than 4 candidates is not yet supported!\n");
	return 0;
			
/*
	// look for affine-transformed rectangle with outlet centers in its corners
	// for doing that iterate through all possible 4-tuples
	int idx[4] = {0, 0, 0, -1};
	while(1)
	{
		// move to the next 4-tuple
		for(int i = 3; i >= 0; i--)
		{
			if(
	}
 */
}

const int outlet_width = 50;
const int outlet_height = 25;

const float xsize = 46.1f;
const float ysize = 38.7f;

void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix, 
							outlet_template_t templ, CvMat* inverse_map_matrix)
{
	CvPoint2D32f rectified[4];

#if 0
	rectified[0] = centers[0];
	rectified[1] = cvPoint2D32f(centers[0].x + outlet_width, centers[0].y);
	rectified[2] = cvPoint2D32f(centers[0].x + outlet_width, centers[0].y + outlet_height);
	rectified[3] = cvPoint2D32f(centers[0].x, centers[0].y + outlet_height);
#else
	memcpy(rectified, templ.get_template(), templ.get_count()*sizeof(CvPoint2D32f));
#endif
	
	cvGetPerspectiveTransform(centers, rectified, map_matrix);
	
	if(inverse_map_matrix)
	{
		cvGetPerspectiveTransform(rectified, centers, inverse_map_matrix);
	}
}

void map_vector(const vector<CvPoint2D32f>& points, CvMat* homography, vector<CvPoint2D32f>& result)
{
	int points_count = points.size();
	CvMat* src = cvCreateMat(1, points_count, CV_32FC2);
	CvMat* dst = cvCreateMat(1, points_count, CV_32FC2);
	
	for(unsigned int i = 0; i < points.size(); i++)
	{
		src->data.fl[2*i] = points[i].x;
		src->data.fl[2*i + 1] = points[i].y;
	}
	
	cvPerspectiveTransform(src, dst, homography);
	
	result.clear();
	for(int i = 0; i < points_count; i++)
	{
		result.push_back(cvPoint2D32f(dst->data.fl[2*i], dst->data.fl[2*i + 1]));
	}
	
	cvReleaseMat(&src);
	cvReleaseMat(&dst);
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
void calc_outlet_homography(const CvPoint2D32f* centers, CvSize src_size, CvMat* map_matrix, CvSize* dst_size)
{
//	CvMat* map_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* inverse_map_matrix = cvCreateMat(3, 3, CV_32FC1);
	calc_outlet_homography(centers, map_matrix, outlet_template_t(), inverse_map_matrix);
	
	CvMat* corners = cvCreateMat(1, 4, CV_32FC2);
	CvMat* dst = cvCreateMat(1, 4, CV_32FC2);
	map_image_corners(src_size, map_matrix, corners, dst);

	float xmin = 1e10, ymin = 1e10, xmax = -1e10, ymax = -1e10;
	float src_xmin, src_ymin;
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
	
	/// !!!! This is an ugly hack. The real way to do it is to offset homography transform
	if(dst_size)
	{
		dst_size->width = xmax;//ceil(xmax - xmin) + 1;
		dst_size->height = ymax;//ceil(ymax - ymin) + 1;
	}
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
						  CvPoint2D32f* scale, const char* output_path, const char* filename, CvPoint2D32f* _centers)
{
	outlet_tuple_t outlet_tuple;
	outlet_tuple.tuple_mask = 0;
	int ret = find_outlet_centroids(src, outlet_tuple, output_path, filename);
	if(!ret)
	{
		printf("Centroids not found\n");
		return 0;
	}
	
	if(_centers)
	{
		memcpy(_centers, outlet_tuple.centers, 4*sizeof(CvPoint2D32f));
	}
	
	if(hor_dir)
	{
		hor_dir->x = outlet_tuple.centers[1].x - outlet_tuple.centers[0].x;
		hor_dir->y = outlet_tuple.centers[1].y - outlet_tuple.centers[0].y;
	}
		
	calc_outlet_homography(outlet_tuple.centers, cvSize(src->width, src->height), map_matrix, dst_size);
	
	calc_origin_scale(outlet_tuple.centers, map_matrix, origin, scale);

	return 1;
}

void calc_origin_scale(const CvPoint2D32f* centers, CvMat* map_matrix, CvPoint3D32f* origin, CvPoint2D32f* scale)
{
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
		
#if 0
		scalex = xsize/(_dst->data.fl[2] - _dst->data.fl[0]);
		scaley = ysize/(_dst->data.fl[5] - _dst->data.fl[3]);
#else
		scalex = scaley = 1.0f;
#endif
		
		cvReleaseMat(&_src);
		cvReleaseMat(&_dst);
	}
	
	if(scale)
	{
		*scale = cvPoint2D32f(scalex, scaley);
	}	
}

IplImage* find_templates(IplImage* img, IplImage* templ)
{
	IplImage* templr = cvCreateImage(cvSize(outlet_width, outlet_height), IPL_DEPTH_8U, 3);
	cvResize(templ, templr);
	
	IplImage* dist = cvCreateImage(cvSize(img->width - templr->width + 1, img->height - templr->height + 1), IPL_DEPTH_32F, 1);
	cvMatchTemplate(img, templr, dist, CV_TM_SQDIFF);
	
	double max_dist, min_dist;
	cvMinMaxLoc(dist, &min_dist, &max_dist);
	
	IplImage* mask = cvCreateImage(cvSize(dist->width, dist->height), IPL_DEPTH_8U, 1);
	
	double thresh = min_dist*2.0f;
	cvThreshold(dist, mask, thresh, 255, CV_THRESH_BINARY_INV);
	
	for(int r = 0; r < dist->height; r++)
	{
		for(int c = 0; c < dist->width; c++)
		{
			if(mask->imageData[r*mask->widthStep + c] == 0)
			{
				continue;
			}
			
//			cvCircle(img, cvPoint(c + templr->width/2, r + templr->height/2), 20, CV_RGB(color, color, 0), 3);
//			cvRectangle(img, cvPoint(c, r), cvPoint(c + templr->width, r + templr->height), CV_RGB(color, color, 0), 2);
			cvRectangle(img, cvPoint(c, r), cvPoint(c + templr->width, r + templr->height), CV_RGB(0, 0, 255), 2);
		}
	}
	
	cvReleaseImage(&templr);

	return mask;
}

void calc_camera_pose(CvMat* intrinsic_mat, CvMat* distortion_coeffs, CvPoint2D32f* centers, 
					  CvMat* rotation_vector, CvMat* translation_vector)
{
	CvMat* object_points = cvCreateMat(4, 3, CV_32FC1);
	CvMat* image_points = cvCreateMat(4, 2, CV_32FC1);
	
	cvmSet(object_points, 0, 0, 0.0f);
	cvmSet(object_points, 0, 1, 0.0f);
	cvmSet(object_points, 0, 2, 0.0f);
	cvmSet(object_points, 1, 0, xsize);
	cvmSet(object_points, 1, 1, 0.0f);
	cvmSet(object_points, 1, 2, 0.0f);
	cvmSet(object_points, 2, 0, xsize);
	cvmSet(object_points, 2, 1, ysize);
	cvmSet(object_points, 2, 2, 0.0f);
	cvmSet(object_points, 3, 0, 0.0f);
	cvmSet(object_points, 3, 1, ysize);
	cvmSet(object_points, 3, 2, 0.0f);
	
	for(int i = 0; i < 4; i++) 
	{
		cvmSet(image_points, i, 0, centers[i].x);
		cvmSet(image_points, i, 1, centers[i].y);
	}
	
	CvMat* _distortion_coeffs = 0;
	if(distortion_coeffs == 0)
	{
		_distortion_coeffs = cvCreateMat(1, 5, CV_32FC1);
		for(int i = 0; i < 5; i++) cvmSet(_distortion_coeffs, 0, i, 0.0f);
	}
	else
	{
		_distortion_coeffs = distortion_coeffs;
	}

	cvFindExtrinsicCameraParams2(object_points, image_points, intrinsic_mat, _distortion_coeffs, rotation_vector, 
								 translation_vector);
	
	if(distortion_coeffs == 0)
	{
		cvReleaseMat(&_distortion_coeffs);
	}
	
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
}
