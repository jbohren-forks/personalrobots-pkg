// outlet_model.cpp : 
//
//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#include <vector>
#include <map>
#include <string>
using namespace std;

#include <cv.h>
#include <highgui.h>
#include <ml.h>

#include "star_detector/detector.h"

#include "outlet_model.h"
#include "learning.h"
#include "planar.h"
#include "outlet_tuple.h"

const int xsize = 11;
const int ysize = 11;
const int feature_count = xsize*ysize;

void DrawKeypoints(IplImage* img, std::vector<Keypoint> keypts)
{
	for(std::vector<Keypoint>::const_iterator it = keypts.begin(); it != keypts.end(); it++)
	{
		cvCircle(img, cvPoint(it->x, it->y), it->scale, CV_RGB(0, 0, 255), 2);
	}
}

void DrawKeypoints(IplImage* img, std::vector<outlet_feature_t> keypts)
{
	for(std::vector<outlet_feature_t>::const_iterator it = keypts.begin(); it != keypts.end(); it++)
	{
		CvPoint center = cvPoint(it->bbox.x + it->bbox.width/2, it->bbox.y + it->bbox.height/2);
		int scale = MAX(it->bbox.width, it->bbox.height);
		cvCircle(img, center, scale, CV_RGB(255, 0, 0), 2);
	}
}

void find_hole_candidates(IplImage* grey, IplImage* mask, CvSeq* socket, vector<CvSeq*>& holes)
{
	cvSetZero(mask);
	
	for(CvSeq* seq = socket->v_next; seq != 0; seq = seq->h_next)
	{
		CvRect rect = cvBoundingRect(seq);
#if defined(_EUROPE)
		int min_size = 5*grey->width/2272;
		int max_size = min_size*6;
		if(rect.width < min_size || rect.height < min_size || rect.width > max_size || rect.height > max_size)
		{
			continue;
		}
#else
		const int min_size = 1;
		const int max_size = 20;
		if(abs(rect.x - 49) < 5 && abs(rect.y - 44) < 5)
		{
			int w = 1;
		}
		if(rect.width < min_size || rect.height < min_size || rect.width > max_size || rect.height > max_size)
		{
			continue;
		}
#endif //_EUROPE
		
#if defined(_EUROPE)
		if(fabs(float(rect.width)/rect.height - 1) > 0.5)
		{
			continue;
		}
#endif //_EUROPE

#if 0
		CvBox2D box = cvMinAreaRect2(seq);
//		float area = fabs(cvContourArea(seq));
//		float length = cvArcLength(seq);
//		float circle_coeff = fabs(area*4*pi)/(length*length);
/*		if(circle_coeff < 0.8)
		{
			continue;
		}
		assert(circle_coeff <= 1.5);
*/
#if defined(_EUROPE)
		if(box.size.height < 0 || fabs(float(box.size.width)/box.size.height - 1) > 0.5)
		{
			continue;
		}
#endif //_EUROPE
#endif 
		
#if 0
		cvDrawContours(mask, seq, cvScalar(255), cvScalar(255), 0, CV_FILLED);
		float avg_inside = cvAvg(grey, mask).val[0];
		CvRect bound = double_rect(rect);
		bound = fit_rect(bound, grey);
		cvRectangle(mask, cvPoint(bound.x, bound.y), cvPoint(bound.x + bound.width, bound.y + bound.height), 
			cvScalar(255), CV_FILLED);
		cvDrawContours(mask, seq, cvScalar(0), cvScalar(0), 0, CV_FILLED);
		CvScalar avg_outside, std_outside;
		cvAvgSdv(grey, &avg_outside, &std_outside, mask);
#else
		CvRect inner_rect = resize_rect(rect, 0.75);
		cvSetImageROI(grey, inner_rect);
		float avg_inside = cvSum(grey).val[0];
		cvResetImageROI(grey);
#if !defined(_TUNING)
		CvRect bound = double_rect(rect);
#else
		CvRect bound = resize_rect(rect, 2.0f);
#endif //_TUNING
		bound = fit_rect(bound, grey);
		CvScalar avg_outside, std_outside;
		cvSetImageROI(grey, bound);
		avg_outside = cvSum(grey);
		cvResetImageROI(grey);
		avg_outside.val[0] -= avg_inside;
		avg_inside /= inner_rect.width*inner_rect.height;
		avg_outside.val[0] /= bound.width*bound.height;
#endif
#if 0
		cvCopy(grey, mask);
		cvDrawContours(mask, seq, cvScalar(255), cvScalar(255), 0, 1);
		cvSetImageROI(mask, bound);
		cvNamedWindow("2", 1);
		cvShowImage("2", mask);
		cvWaitKey(0);
		cvResetImageROI(mask);
#endif

		// return the mask in its previous state
		cvRectangle(mask, cvPoint(bound.x, bound.y), cvPoint(bound.x + bound.width, bound.y + bound.height), 
					cvScalar(0), CV_FILLED);
#if defined(_EUROPE)
		const float avg_factor = 1.0f;
#else
		const float avg_factor = 1.3f;
#endif //_EUROPE		
		
		float contrast, variation;
		calc_contrast_factor(grey, rect, contrast, variation);
#if !defined(_TUNING)
		if(/*std_outside.val[0]/avg_outside.val[0] > 0.3f ||*/ avg_outside.val[0] < avg_inside*avg_factor)
		{
			continue;
		}
#else
		if(contrast < 1.0f /*|| variation > 0.7f*/ || avg_outside.val[0] < avg_inside*1.0f)
		{
			continue;
		}
#endif //_TUNING
		//printf("Std outside = %f\n", std_outside.val[0]/avg_outside.val[0]);
		holes.push_back(seq);
	}
}

vector<outlet_feature_t>::const_iterator find_fartherst_hole(const vector<vector<outlet_feature_t>::const_iterator>& candidates, 
															 outlet_feature_t feature)
{
	vector<vector<outlet_feature_t>::const_iterator>::const_iterator itmax;
	int distmax = 0;
	int fx = feature.bbox.x + feature.bbox.width/2;
	for(vector<vector<outlet_feature_t>::const_iterator>::const_iterator it = candidates.begin(); it != candidates.end(); it++)
	{
		int cx = (*it)->bbox.x + (*it)->bbox.width/2;
		int dist = abs(fx - cx);
		if(dist > distmax)
		{
			distmax = dist;
			itmax = it;
		}
	}
	
	return(*itmax);
}

void find_holes(const vector<outlet_feature_t>& holes, vector<outlet_t>& outlets, IplImage* grey, IplImage* mask, IplImage* img)
{
#if defined(_EUROPE)
	int min_dist = 5*grey->width/2272;
	int max_dist = 50*grey->width/2272;
	int max_dist_prox = max_dist/2;
#else
	const int min_dist = 14;//7;//20;
	const int max_dist = 80;//30;//100;
	const int max_dist_prox = 2*max_dist;
#endif //_EUROPE
	
	// we will keep a list of references to outlets for each feature
	// we will use this to filter out overlapping outlets
	vector<vector<int> > references;
	references.resize(holes.size());

	for(vector<outlet_feature_t>::const_iterator it1 = holes.begin(); it1 != holes.end(); it1++)
	{
		vector<vector<outlet_feature_t>::const_iterator> candidates;
		int proximity_count = 0;
		
		CvRect rect1 = it1->bbox;
		int x1 = rect1.x + rect1.width/2;
		int y1 = rect1.y + rect1.height/2;
		
		if(abs(x1 - 1056) < 10 && abs(y1 - 500) < 10)
		{
			int w = 1;
		}
#if defined(_TUNING)
		int proximity_thresh = 50;
		if(x1 < proximity_thresh || y1 < proximity_thresh || 
		   x1 > grey->width - proximity_thresh ||
		   y1 > grey->height - proximity_thresh)
		{
			continue;
		}
#endif
		for(vector<outlet_feature_t>::const_iterator it2 = holes.begin(); it2 != holes.end(); it2++)
		{
//			if(it1 >= it2) continue;

			CvRect rect2 = it2->bbox;

			int x2 = rect2.x + rect2.width/2;
			int y2 = rect2.y + rect2.height/2;
			
			if(abs(x2 - 1093) < 10 && abs(y2 - 500) < 10)
			{
				int w = 1;
			}
#if defined(_TUNING)
			int proximity_thresh = 20;
			if(x2 < proximity_thresh || y2 < proximity_thresh || 
			   x2 > grey->width - proximity_thresh ||
			   y2 > grey->height - proximity_thresh)
			{
				continue;
			}
#endif
			
			if(x2 <= x1)
			{
				continue;
			}

			if(__max(abs(x1 - x2), abs(y1 - y2)) < max_dist_prox)
			{
				proximity_count++;
			}

#if defined(_SMALL)
			const int min_candidates = 5;
			const int min_ydist = 5;
#else
#if !defined(_USE_OUTLET_TUPLE)
			const int min_candidates = 3;
#else
			const int min_candidates = 100;
#endif //_USE_OUTLET_TUPLE
			const int min_ydist = 40;
#endif //_SMALL
			if(abs(y1 - y2) > min_ydist)
			{
				continue;
			}
			
#if !defined(_USE_OUTLET_TUPLE)
			 if(atan2(fabs(y1 - y2), fabs(x1 - x2)) > 2*pi/3)
			 {
				 continue;
			 }
#endif //_USE_OUTLET_TUPLE

			float dist = sqrt(float(x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
			if(dist < min_dist || dist > max_dist)
			{
				continue;
			}

			// check if there are other outlets in the region
			candidates.push_back(it2);
			int s = candidates.size();
			if(s > min_candidates)
			{
				candidates.clear();
				break;
			}			
		}

		
#if defined(_EUROPE)
		if(proximity_count < 4 && candidates.size() > 0)
#else
			for(int i = 0; i < candidates.size(); i++) 
#endif
		{
#if defined(_EUROPE)
			vector<outlet_feature_t>::const_iterator cand_it = find_fartherst_hole(candidates, *it1);
#else
			vector<outlet_feature_t>::const_iterator cand_it = candidates[i];
#endif //_EUROPE
			outlet_feature_t cand = *cand_it;
			CvRect rect2 = cand.bbox;//candidates[0].bbox;
			int x2 = rect2.x + rect2.width/2;
			int y2 = rect2.y + rect2.height/2;

			CvRect small_rect = cvRect(MIN(rect1.x, rect2.x), MIN(rect1.y, rect2.y), 
									   MAX(rect2.x + rect2.width - rect1.x, rect1.x + rect1.width - rect2.x), 
									   MAX(rect2.y + rect2.height - rect1.y, rect1.y + rect1.height - rect2.y));

			cvSetZero(mask);
			//cvRectangle(mask, small_rect, cvScalar(255), CV_FILLED);
			cvSetImageROI(grey, small_rect);
			cvSetImageROI(mask, small_rect);
			cvThreshold(grey, mask, 140, 255, CV_THRESH_BINARY_INV);
			cvResetImageROI(grey);
			cvResetImageROI(mask);
#if 0
			int color1 = (int)cvAvg(grey, mask).val[0];
#else		
			int color1 = (int)cvAvg(grey).val[0];
#endif
//			cvSaveImage("grey.jpg", grey);
//			cvSaveImage("mask.jpg", mask);
			
			// double the size of the rectangle
			small_rect = double_rect(small_rect);
			small_rect = fit_rect(small_rect, grey);
			
			CvRect large_rect = double_rect(small_rect);
			large_rect = fit_rect(large_rect, grey);
			
			cvRectangle(mask, large_rect, cvScalar(255), CV_FILLED);
			cvRectangle(mask, small_rect, cvScalar(0), CV_FILLED);
			int color2 = (int)cvAvg(grey, mask).val[0];
			
			const float rect_ratio = 1.0f;
			if(color2/color1 < rect_ratio)
			{
//				continue;
			}
			
#if 0
			cvRectangle(img, large_rect, CV_RGB(color2, color2, color2), CV_FILLED);
			cvRectangle(img, small_rect, CV_RGB(color1, color1, color1), CV_FILLED);
#endif
				
			outlet_t outlet;
			outlet.hole1 = cvPoint(x1, y1);
			outlet.hole2 = cvPoint(x2, y2);
			outlet.feature1 = *it1;
			outlet.feature2 = cand;
			outlets.push_back(outlet);
			
			int val1 = it1 - holes.begin();
			int val2 = cand_it - holes.begin();
			references[it1 - holes.begin()].push_back(outlets.size() - 1);
			references[cand_it - holes.begin()].push_back(outlets.size() - 1);
		}		

		candidates.clear();
	}
	
#if defined(_EUROPE)
	// filter overlapping outlets
	vector<outlet_t> filtered_outlets;
	vector<int> outlet_flags;
	outlet_flags.resize(outlets.size(), 1);
	for(vector<vector<int> >::const_iterator it = references.begin(); it != references.end(); it++)
	{
		float dist_max = 0;
		int it_max = -1;
		for(vector<int>::const_iterator oit = it->begin(); oit != it->end(); oit++)
		{
			float dist = outlet_size(outlets[*oit]);
			if(dist > dist_max)
			{
				dist_max = dist;
				it_max = *oit;
			}
		}
		for(vector<int>::const_iterator oit = it->begin(); oit != it->end(); oit++)
		{
			if(*oit != it_max)
			{
				outlet_flags[*oit] = outlet_flags[*oit] & 0;
			}
		}
	}
	
	for(int i = 0; i < outlet_flags.size(); i++)
	{
		if(outlet_flags[i])
		{
			filtered_outlets.push_back(outlets[i]);
		}
	}
	
	outlets = filtered_outlets;
#endif //_EUROPE
}

int test_adjacency(const vector<outlet_feature_t>& features, outlet_feature_t f)
{
	int fx = f.bbox.x + f.bbox.width/2;
	int fy = f.bbox.y + f.bbox.height/2;
	int fscale = max(f.bbox.width, f.bbox.height);
	for(vector<outlet_feature_t>::const_iterator it = features.begin(); it != features.end(); it++)
	{
		int x = it->bbox.x + it->bbox.width/2;
		int y = it->bbox.y + it->bbox.height/2;
		
		if(abs(fx - x) < fscale && abs(fy - y) < fscale)
		{
			return 1;
		}
	}
	
	return 0;
}

void find_outlet_features(IplImage* src, vector<outlet_feature_t>& features, const char* filename)
{
	const float min_intersect = 0.2;

	//cvErode(src, src);
	IplImage* grey = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	cvCvtColor(src, grey, CV_RGB2GRAY);
	cvSmooth(grey, grey);
	IplImage* mask = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	
	IplImage* mask_black = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	IplImage* mask_white = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	IplImage* imgfeat = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	cvSetZero(imgfeat);
	
	CvMemStorage* storage = cvCreateMemStorage();
	
	IplImage* imgholes = cvCloneImage(src);
	for(int coi = 1; coi < 4; coi++)
	{
		cvSetImageCOI(imgholes, coi);
		cvCopy(grey, imgholes);
	}
	cvSetImageCOI(imgholes, 0);
	
	for(int thresh = 20; thresh < 170; thresh += 20)
	{
		cvSet(mask_black, cvScalar(255));
		cvSet(mask_white, cvScalar(255));
		IplImage* tempgrey = cvCloneImage(mask_white);
		IplImage* tempmask = cvCloneImage(mask_white);

#if 0
		for(int coi = 1; coi < 4; coi++)
		{
			cvSetImageCOI(src, coi);
			cvCopy(src, tempgrey);
			cvThreshold(tempgrey, tempmask, thresh, 255, CV_THRESH_BINARY_INV);
			cvAnd(mask_black, tempmask, mask_black);
			
			cvThreshold(tempgrey, tempmask, thresh, 255, CV_THRESH_BINARY);
			cvAnd(mask_white, tempmask, mask_white);
		}
#else
		cvThreshold(grey, mask_white, thresh, 255, CV_THRESH_BINARY);
		cvThreshold(grey, mask_black, thresh, 255, CV_THRESH_BINARY_INV);
#endif
		cvSetImageCOI(src, 0);
		cvNot(mask_black, mask_black);
		
#if 0
		cvAnd(mask_white, mask_black, tempgrey);
		printf("Processing thresh = %d\n", thresh);
		cvNamedWindow("1", 1);
		cvShowImage("1", tempgrey);
		cvWaitKey(0);
#endif
		//cvCopy(src, temp);
		cvReleaseImage(&tempgrey);
		
		CvSeq* first = 0;
		cvFindContours(mask_white, storage, &first, sizeof(CvContour), CV_RETR_CCOMP);
		
		for(CvSeq* seq = first; seq != 0; seq = seq->h_next)
		{
			CvRect rect = cvBoundingRect(seq);
			if(rect.width < 40 || rect.height < 40)
			{
				continue;
			}
			
			cvSetZero(tempmask);
			cvDrawContours(tempmask, seq, cvScalar(255), cvScalar(255), 0, CV_FILLED);
			cvAnd(tempmask, mask_black, tempmask);
			CvSeq* outlet = 0;
#if 0
			int w = 1;
			if(w)
			{
			
			cvNamedWindow("1", 1);
			cvShowImage("1", tempmask);
			cvSaveImage("mask.jpg", tempmask);
			cvWaitKey(0);
			}
#endif
			cvFindContours(tempmask, storage, &outlet, sizeof(CvContour), CV_RETR_TREE);
			
			int holes_count = 0;
			for(CvSeq* seqhole = outlet->v_next; seqhole != NULL; seqhole = seqhole->h_next, holes_count++);
			if(holes_count < 2) 
			{
				continue;
			}
			
			
			vector<CvSeq*> holes;
			find_hole_candidates(grey, mask, outlet, holes);
			
			for(vector<CvSeq*>::iterator it = holes.begin(); it != holes.end(); it++)
			{
				CvRect roi = cvBoundingRect(*it);
				cvSetImageROI(imgfeat, roi);
				float avg = cvAvg(imgfeat).val[0];
				if(avg < min_intersect)
				{
					outlet_feature_t feature;
					feature.bbox = roi;
//					if(test_adjacency(features, feature) == 0)
//					{
						features.push_back(feature);
//					}
				}
			}
			cvResetImageROI(imgfeat);
			
			for(vector<outlet_feature_t>::iterator it = features.begin(); it != features.end(); it++)
			{
				//cvRectangle(imgholes, it->bbox, CV_RGB(255, 0, 0), 1);
				cvCircle(imgholes, cvPoint(it->bbox.x, it->bbox.y), 2, CV_RGB(255, 0, 0));
			}
			
		}
		
#if 0
		cvNamedWindow("1", 1);
		cvShowImage("1", imgholes);
		cvWaitKey(0);
		cvSaveImage("mask.jpg", imgholes);
#endif
		cvReleaseImage(&tempmask);
	}
	/*
	 cvNamedWindow("1", 0);
	 cvSImage("1", temp);
	 cvWaitKey(0);
	 */
	char buf[1024];
	
#if defined(_VERBOSE)
	sprintf(buf, "../../holes/%s", filename);
	cvSaveImage(buf, imgholes);
	
	sprintf(buf, "../../src/%s", filename);
	cvSaveImage(buf, src);
#endif //_VERBOSE
	
	cvReleaseImage(&grey);
	cvReleaseImage(&mask);
	
	cvReleaseImage(&mask_black);
	cvReleaseImage(&mask_white);
	cvReleaseImage(&imgholes);
	cvReleaseMemStorage(&storage);	
}

inline float outlet_size(outlet_t outlet)
{
	return fabs((outlet.hole2.x - outlet.hole1.x)*(outlet.hole2.x - outlet.hole1.x) + 
				(outlet.hole2.y - outlet.hole1.y)*(outlet.hole2.y - outlet.hole1.y));
}

CvRect outlet_rect(outlet_t outlet)
{
	float dist = fabs(outlet.hole2.x - outlet.hole1.x);
#if defined(_EUROPE)
	float width = dist*2.0f;
#else
	float width = dist*1.5f;
#endif
	float height = width;
	return double_rect(cvRect(outlet.hole1.x - width*0.25f, outlet.hole1.y - height*0.5f, width, height));
	
}

void move_features(vector<outlet_feature_t>& features, CvPoint origin)
{
	for(vector<outlet_feature_t>::iterator it = features.begin(); it != features.end(); it++)
	{
		it->bbox.x += origin.x;
		it->bbox.y += origin.y;
	}
}

void detect_outlets(IplImage* src, vector<outlet_feature_t>& features, vector<outlet_t>& outlets, 
					outlet_tuple_t* outlet_tuple, const char* output_path, const char* filename)
{
	IplImage* temp = cvCloneImage(src);
	IplImage* grey = 0;
	
	CvRect roi = outlet_tuple ? outlet_tuple->roi : cvRect(0, 0, src->width, src->height);
	cvSetImageROI(src, roi);
	grey = cvCreateImage(cvSize(roi.width, roi.height), IPL_DEPTH_8U, 1);
#if 0
	cvCvtColor(src, grey, CV_RGB2GRAY);
#else
	cvSetImageCOI(src, 3);
	cvCopy(src, grey);
	cvSetImageCOI(src, 0);
	cvConvertScale(grey, grey, 0.5);
#endif
	
	cvErode(grey, grey);
	cvDilate(grey, grey);

#if 0
	cvNamedWindow("1", 1);
	cvShowImage("1", grey);
	cvWaitKey(0);
#endif
	
	find_outlet_features_fast(grey, features, output_path, filename);
	
	move_features(features, cvPoint(roi.x, roi.y));
	
	cvReleaseImage(&grey);
	cvResetImageROI(src);

#if _PREDICT
	printf("Filtering keypoints...");
	CvRTrees* rtrees = new CvRTrees();
	rtrees->load("../../forest.xml");
	FilterPoints(grey, features, rtrees);
	printf("done. %d points left\n", features.size());
	delete rtrees;
#endif //_PREDICT

#if defined(_USE_OUTLET_TUPLE)
//	cvErode(tuple_mask, tuple_mask, 0, 10); // remove feature points on the edges
	
	if(outlet_tuple)
	{
		IplImage* distance_map = calc_tuple_distance_map(outlet_tuple->tuple_mask);
		filter_features_distance_mask(features, distance_map);
		cvReleaseImage(&distance_map);
	}
		
#endif //_USE_OUTLET_TUPLE
	
#if defined(_VERBOSE)
	char buf[1024];
	IplImage* _src = cvCloneImage(src);
	DrawKeypoints(_src, features);
	sprintf(buf, "%s/keyout/%s", output_path, filename);
	cvSaveImage(buf, _src);
	cvReleaseImage(&_src);
#endif //_VERBOSE

	grey = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	IplImage* mask = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	cvCvtColor(src, grey, CV_RGB2GRAY);
	find_holes(features, outlets, grey, mask, temp);
	
#if defined(_PREDICT_OUTLETS)
	CvRTrees* outlet_rtrees = new CvRTrees();
	outlet_rtrees->load("../../outlet_forest.xml");
	filter_outlets(grey, outlets, outlet_rtrees);
	delete outlet_rtrees;
#endif //_PREDICT_OUTLETS
	
	
#if defined(_VERBOSE)
	draw_outlets(temp, outlets);

	sprintf(buf, "%s/output/%s", output_path, filename);
	strcpy(buf + strlen(buf) - 3, "jpg"); 
	cvSaveImage(buf, temp);
#endif //_VERBOSE
	
#if defined(_TUNING) && 1
	cvCopy(src, temp);
	cvResetImageROI(src);
	cvResetImageROI(grey);

#endif // _TUNING

	
	cvReleaseImage(&temp);
	cvReleaseImage(&grey);
	cvReleaseImage(&mask);
}

int filter_outlets_templmatch(IplImage* src, vector<outlet_t>& outlets, IplImage* templ, const char* output_path, const char* filename, CvMat** homography, 
							  CvPoint3D32f* origin, CvPoint2D32f* scale)
{
	// using template matching for further filtering of fps
	// first calculate homography
	CvMat* map_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvSize dst_size;
	int ret = calc_image_homography(src, map_matrix, &dst_size, 0, origin, scale, output_path, filename);
	if(ret)
	{
		if(homography)
		{
			*homography = map_matrix;
		}
		const int max_image_width = 2048;
		const int max_image_height = 2048;
		dst_size.width = MIN(dst_size.width, max_image_width);
		dst_size.height = MIN(dst_size.height, max_image_height);
		printf("warped size: %d %d\n", dst_size.width, dst_size.height);
		IplImage* warped = cvCreateImage(dst_size, IPL_DEPTH_8U, 3);
		cvWarpPerspective(src, warped, map_matrix);
		
		cvSaveImage("warped.jpg", warped);
		
		// now load template and calculate the mask
		IplImage* warped_mask = find_templates(warped, templ);
		cvDilate(warped_mask, warped_mask, 0, 4);

#if 0
		cvNamedWindow("1", 1);
		cvShowImage("1", warped);
		cvWaitKey(0);
#endif
		
		filter_outlets_templ_ex(outlets, map_matrix, warped_mask);
				
		cvReleaseImage(&warped);
	}
	
	if(!homography)
	{
		cvReleaseMat(&map_matrix);
	}
	
	return ret;
}

void read_outlet_roi(const char* filename, outlet_roi_t& outlet_roi)
{
	FILE* fp = fopen(filename, "rt");
	
	int ret = 0;
	char buf[1024];
	int x1, y1, x2, y2;
	while((ret = fscanf(fp, "%s %d %d %d %d\n", buf, &x1, &y1, &x2, &y2)) > 0)
	{
		string str = string(buf);
		outlet_roi[str].push_back(cvRect(x1, y1, x2 - x1, y2 - y1));
	}
	
	fclose(fp);
}

int is_point_inside_roi(const vector<CvRect>& rects, CvPoint point)
{
	for(vector<CvRect>::const_iterator it = rects.begin(); it != rects.end(); it++)
	{
		if(is_point_inside_rect(*it, point))
		{
		   return 1;
		}
	}
		   
	return 0;
}

int is_point_incenter_roi(const vector<CvRect>& rects, CvPoint point)
{
	for(vector<CvRect>::const_iterator it = rects.begin(); it != rects.end(); it++)
	{
		CvRect small = resize_rect(*it, 0.5f);
		if(is_point_inside_rect(small, point))
		{
			return 1;
		}
	}
	
	return 0;
}

int is_point_inside_roi(const outlet_roi_t& outlet_roi, CvPoint point, string img_name)
{
	map<string, vector<CvRect> >::const_iterator it = outlet_roi.find(img_name);
	//vector<CvRect> vrect = it->second;
	if(it == outlet_roi.end())
	{
		// no element with such a name
		return 0;
	}
	
	int ret = is_point_inside_roi(it->second, point);
	return ret;
}

inline int is_rect_inside_rect(CvRect large, CvRect small)
{
	if(small.x >= large.x && small.y >= large.y && small.x + small.width <= large.x + large.width &&
	   small.y + small.height <= large.y + large.height)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void calc_labels(const vector<CvRect>& rects, const vector<Keypoint>& keypts, vector<int>& labels)
{
	for(vector<Keypoint>::const_iterator it = keypts.begin(); it != keypts.end(); it++)
	{
#if defined(_TRAIN)
		int label = is_point_incenter_roi(rects, cvPoint(it->x, it->y));
#else
		int label = is_point_inside_roi(rects, cvPoint(it->x, it->y));
#endif //_TRAIN
		labels.push_back(label);
	}
}

void calc_labels(const vector<CvRect>& rects, const vector<outlet_feature_t>& keypts, vector<int>& labels)
{
	for(vector<outlet_feature_t>::const_iterator it = keypts.begin(); it != keypts.end(); it++)
	{
		CvPoint center = cvPoint(it->bbox.x + it->bbox.width/2, it->bbox.y + it->bbox.height/2);
#if defined(_TRAIN)
		int label = is_point_incenter_roi(rects, center);
#else
		int label = is_point_inside_roi(rects, center);
#endif //_TRAIN
		labels.push_back(label);
	}
}

void extract_intensity_features(IplImage* grey, const vector<outlet_feature_t>& keypts, CvMat** mat, 
								int equalize_hist, const vector<int>& labels, const char* buf)
{
	int start_row = 0;
	
	if(!(*mat))
	{
		*mat = cvCreateMat(keypts.size(), feature_count, CV_32FC1);
	}
	else
	{
		start_row = (*mat)->rows;
		CvMat* _mat = cvCreateMat((*mat)->rows + keypts.size(), feature_count, CV_32FC1);
		// copy mat to the beginning of _mat
		for(int r = 0; r < (*mat)->rows; r++)
		{
			memcpy(_mat->data.ptr + _mat->step*r, (*mat)->data.ptr + (*mat)->step*r, sizeof(float)*_mat->cols);
		}
		
		// now free *mat
		cvReleaseMat(mat);
		*mat = _mat;
	}

	// prepare a temporary image
	IplImage* features = cvCreateImage(cvSize(xsize, ysize), IPL_DEPTH_8U, 1);
	
	// now extract intensity values and copy them to the matrix
	for(int r = 0; r < keypts.size(); r++)
	{
//		CvPoint center = cvPoint(keypts[r].bbox.x + keypts[r].bbox.width/2, 
//								 keypts[r].bbox.y + keypts[r].bbox.height/2);
//		int scale = keypts[r].bbox.width;
		const float scale_factor = 2.0f;
//		CvRect roi = cvRect(center.x - scale_factor*scale, center.y - scale_factor*scale, 
//							2*scale_factor*scale, 2*scale_factor*scale);
		CvRect roi = keypts[r].bbox;
		roi = resize_rect(roi, scale_factor);		
		roi = fit_rect(roi, grey);
		
		cvSetImageROI(grey, roi);
		cvResize(grey, features);
		if(equalize_hist)
		{
			cvEqualizeHist(features, features);
		}
#if 0
		const float norm = 1;
#else
		float norm = cvSum(features).val[0];
#endif
		for(int y = 0; y < ysize; y++)
		{
			for(int x = 0; x < xsize; x++)
			{
				cvmSet(*mat, start_row + r, y*xsize + x, float(features->imageData[features->widthStep*y + x])/norm);
			}
		}
		cvResetImageROI(grey);
		if(labels.size() == 0)
		{
			// do not save image patch
			continue;
		}
		
		// save the feature to an image
		char filename[1024];
		char lab[1024];
		if(labels[start_row + r] == 0)
		{
			continue;
			strcpy(lab, "neg");
		}
		else
		{
			strcpy(lab, "pos");
		}
#if defined(_VERBOSE)
		sprintf(filename, "../../rfsamples/%s/%s_%d.jpg", lab, buf, start_row + r);
		cvSaveImage(filename, features);
#endif
	}
	
	cvReleaseImage(&features);
}

CvMat* vector2mat(const vector<int>& vec)
{
	CvMat* mat = cvCreateMat(vec.size(), 1, CV_32SC1);
	for(int i = 0; i < vec.size(); i++)
	{
		*(int*)(mat->data.ptr + mat->step*i) = vec[i];
	}
	
	return mat;
}

void FilterPoints(IplImage* grey, vector<outlet_feature_t>& keypts, const CvRTrees* rtrees)
{
/*	IplImage* _temp = cvCreateImage(cvSize(grey->width, grey->height), IPL_DEPTH_8U, 3);
	for(int coi = 1; coi < 4; coi++)
	{
		cvSetImageCOI(_temp, coi);
		cvCopyImage(grey, _temp);
	}
	cvSetImageCOI(_temp, 0);
*/	
	vector<outlet_feature_t> filtered;
	for(vector<outlet_feature_t>::const_iterator it = keypts.begin(); it != keypts.end(); it++)
	{
		vector<outlet_feature_t> temp;
		temp.push_back(*it);
		CvMat* sample = 0;
		extract_intensity_features(grey, temp, &sample);
		float prob = rtrees->predict_prob(sample);
		if(prob > 350.0f)
		{
			outlet_feature_t feature;
			feature.bbox = it->bbox;
			feature.weight = prob;
			filtered.push_back(feature);
		}
	}
	
//	cvNamedWindow("1", 1);
//	cvShowImage("1", _temp);
//	cvWaitKey();
	
//	cvReleaseImage(&_temp);
	
	keypts = filtered;
}

void filter_outlets(IplImage* grey, vector<outlet_t>& outlets, CvRTrees* rtrees)
{
	vector<outlet_t> filtered_outlets;
	float max_prob = 0;
	vector<outlet_t>::const_iterator max_it;
	for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		vector<outlet_feature_t> features;
		outlet_feature_t feature;
		feature.bbox = outlet_rect(*it);
		features.push_back(feature);
		CvMat* sample = 0;
		extract_intensity_features(grey, features, &sample, 1);
		float prob = rtrees->predict_prob(sample);
		printf("outlet center = %d %d, prob = %f\n", feature.bbox.x + feature.bbox.width/2, 
			   feature.bbox.y + feature.bbox.height/2, prob);
#if 1
		if(prob > 0.0f)
		{
			outlet_t outlet = *it;
			outlet.weight = prob;
			filtered_outlets.push_back(outlet);
		}
	}
#else
		if(prob > max_prob)
		{
			max_prob = prob;
			max_it = it;
		}
	}

	if(max_prob > 0)
	{
		filtered_outlets.push_back(*max_it);
	}
	
#endif
	outlets = filtered_outlets;
}

Keypoint outletf2keypoint(outlet_feature_t feature)
{
	Keypoint p;
	p.x = feature.bbox.x + feature.bbox.width/2;
	p.y = feature.bbox.y + feature.bbox.height/2;
	p.scale = min(feature.bbox.width, feature.bbox.height)/2;
	return p;
}

outlet_feature_t keypoint2outletf(Keypoint p)
{
	outlet_feature_t feature;
	feature.bbox = cvRect(p.x - p.scale, p.y - p.scale, p.scale*2, p.scale*2);
	return feature;
}

void outletfarr2keypointarr(const vector<outlet_feature_t>& features, vector<Keypoint>& keypoints)
{
	for(vector<outlet_feature_t>::const_iterator it = features.begin(); it != features.end(); it++)
	{
		outlet_feature_t feature = *it;
		Keypoint p = outletf2keypoint(feature);
		keypoints.push_back(p);
	}
}

void keypointarr2outletfarr(const vector<Keypoint>& keypoints, vector<outlet_feature_t>& features)
{
	for(vector<Keypoint>::const_iterator it = keypoints.begin(); it != keypoints.end(); it++)
	{
		Keypoint p = *it;
		outlet_feature_t feature = keypoint2outletf(p);
		features.push_back(feature);
	}
}

void find_outlet_features_fast(IplImage* src, vector<outlet_feature_t>& features, const char* output_path, const char* filename)
{
	const float min_intersect = 1;//0.2;
	const int min_pixel_area = 6;
	
	//cvErode(src, src);
	IplImage* grey = 0;
	
	if(src->nChannels == 3)
	{
		grey = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
		
#if _USE_OUTLET_TUPLE
		cvSetImageCOI(src, 3);
		cvCopy(src, grey);
//		cvConvertScale(grey, grey, 0.4);
#else
		cvCvtColor(src, grey, CV_RGB2GRAY);
#endif
	}
	else
	{
		grey = src;
	}
		
	cvSmooth(grey, grey);
	
	IplImage* mask = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	
	IplImage* mask_black = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	IplImage* mask_white = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	IplImage* imgfeat = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	cvSetZero(imgfeat);
	
	CvMemStorage* storage = cvCreateMemStorage();
	
	IplImage* imgholes = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 3);
	for(int coi = 1; coi < 4; coi++)
	{
		cvSetImageCOI(imgholes, coi);
		cvCopy(grey, imgholes);
	}
	cvSetImageCOI(imgholes, 0);

#if !defined(_TUNING)
	for(int thresh = 20; thresh < 170; thresh += 20)
#else
	for(int thresh = 0; thresh < 90; thresh += thresh < 20 ? 1 : 5)
#endif //_TUNING
		{
		cvSet(mask_black, cvScalar(255));
		cvSet(mask_white, cvScalar(255));
		IplImage* tempgrey = cvCloneImage(mask_white);
		IplImage* tempmask = cvCloneImage(mask_white);
		
#if 0
		for(int coi = 1; coi < 4; coi++)
		{
			cvSetImageCOI(src, coi);
			cvCopy(src, tempgrey);
			cvThreshold(tempgrey, tempmask, thresh, 255, CV_THRESH_BINARY_INV);
			cvAnd(mask_black, tempmask, mask_black);
			
			cvThreshold(tempgrey, tempmask, thresh, 255, CV_THRESH_BINARY);
			cvAnd(mask_white, tempmask, mask_white);
		}
#else
		cvThreshold(grey, mask_white, thresh, 255, CV_THRESH_BINARY);
		cvThreshold(grey, mask_black, thresh, 255, CV_THRESH_BINARY_INV);
#endif
		cvSetImageCOI(src, 0);
		cvNot(mask_black, mask_black);
		
#if 0
		cvAnd(mask_white, mask_black, tempgrey);
//		cvRectangle(tempgrey, cvPoint(0, 0), cvPoint(tempmask->width-1, tempmask->height-1),
//					cvScalar(0), 50);
		printf("Processing thresh = %d\n", thresh);
		cvNamedWindow("1", 1);
		cvShowImage("1", tempgrey);
		cvWaitKey(0);
#endif
		//cvCopy(src, temp);
		cvReleaseImage(&tempgrey);
		
		cvAnd(mask_white, mask_black, tempmask);
//		cvRectangle(tempmask, cvPoint(0, 0), cvPoint(tempmask->width-1, tempmask->height-1),
//					cvScalar(0), 50);
		CvSeq* first = 0;
		cvFindContours(tempmask, storage, &first, sizeof(CvContour), CV_RETR_CCOMP);
		
		for(CvSeq* seq = first; seq != 0; seq = seq->h_next)
		{
//			if(seq->total < 20)
//			{
//				continue;
//			}
			CvRect rect = cvBoundingRect(seq);
			if(rect.width < 40 || rect.height < 40)
			{
				continue;
			}

			CvSeq* outlet = seq;
			
			int holes_count = 0;
			for(CvSeq* seqhole = outlet->v_next; seqhole != NULL; seqhole = seqhole->h_next, holes_count++);
			if(holes_count < 2) 
			{
				continue;
			}
			
			
			vector<CvSeq*> holes;
			find_hole_candidates(grey, mask, outlet, holes);
			
			for(vector<CvSeq*>::iterator it = holes.begin(); it != holes.end(); it++)
			{
				CvRect roi = cvBoundingRect(*it);

				if(abs(roi.x - 200) < 10 && abs(roi.y - 872) < 10)
				{
					int w = 1;
				}

//				float scale_factor = MAX(2.0f, 2*min_pixel_area/roi.width);
//				roi = resize_rect(roi, scale_factor);
				cvSetImageROI(imgfeat, roi);
				float avg = cvAvg(imgfeat).val[0];
				cvResetImageROI(imgfeat);
				if(avg < min_intersect)
				{
					outlet_feature_t feature;
					feature.bbox = roi;
					features.push_back(feature);
					
					cvDrawContours(imgfeat, *it, cvScalar(255), cvScalar(255), 0, CV_FILLED);
				}
			}
			cvResetImageROI(imgfeat);
			
			for(vector<outlet_feature_t>::iterator it = features.begin(); it != features.end(); it++)
			{
				cvRectangle(imgholes, it->bbox, CV_RGB(0, 0, 255), 1);
				//cvCircle(imgholes, cvPoint(it->bbox.x, it->bbox.y), 2, CV_RGB(0, 0, 255));
			}
			
		}
		
#if 0
		cvNamedWindow("1", 1);
		cvShowImage("1", imgholes);
		cvWaitKey(0);
		cvSaveImage("mask.jpg", imgholes);
#endif
		cvReleaseImage(&tempmask);
	}
	/*
	 cvNamedWindow("1", 0);
	 cvSImage("1", temp);
	 cvWaitKey(0);
	 */
	
	// filtering with canny
#if 0
	filter_canny(grey, features);
#endif
	
	char buf[1024];
	
#if defined(_VERBOSE)
	sprintf(buf, "%s/holes/%s", output_path, filename);
	cvSaveImage(buf, imgholes);
#endif //_VERBOSE
	
	if(src->nChannels == 3)
	{
		cvReleaseImage(&grey);
	}
	cvReleaseImage(&mask);
	
	cvReleaseImage(&mask_black);
	cvReleaseImage(&mask_white);
	cvReleaseImage(&imgholes);
	cvReleaseMemStorage(&storage);	
}

int is_outlet_inside_roi(const outlet_roi_t& outlet_roi, outlet_t outlet, string img_name)
{
	map<string, vector<CvRect> >::const_iterator it = outlet_roi.find(img_name);
	if(it == outlet_roi.end())
	{
		// no element with such a name
		return 0;
	}
	
	int ret1 = is_point_inside_roi(it->second, outlet.hole1);
	int ret2 = is_point_inside_roi(it->second, outlet.hole2);
	
	return ret1 && ret2;
}

int generate_outlet_samples(IplImage* grey, outlet_t outlet, int count, CvMat** predictors, const char* filename)
{
	IplImage** patches = new IplImage*[count];
	CvRect roi = outlet_rect(outlet);
	cvSetImageROI(grey, roi);
	gen_random_homog_patches(grey, count, patches);
	cvResetImageROI(grey);
	save_image_array("../../patches", filename, count, patches);
	
	int outlet_count = 0;
	for(int i = 0; i < count; i++)
	{
#if 0
		outlet_feature_t feature;
		feature.bbox = cvRect(0, 0, patches[i]->width, patches[i]->height);
		vector<outlet_feature_t> features;
		features.push_back(feature);
		extract_intensity_features(patches[i], features, predictors);
#else
		vector<outlet_feature_t> features;
		vector<outlet_t> outlets;
		IplImage* color = cvCreateImage(cvSize(patches[i]->width, patches[i]->height), IPL_DEPTH_8U, 3);
		cvCvtColor(patches[i], color, CV_GRAY2RGB);
		detect_outlets(color, features, outlets, 0, 0, filename);
		if(outlets.size() > 0)
		{
			outlet_feature_t feature;
			feature.bbox = outlet_rect(outlets[0]);
			vector<outlet_feature_t> features;
			features.push_back(feature);
			extract_intensity_features(patches[i], features, predictors);
			outlet_count++;
		}
		else
		{
			continue;
		}
#endif
	}
	
	
	// releasing image patches
	for(int i = 0; i < count; i++)
	{
		cvReleaseImage(&patches[i]);
	}
	delete []patches;
	
	return(outlet_count);
}

void train_outlet_model(const char* path, const char* config_filename, 
						const char* roi_filename, const char* forest_filename)
{
	const int samples_per_outlet = 30; 
	outlet_roi_t outlet_roi;
	read_outlet_roi(roi_filename, outlet_roi);

	CvMat* predictors = 0;
	vector<int> labels;
	
	FILE* fp = fopen(config_filename, "rt");
	char buf[1024];
	int class_id;
	int ret;
	while((ret=fscanf(fp, "%d %s\n", &class_id, buf)) > 0)
	{
		printf("Processing file %s...", buf);
		
		char filename[1024];
		sprintf(filename, "%s/%s", path, buf);
		IplImage* src = cvLoadImage(filename);
		IplImage* grey = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
		cvCvtColor(src, grey, CV_RGB2GRAY);
		
		vector<outlet_feature_t> features;
		vector<outlet_t> outlets;
		detect_outlets(src, features, outlets, 0, 0, buf);
		
		// now transform outlets into features and calculate labels
		for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
		{
			CvRect orect = outlet_rect(*it);
			outlet_feature_t feature;
			feature.bbox = orect;
			vector<outlet_feature_t> f;
			f.push_back(feature);
			int outlet_count = generate_outlet_samples(grey, *it, samples_per_outlet, &predictors, buf);
			
			int outlet_class = class_id == 1 && is_outlet_inside_roi(outlet_roi, *it, string(buf));
			labels.insert(labels.end(), outlet_count, outlet_class);
		}
		
#if defined(_VERBOSE)
		DrawKeypoints(src, features);
		sprintf(filename, "../../keyout/%s", buf);
		cvSaveImage(filename, src);
#endif //_VERBOSE
		
		cvReleaseImage(&grey);
		cvReleaseImage(&src);
		
		printf("done.\n");
	}
	
	CvMat* labels_mat = vector2mat(labels);
	
	printf("Training RF model...");
	CvRTrees* rtrees = train_rf(predictors, labels_mat); 
	printf("done.\n");
		
	rtrees->save("../../outlet_forest.xml");
}

void write_pr(const char* pr_filename, const char* image_filename, const outlet_roi_t& outlet_roi, 
	const vector<outlet_t>& outlets)
{
	FILE* fp = fopen(pr_filename, "at");
	
	for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		// determine outlet class
		int class_id = is_outlet_inside_roi(outlet_roi, *it, image_filename);
		float weight = MIN(it->feature1.weight, it->feature2.weight);//it->weight
		fprintf(fp, "%s,%d,%f,%d,%d\n", image_filename, class_id, weight, (it->hole1.x + it->hole2.x)/2, 
				(it->hole1.y + it->hole2.y)/2);
	}
	
	fclose(fp);
}

void filter_negative_samples(const vector<CvRect>& rects, vector<outlet_feature_t>& keypts, float fraction)
{
	vector<int> labels;
	calc_labels(rects, keypts, labels);
	vector<outlet_feature_t> filtered_keypts;
	for(int i = 0; i < labels.size(); i++)
	{
		if(labels[i] == 1 || float(rand())/RAND_MAX < fraction) // leaving fraction negative samples 
		{
			filtered_keypts.push_back(keypts[i]);
			continue;
		}
	}
	
	keypts = filtered_keypts;
}

void calc_contrast_factor(IplImage* grey, CvRect rect, float& contrast, float& variation)
{
	CvPoint center = cvPoint(rect.x + rect.width/2, 
							 rect.y + rect.height/2);
	int scale = MAX(rect.width/2, rect.height/2);
	rect = cvRect(center.x - scale, center.y - scale, 2*scale, 2*scale);
	rect = resize_rect(rect, 1.5f);
	rect = fit_rect(rect, grey);
	
	int Ic = (unsigned char)grey->imageData[center.y*grey->widthStep + center.x];
	int I[4];
	I[0] = (unsigned char)grey->imageData[rect.y*grey->widthStep + rect.x];
	I[1] = (unsigned char)grey->imageData[(rect.y + rect.height)*grey->widthStep + rect.x];
	I[2] = (unsigned char)grey->imageData[(rect.y + rect.height)*grey->widthStep + rect.x + rect.width];
	I[3] = (unsigned char)grey->imageData[rect.y*grey->widthStep + rect.x + rect.width];
	int minI = 65535;
	int maxI = 0;
	for(int i = 0; i < 4; i++) 
	{
		minI = MIN(minI, I[i]);
		maxI = MAX(maxI, I[i]);
	}

	contrast = float(minI)/Ic;
	variation = float(maxI - minI)/maxI;
}

bool outlet_orient_pred_greater(outlet_t outlet1, outlet_t outlet2)
{
	return outlet1.weight_orient > outlet2.weight_orient;
}

void select_orient_outlets(CvPoint2D32f orientation, vector<outlet_t>& outlets, int count)
{
	// normalize the vector of horizontal direction
	float mod = sqrt(orientation.x*orientation.x + orientation.y*orientation.y);
	orientation.x /= mod;
	orientation.y /= mod;
	
	// filter out non-horizontal outlets
	vector<outlet_t> filtered_outlets;
	const float min_product = cos(pi*10/180);
	for(vector<outlet_t>::iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		CvPoint2D32f outlet_dir = cvPoint2D32f(it->hole2.x - it->hole1.x, it->hole2.y - it->hole1.y);
		float outlet_mod = sqrt(outlet_dir.x*outlet_dir.x + outlet_dir.y*outlet_dir.y);
		outlet_dir.x /= outlet_mod;
		outlet_dir.y /= outlet_mod;
		
		float product = outlet_dir.x*orientation.x + outlet_dir.y*orientation.y;
		it->weight_orient = product;
		if(count == 0 && product > min_product)
		{
			filtered_outlets.push_back(*it);
		}
	}
	
	if(count == 0)
	{
		outlets = filtered_outlets;
		return;
	}
	
	sort(outlets.begin(), outlets.end(), outlet_orient_pred_greater);
	count = MIN(count, outlets.size());
	outlets = vector<outlet_t>(outlets.begin(), outlets.begin() + count);
}

int select_orient_outlets_ex(IplImage* grey, vector<outlet_t>& outlets, const char* filename)
{
	const int board_width = 6;
	const int board_height = 9;
	const int corners_count = board_width*board_height;
	int corners_found = -1;
	CvPoint2D32f corners[corners_count];

#if 0	
	cvNamedWindow("1", 1);
	cvShowImage("1", grey);
	cvWaitKey(0);
#endif

#if 0
	int ret = cvFindChessboardCorners(grey, cvSize(board_width, board_height), 
									  corners, &corners_found);
	
	printf("ret = %d\n", ret);
	if(ret == 0)
	{
		return 0;
	}
	
	// find horizonal direction
	CvPoint2D32f hor_dir = cvPoint2D32f(corners[4].x - corners[0].x, corners[4].y - corners[0].y);
#else
	CvPoint2D32f hor_dir;
	char buf[1024];
	sprintf(buf, "../../../images/us_outlets_hr/%s", filename);
	strcpy(buf + strlen(buf) - 3, "txt");
	FILE* fp = fopen(buf, "rt");
	if(fp != 0)
	{
		int xdir = -1, ydir = -1;
		fscanf(fp, "%d %d\n", &xdir, &ydir);
		fclose(fp);
		hor_dir = cvPoint2D32f(xdir, ydir);
	}
	else
	{
		printf("File %s not found...\n", buf);
		return 0;
	}
		
#endif

	select_orient_outlets(hor_dir, outlets);
	
	return 1;
}

void draw_outlets(IplImage* temp, const vector<outlet_t>& outlets)
{
	for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		cvCircle(temp, it->hole1, 3, CV_RGB(255, 0, 0), CV_FILLED);
		cvCircle(temp, it->hole2, 3, CV_RGB(255, 0, 0), CV_FILLED);
		cvLine(temp, it->hole1, it->hole2, CV_RGB(0, 0, 255), 2);
		CvRect orect = outlet_rect(*it);
		cvRectangle(temp, orect, CV_RGB(0, 255, 0), 2);
	}
}

void filter_canny(IplImage* grey, vector<outlet_feature_t>& features)
{
	const int max_size = 100;
	const float min_dist = 10;
	
	IplImage* canny = cvCloneImage(grey);
	cvCanny(grey, canny, 20, 40);
	
	IplImage* canny1 = cvCloneImage(canny);
	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first = 0;
	cvFindContours(canny1, storage, &first, sizeof(CvContour), CV_RETR_CCOMP);
	for(CvSeq* seq = first; seq != 0; seq = seq->h_next)
	{
		CvRect rect = cvBoundingRect(seq);
		if(MAX(rect.width, rect.height) < max_size && seq->total < max_size)
		{
			cvDrawContours(canny, seq, cvScalar(0), cvScalar(0), 0, CV_FILLED);
		}
	}

/*	cvCopy(canny, canny1);
	for(vector<outlet_feature_t>::const_iterator it = features.begin(); it != features.end(); it++)
	{
		cvRectangle(canny1, it->bbox, cvScalar(255), 1);
	}
	cvNamedWindow("1", 1);
	cvShowImage("1", canny1);
	cvWaitKey(0);
*/
	for(int i = 0; i < min_dist; i++)
	{
		cvDilate(canny, canny);
	}
	vector<outlet_feature_t> filtered;
	for(vector<outlet_feature_t>::const_iterator it = features.begin(); it != features.end(); it++)
	{
		CvPoint center = cvPoint(it->bbox.x + it->bbox.width/2, it->bbox.y + it->bbox.height/2);
		if(canny->imageData[center.y*canny->widthStep + center.x] == 0)
		{
			filtered.push_back(*it);
		}
	}
	
	features = filtered;
}

IplImage* load_match_template_mask(const char* filename)
{
	char buf[1024];
	sprintf(buf, "../../../rectify_outlets/mask/%s", filename);
	strcpy(buf + strlen(buf) - 3, "jpg");
	IplImage* mask = cvLoadImage(buf);
	return(mask);
}

int load_homography_map(const char* filename, CvMat** map_matrix)
{
	char buf[1024];
	sprintf(buf, "../../../rectify_outlets/homography/%s", filename);
	strcpy(buf + strlen(buf) - 3, "xml");
	*map_matrix = (CvMat*)cvLoad(buf);
	if(*map_matrix == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void filter_outlets_templ_ex(vector<outlet_t>& outlets, CvMat* map_matrix, IplImage* mask)
{
	const int outlet_width = 50; // hack, should be updated each time a template size is changed
	const int outlet_height = 25;
	
	vector<outlet_t> filtered;
	CvMat* src = cvCreateMat(1, 1, CV_32FC2);
	CvMat* dst = cvCreateMat(1, 1, CV_32FC2);
	
	for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		CvPoint2D32f center = cvPoint2D32f((it->hole1.x + it->hole2.x)*0.5f, (it->hole1.y + it->hole2.y)*0.5f);
		
		// map center point to the mask
		src->data.fl[0] = center.x;
		src->data.fl[1] = center.y;
		cvPerspectiveTransform(src, dst, map_matrix);
		CvPoint mapped_center = cvPoint((int)dst->data.fl[0] - outlet_width/2, (int)dst->data.fl[1] - outlet_height/2);
		if(is_point_inside_rect(cvRect(0, 0, mask->width, mask->height), mapped_center) &&
		   mask->imageData[mapped_center.y*mask->widthStep + mapped_center.x])
		{
			filtered.push_back(*it);
		}
		
	}
	
	cvReleaseMat(&src);
	cvReleaseMat(&dst);

	outlets = filtered;
}

// uses template matching, currently done offline
int filter_outlets_templ(vector<outlet_t>& outlets, const char* filename)
{
	// load mask and homography
	IplImage* mask = load_match_template_mask(filename);
	if(mask == 0)
	{
		printf("Homography mask not found for image %s\n", filename);
		return 0;
	}
	cvThreshold(mask, mask, 128, 255, CV_THRESH_BINARY);
	cvDilate(mask, mask, 0, 2);
	
	CvMat* map_matrix = 0;
	int ret = load_homography_map(filename, &map_matrix);
	if(ret == 0)
	{
		printf("Matrix not found for image %s\n", filename);
		return 0;
	}
	
	filter_outlets_templ_ex(outlets, map_matrix, mask);	
	
	cvReleaseImage(&mask);
	cvReleaseMat(&map_matrix);
	
	return 1;
}

CvPoint3D32f map_point(CvPoint3D32f point, CvMat* rotation_mat, CvMat* translation_vector)
{
	CvMat* _point = cvCreateMat(3, 1, CV_32FC1);
	cvmSet(_point, 0, 0, point.x);
	cvmSet(_point, 1, 0, point.y);
	cvmSet(_point, 2, 0, point.z);
	
	CvMat* _res = cvCreateMat(3, 1, CV_32FC1);
	
	cvMatMulAdd(rotation_mat, _point, translation_vector, _res);
	
	CvPoint3D32f res = cvPoint3D32f(cvmGet(_res, 0, 0), cvmGet(_res, 1, 0), cvmGet(_res, 2, 0));
	return res;
}

int calc_outlet_coords(vector<outlet_t>& outlets, CvMat* map_matrix, CvPoint3D32f origin, CvPoint2D32f scale, 
	CvMat* rotation_vector, CvMat* translation_vector)
{	
	// create rotation matrix
	CvMat* rotation_mat = cvCreateMat(3, 3, CV_32FC1);
	cvRodrigues2(rotation_vector, rotation_mat);
	
	// map outlets to rectified plane
	CvMat* src = cvCreateMat(1, 2, CV_32FC2);
	CvMat* dst = cvCreateMat(1, 2, CV_32FC2);
	
	for(vector<outlet_t>::iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		src->data.fl[0] = it->hole1.x;
		src->data.fl[1] = it->hole1.y;
		src->data.fl[2] = it->hole2.x;
		src->data.fl[3] = it->hole2.y;
		cvPerspectiveTransform(src, dst, map_matrix);
		it->coord_hole1 = cvPoint3D32f((dst->data.fl[0] - origin.x)*scale.x, 
									   (dst->data.fl[1] - origin.y)*scale.y, -origin.z);
		it->coord_hole2 = cvPoint3D32f((dst->data.fl[2] - origin.x)*scale.x, 
									   (dst->data.fl[3] - origin.y)*scale.y, -origin.z);
		const float ground_hole_offset = 11.5; // mm
		it->coord_hole_ground = cvPoint3D32f((it->coord_hole1.x + it->coord_hole2.x)*0.5f,
											 (it->coord_hole1.y + it->coord_hole2.y)*0.5f - ground_hole_offset,
											 0.0f);
		
		it->coord_hole1 = map_point(it->coord_hole1, rotation_mat, translation_vector);
		it->coord_hole2 = map_point(it->coord_hole2, rotation_mat, translation_vector);
		it->coord_hole_ground = map_point(it->coord_hole_ground, rotation_mat, translation_vector);
	}
	
	cvReleaseMat(&src);
	cvReleaseMat(&dst);
	
	cvReleaseMat(&rotation_mat);
	
	return 1;
}
	
void calc_outlet_dist_stat(const vector<outlet_t>& outlets, float& mean, float& stddev)
{
	float sum = 0;
	float sum2 = 0;
	// calculate the average distance between the holes
	for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		float diff = length(it->coord_hole2 - it->coord_hole1);
		sum += diff;
		sum2 += diff*diff;
	}
	mean = sum/outlets.size();
	stddev = sqrt(sum2/outlets.size() - mean*mean);
}

void calc_outlet_tuple_dist_stat(const vector<outlet_t>& outlets, float& ground_dist_x1, 
								 float& ground_dist_x2, float& ground_dist_y)
{
	if(outlets.size() < 4) return;
	
//	CvPoint2D32f ground_holes[4];
//	for(int i = 0; i < 4; i++)
//	{
//		CvPoint3D32f point = outlets[i].coord_hole_ground;
//		ground_holes[i] = cvPoint2D32f(point.x, point.y);
//	}
	
//	order_tuple2(ground_holes); outlets are ordered before, no need to do it here!
	
	ground_dist_x1 = length(outlets[1].coord_hole_ground - outlets[0].coord_hole_ground);
	ground_dist_x2 = length(outlets[2].coord_hole_ground - outlets[3].coord_hole_ground);
	ground_dist_y = length(outlets[2].coord_hole_ground - outlets[1].coord_hole_ground);	
}

int find_origin_chessboard(IplImage* src, CvMat* map_matrix, CvPoint3D32f& origin, float bar_length)
{
	const int board_width = 6;
	const int board_height = 9;
	const int corners_count = 6*9;
	CvPoint2D32f corners[corners_count];
	int found_corners = 0;
	cvFindChessboardCorners(src, cvSize(board_width, board_height), corners, &found_corners);
	if(found_corners < 4*board_width)
	{
		return 0;
	}
	
	CvMat* _src = cvCreateMat(1, 2, CV_32FC2);
	CvMat* _dst = cvCreateMat(1, 2, CV_32FC2);
	_src->data.fl[0] = corners[3*board_width].x;
	_src->data.fl[1] = corners[3*board_width].y;
	_src->data.fl[2] = corners[3*board_width + board_width - 1].x;
	_src->data.fl[3] = corners[3*board_width + board_width - 1].y;
	cvPerspectiveTransform(_src, _dst, map_matrix);
	origin = cvPoint3D32f(_dst->data.fl[0], _dst->data.fl[1], 0);
	bar_length = (_dst->data.fl[2] - _dst->data.fl[0])/(board_width - 1);
	
	return 0;
}

void filter_outlets_mask(vector<outlet_t>& outlets, IplImage* mask)
{
	vector<outlet_t> filtered;
	for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		const outlet_t outlet = *it;
		if(mask->imageData[it->hole1.y*mask->widthStep + it->hole1.x] != 0 &&
		   mask->imageData[it->hole2.y*mask->widthStep + it->hole2.x] != 0)
		{
			filtered.push_back(*it);
		}
	}
	
	outlets = filtered;
}

void filter_features_mask(vector<outlet_feature_t>& features, IplImage* mask)
{
	vector<outlet_feature_t> filtered;
	for(vector<outlet_feature_t>::const_iterator it = features.begin(); it != features.end(); it++)
	{
		CvPoint center = cvPoint(it->bbox.x + it->bbox.width/2, it->bbox.y + it->bbox.height/2);
		if(mask->imageData[mask->widthStep*center.y + center.x] != 0)
		{
			filtered.push_back(*it);
		}
	}
	
	features = filtered;
}

IplImage* calc_tuple_distance_map(IplImage* tuple_mask)
{
	IplImage* mask = cvCloneImage(tuple_mask);

	// create an image with tuple boundaries
	
	cvCopy(tuple_mask, mask);
	
	// calculate distance map
	IplImage* dist = cvCreateImage(cvSize(mask->width, mask->height), IPL_DEPTH_32F, 1);
	cvDistTransform(mask, dist);
	
	double minval, maxval;
	cvMinMaxLoc(dist, &minval, &maxval);

	cvReleaseImage(&mask);
	
	return dist;
}

void filter_features_distance_mask(vector<outlet_feature_t>& features, IplImage* distance_map)
{
	vector<outlet_feature_t> filtered;
	
	const double dist_factor = 0.6;
	double dist_max = 0;
	cvMinMaxLoc(distance_map, 0, &dist_max);
	double dist_min = dist_max*dist_factor;
	
	for(vector<outlet_feature_t>::const_iterator it = features.begin(); it != features.end(); it++)
	{
		CvPoint center = feature_center(*it);
		float dist = *((float*)(distance_map->imageData + distance_map->widthStep*center.y) + center.x);
		if(dist > dist_min)
		{
			filtered.push_back(*it);
		}
	}
	
	features = filtered;
}

int find_outlet_position(outlet_t outlet, IplImage* tuple_mask)
{
	int idx1 = (unsigned char)tuple_mask->imageData[outlet.hole1.y*tuple_mask->widthStep + outlet.hole1.x];
	int idx2 = (unsigned char)tuple_mask->imageData[outlet.hole2.y*tuple_mask->widthStep + outlet.hole2.x];
	
	if(idx1 == idx2)
	{
		return idx1;
	}
	else
	{
		return -1;
	}
}

void filter_outlets_tuple(vector<outlet_t>& outlets, IplImage* tuple_mask, CvPoint2D32f hor_dir)
{
	vector<outlet_t> filtered;
	vector<int> outlets_idx;
	
	for(vector<outlet_t>::const_iterator it = outlets.begin(); it != outlets.end(); it++)
	{
		int idx = find_outlet_position(*it, tuple_mask);
		outlets_idx.push_back(idx);
	}
	
	for(int i = 0; i < 4; i++)
	{
		vector<outlet_t> candidates;
		for(int j = 0; j < outlets.size(); j++)
		{
			if(outlets_idx[j] == i + 1)
			{
				candidates.push_back(outlets[j]);
			}
		}
		
		if(candidates.size() > 0)
		{
			select_orient_outlets(hor_dir, candidates, 1);
			filtered.push_back(candidates[0]);
		}
	}
	
	outlets = filtered;
}

void get_outlet_coordinates(const outlet_t& outlet, CvPoint3D32f* points)
{
	points[1] = outlet.coord_hole1;
	points[2] = outlet.coord_hole2;
	points[0] = outlet.coord_hole_ground;
}
