/*
 *  outlet_model.h
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 12/22/08.
 *  Copyright 2008 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#if !defined(_OUTLET_MODEL_H)
#define _OUTLET_MODEL_H

typedef struct 
{
	CvRect bbox;
	float weight;
} outlet_feature_t;

inline CvPoint feature_center(outlet_feature_t feature)
{
	return cvPoint(feature.bbox.x + feature.bbox.width/2, feature.bbox.y + feature.bbox.height/2);
}

typedef struct
{
	CvSeq* outlet;
	CvPoint hole1;
	CvPoint hole2;
	outlet_feature_t feature1;
	outlet_feature_t feature2;
	CvPoint3D32f coord_hole1;
	CvPoint3D32f coord_hole2;
	float weight;
	float weight_orient;
} outlet_t;

inline float outlet_size(outlet_t outlet);
CvRect outlet_rect(outlet_t outlet);

#define __max MAX 
#define __min MIN 

void detect_outlets(IplImage* src, vector<outlet_feature_t>& features, vector<outlet_t>& outlets, 
					const char* output_path, const char* filename = 0);

inline void cvRectangle(IplImage* img, CvRect rect, CvScalar color, int thickness)
{
	cvRectangle(img, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height), 
				color, thickness);
}

inline CvRect fit_rect_roi(CvRect rect, CvRect roi)
{
	CvRect fit = rect;
	fit.x = MAX(fit.x, roi.x);
	fit.y = MAX(fit.y, roi.y);
	fit.width = MIN(fit.width, roi.x + roi.width - fit.x - 1);
	fit.height = MIN(fit.height, roi.y + roi.height - fit.y - 1);
	assert(fit.width > 0);
	assert(fit.height > 0);
	return(fit);
}

inline CvRect fit_rect(CvRect rect, IplImage* img)
{
	CvRect roi = cvGetImageROI(img);
	return fit_rect_roi(rect, roi);
}

inline CvRect double_rect(CvRect small_rect)
{
	return cvRect(small_rect.x - small_rect.width/2, small_rect.y - small_rect.height/2,
				  small_rect.width*2, small_rect.height*2);
}

inline CvRect resize_rect(CvRect rect, float alpha)
{
	return cvRect(rect.x + int(0.5*(1 - alpha)*rect.width), rect.y + int(0.5*(1 - alpha)*rect.height), 
				  int(rect.width*alpha), int(rect.height*alpha));
}

inline int is_point_inside_rect(CvRect rect, CvPoint point)
{
	if(point.x >= rect.x && point.y >= rect.y && 
	   point.x <= rect.x + rect.width && point.y <= rect.y + rect.height)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int is_point_incenter_roi(const vector<CvRect>& rects, CvPoint point);

void DrawKeypoints(IplImage* img, std::vector<Keypoint> keypts);

typedef map<string, vector<CvRect> > outlet_roi_t;
CvMat* vector2mat(const vector<int>& vec);
void read_outlet_roi(const char* filename, outlet_roi_t& outlet_roi);
void extract_intensity_features(IplImage* grey, const vector<outlet_feature_t>& keypts, CvMat** mat, 
								int equalize_hist = 0,
								const vector<int>& labels = vector<int>(), const char* buf = 0);
void calc_labels(const vector<CvRect>& rects, const vector<Keypoint>& keypts, vector<int>& labels);
void calc_labels(const vector<CvRect>& rects, const vector<outlet_feature_t>& keypts, vector<int>& labels);
void FilterPoints(IplImage* grey, vector<outlet_feature_t>& keypts, const CvRTrees* rtrees);
void filter_outlets(IplImage* grey, vector<outlet_t>& outlets, CvRTrees* rtrees);

void outletfarr2keypointarr(const vector<outlet_feature_t>& features, vector<Keypoint>& keypoints);
void keypointarr2outletfarr(const vector<Keypoint>& keypoints, vector<outlet_feature_t>& features);

void find_outlet_features(IplImage* src, vector<outlet_feature_t>& features, const char* filename);
void find_outlet_features_fast(IplImage* src, vector<outlet_feature_t>& features, const char* output_path, const char* filename);

// generates several perspective distortions of the original outlet and 
// extracts intensity values into CvMat format for subsequent learning of a classifier
int generate_outlet_samples(IplImage* grey, outlet_t outlet, int count, CvMat** predictors, const char* filename = 0);

// train an outlet model by generating perspective distorted outlets
void train_outlet_model(const char* path, const char* config_filename, 
						const char* roi_filename, const char* forest_filename);

void write_pr(const char* pr_filename, const char* image_filename, const outlet_roi_t& outlet_roi, 
			  const vector<outlet_t>& outlets);

void filter_negative_samples(const vector<CvRect>& rects, vector<outlet_feature_t>& keypts, float fraction);
void calc_contrast_factor(IplImage* grey, CvRect rect, float& contrast, float& variation);

void select_orient_outlets(CvPoint2D32f orientation, vector<outlet_t>& outlets, int count = 0);
int select_orient_outlets_ex(IplImage* grey, vector<outlet_t>& outlets, const char* filename = 0);

void draw_outlets(IplImage* temp, const vector<outlet_t>& outlets);
void filter_canny(IplImage* grey, vector<outlet_feature_t>& features);

// a temporary solution for finding image homography
int load_homography_map(const char* filename, CvMat** map_matrix);
IplImage* load_match_template_mask(const char* filename);

CvMat* calc_image_homography(IplImage* src);


// uses template matching, currently done offline
void filter_outlets_templ_ex(vector<outlet_t>& outlets, CvMat* map_matrix, IplImage* mask);
int filter_outlets_templ(vector<outlet_t>& outlets, const char* filename);
int filter_outlets_templmatch(IplImage* src, vector<outlet_t>& outlets, IplImage* templ, const char* output_path, 
							  const char* filename = 0, CvMat** homography = 0, CvPoint3D32f* origin = 0, CvPoint2D32f* scale = 0);

IplImage* calc_tuple_distance_map(IplImage* tuple_mask);


int calc_outlet_coords(vector<outlet_t>& outlets, CvMat* map_matrix, CvPoint3D32f origin, CvPoint2D32f scale);
void calc_outlet_dist_stat(const vector<outlet_t>& outlets, float& mean, float& stddev);

void filter_features_mask(vector<outlet_feature_t>& features, IplImage* mask);
void filter_outlets_mask(vector<outlet_t>& outlets, IplImage* mask);
void filter_features_distance_mask(vector<outlet_feature_t>& features, IplImage* distance_map);

int find_origin_chessboard(IplImage* src, CvMat* map_matrix, CvPoint3D32f& origin, float bar_length);

#endif //_OUTLET_MODEL_H
