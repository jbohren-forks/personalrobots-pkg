/*
 *  outlet_tuple.h
 *  rectify_outlets
 *
 *  Created by Victor  Eruhimov on 1/25/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

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

inline CvPoint2D32f operator +(CvPoint2D32f p1, CvPoint2D32f p2)
{
	return cvPoint2D32f(p1.x + p2.x, p1.y + p2.y);
}

inline CvPoint2D32f operator -(CvPoint2D32f p1, CvPoint2D32f p2)
{
	return cvPoint2D32f(p1.x - p2.x, p1.y - p2.y);
}

inline CvPoint3D32f operator -(CvPoint3D32f p1, CvPoint3D32f p2)
{
	return cvPoint3D32f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

inline float length(const CvPoint2D32f& p)
{
	return sqrt(p.x*p.x + p.y*p.y);
}

inline float length(const CvPoint3D32f& p)
{
	return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

struct outlet_tuple_t
{
	CvPoint2D32f centers[4];
	vector<CvPoint2D32f> borders[4];
	IplImage* tuple_mask;
	CvRect roi;
	
	~outlet_tuple_t()
	{
		cvReleaseImage(&tuple_mask);
	}
};

class outlet_template_t
{
public:
	outlet_template_t(int count = 4, const CvPoint2D32f* templ = 0)
	{
		initialize(count, templ);
	};
	
	outlet_template_t(const outlet_template_t& outlet_templ)
	{
		initialize(outlet_templ.get_count(), outlet_templ.get_template());
	};
	
	~outlet_template_t() 
	{
		delete []centers;
	};
	
public:
	void initialize(int count, const CvPoint2D32f* templ)
	{
		outlet_count = count;
		centers = new CvPoint2D32f[count];
		if(templ)
		{
			memcpy(centers, templ, count*sizeof(CvPoint2D32f));
		}
		else
		{
			set_default_template();
		}
	};
	
	int get_count() const
	{
		return outlet_count;
	};
	
	const CvPoint2D32f* get_template() const
	{
		return centers;
	};
	
	void set_default_template()
	{
		centers[0] = cvPoint2D32f(0.0f, 0.0f);
		centers[1] = cvPoint2D32f(46.0f, 0.0f);
		centers[2] = cvPoint2D32f(46.15f, 38.7f);
		centers[3] = cvPoint2D32f(-0.15f, 38.7f);
	};
    
    void save(const char* filename);
    int load(const char* filename);
	
protected:
	int outlet_count;
	CvPoint2D32f* centers;
};

typedef struct 
{
	CvPoint2D32f center;
	float angle;
	int idx;
	CvSeq* seq;
} outlet_elem_t;


CvPoint2D32f calc_center(CvSeq* seq);
int find_dir(const CvPoint2D32f* dir, int xsign, int ysign);
int order_tuple(CvPoint2D32f* centers);
int order_tuple2(vector<outlet_elem_t>& tuple);

void map_vector(const vector<CvPoint2D32f>& points, CvMat* homography, vector<CvPoint2D32f>& result);
CvPoint2D32f calc_center(const vector<CvPoint2D32f>& points);



// find_outlet_centroids: finds a tuple of 4 orange outlets.
// Input parameters:
//	src: input color image
//	outlet_tuple: the output outlet tuple
//	output_path: a path for logging the results
//	filename: filename for logging the results
// Returns 1 if a tuple is found, 0 otherwise.
int find_outlet_centroids(IplImage* img, outlet_tuple_t& outlet_tuple, const char* output_path, const char* filename);


int find_tuple(vector<outlet_elem_t>& candidates, CvPoint2D32f* centers);

// calc_origin_scale: function for calculating the 3D position of origin (center of the upper left outlet)
// and scale (pixels to mm in rectified image)
// Input parameters:
//	centers: tuple outlet centers (an array with 4 elements)
//	map_matrix: homography matrix from a camera image to a rectified image
//	origin: output coordinates of origin
//	scale: output scale in horizontal and vertical direction. A vector p = (p.x, p.y) in a rectified image
//		in outlets plane corresponds to p1 = (p.x*scale.x, p.y*scale.y, 0) in 3D.
void calc_origin_scale(const CvPoint2D32f* centers, CvMat* map_matrix, CvPoint3D32f* origin, CvPoint2D32f* scale);

// calc_outlet_homography: calculates homography for rectifying the outlet tuple image
// Input parameters:
//	centers: outlet centers
//	map_matrix: output homography matrix (map from a camera image to a rectified image)
//  templ: outlet template
//	inverse_map_matrix: optional inverse homography matrix
void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix, 
							outlet_template_t templ = outlet_template_t(), 
							CvMat* inverse_map_matrix = 0);

void calc_outlet_homography(const CvPoint2D32f* centers, CvSize src_size, CvMat* map_matrix, CvSize* dst_size);

int calc_image_homography(IplImage* src, CvMat* map_matrix, CvSize* dst_size, CvPoint2D32f* hor_dir = 0, 
						  CvPoint3D32f* origin = 0, CvPoint2D32f* scale = 0, const char* output_path = 0, 
						  const char* filename = 0, CvPoint2D32f* _centers = 0);
IplImage* find_templates(IplImage* img, IplImage* templ);

void calc_bounding_rect(int count, const CvRect* rects, CvRect& bounding_rect);


// calc_camera_pose: pose estimation function. Assumes that camera calibration was done with real length.
// Input parameters:
//	intrinsic_mat: matrix of camera intrinsic parameters
//	distortion_coeffs: vector of distortion coefficients. If 0, all coefficients are assumed 0.
//	centers: outlet centers
//	rotation_vector, translation_vector: output transformation from outlet coordinate system (origin in the 
//	center of the upper left outlet) to camera coordinate system
void calc_camera_pose(CvMat* intrinsic_mat, CvMat* distortion_coeffs, CvPoint2D32f* centers, 
					  CvMat* rotation_vector, CvMat* translation_vector);

#endif //_OUTLET_TUPLE_H
