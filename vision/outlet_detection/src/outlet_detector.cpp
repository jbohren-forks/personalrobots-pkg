/*
 *  outlet_detector.cpp
 *  outlet_sample
 *
 *  Created by Victor  Eruhimov on 2/19/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include <stdio.h>
#include <stdarg.h>

#if defined(_VERBOSE)
static int PRINTF( const char* fmt, ... )
{
    va_list args;
    va_start(args, fmt);
    int ret = vprintf(fmt, args);
	return ret;
}
#else
static int PRINTF( const char*, ... )
{
    return 0;
}
#endif // _VERBOSE


#include "outlet_detector.h"
#include "planar.h"
#include "outlet_tuple.h"

#include "highgui.h"

int detect_outlet_tuple(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params, vector<outlet_t>& outlets,
	const char* output_path, const char* filename)
{
    if (distortion_params) {
        // correcting for distortion
        IplImage* _img = cvCloneImage(src);
        //		int64 _t1 = cvGetTickCount();
        cvUndistort2(_img, src, intrinsic_matrix, distortion_params);
        //		int64 _t2 = cvGetTickCount();
        //		printf("Undistort time elapsed: %f", double(_t2 - _t1)/cvGetTickFrequency()*1e-6);
        cvReleaseImage(&_img);
    }
	
	outlet_tuple_t outlet_tuple;
	
	outlet_tuple.tuple_mask = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	int ret = find_outlet_centroids(src, outlet_tuple, output_path, filename);
	if(!ret)
	{
		PRINTF("find_outlet_centroids did not find a tuple\n");
		return 0;
	}
	
	vector<outlet_feature_t> features;	
	detect_outlets(src, features, outlets, &outlet_tuple, output_path, filename);
	
	CvPoint2D32f hor_dir = outlet_tuple.centers[1] - outlet_tuple.centers[0];
	//	select_orient_outlets(hor_dir, outlets, 4);
	filter_outlets_tuple(outlets, outlet_tuple.tuple_mask, hor_dir);
	
	// filter outlets using template match
	CvMat* homography = 0;
	CvPoint3D32f origin;
	CvPoint2D32f scale;
	
	homography = cvCreateMat(3, 3, CV_32FC1);
	calc_outlet_homography(outlet_tuple.centers, homography, 0);
	calc_origin_scale(outlet_tuple.centers, homography, &origin, &scale);
	
#if defined(_VERBOSE)
	IplImage* temp = cvCloneImage(src);
	draw_outlets(temp, outlets);
	
	char buf[1024];
	sprintf(buf, "%s/output_filt/%s", output_path, filename);
	strcpy(buf + strlen(buf) - 3, "jpg"); 
	cvSaveImage(buf, temp);	
	
	cvReleaseImage(&temp);
#endif //_VERBOSE
	
	PRINTF(" found %d holes, %d outlets\n", features.size(), outlets.size());
	
	if(homography == 0)
	{
		PRINTF("Homography mask not found.\n");
		return 0;
	}
	else if(outlets.size() != 4)
	{
		PRINTF("Outlet tuple not found!\n");
		return 0;
	}
	else
	{
		CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
		CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);
		calc_camera_pose(intrinsic_matrix, 0, outlet_tuple.centers, rotation_vector, translation_vector);
		
		calc_outlet_coords(outlets, homography, origin, scale, rotation_vector, translation_vector);

#if defined(_VERBOSE)
		float mean, stddev;
		calc_outlet_dist_stat(outlets, mean, stddev);
		PRINTF("Distance between holes: Mean = %f, stddev = %f\n", mean, stddev);
		
		float ground_dist_x1, ground_dist_x2, ground_dist_y;
		calc_outlet_tuple_dist_stat(outlets, ground_dist_x1, ground_dist_x2, ground_dist_y);
		PRINTF("Horizontal distance between ground holes: top %f, bottom %f\n", 
			   ground_dist_x1, ground_dist_x2);
		PRINTF("Vertical distance between ground holes: %f\n", ground_dist_y);
#endif // _VERBOSE
	}	
	
	return 1;
}
