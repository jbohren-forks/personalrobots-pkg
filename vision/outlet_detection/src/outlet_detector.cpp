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


#include "outlet_detection/outlet_detector.h"
#include "outlet_detection/planar.h"
#include "outlet_detection/outlet_tuple.h"

#include "highgui.h"

int detect_outlet_tuple(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params, 
	vector<outlet_t>& outlets, outlet_template_t outlet_templ,
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
	
	// filter outlets using template match
	CvMat* homography = 0;
	CvPoint3D32f origin;
	CvPoint2D32f scale;
	
	homography = cvCreateMat(3, 3, CV_32FC1);
	CvMat* inv_homography = cvCreateMat(3, 3, CV_32FC1);
	
	// test the distance
	const int iter_count = 1;
	for(int j = 0; j < iter_count; j++)
	{
		calc_outlet_homography(outlet_tuple.centers, homography, 
							   outlet_templ, inv_homography);

/*		
		float sum_dist = 0;
		for(int i = 0; i < 4; i++)
		{
			vector<CvPoint2D32f> borders;
			map_vector(outlet_tuple.borders[i], homography, borders);
			CvPoint2D32f center = calc_center(borders);
			vector<CvPoint2D32f> temp;
			temp.push_back(outlet_tuple.centers[i]);
			map_vector(temp, homography, temp);
			float dist = length(temp[0] - center);
			sum_dist += dist;
						
			temp.clear();
			temp.push_back(center);
			map_vector(temp, inv_homography, temp);
			outlet_tuple.centers[i] = temp[0];
		}

		
#if defined(_VERBOSE)
		printf("Iteration %d: error %f pixels\n", j, sum_dist/4);
#endif //_VERBOSE
*/		
	}
	cvReleaseMat(&inv_homography);
    	
	calc_origin_scale(outlet_tuple.centers, homography, &origin, &scale);

	CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
	CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);
	calc_camera_pose(intrinsic_matrix, 0, outlet_tuple.centers, rotation_vector, translation_vector);
	calc_outlet_coords(outlets, homography, origin, scale, rotation_vector, translation_vector);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);

	filter_outlets_size(outlets);
	
	filter_outlets_tuple(outlets, outlet_tuple.tuple_mask, hor_dir);
    
#if defined(_VERBOSE)
	if(output_path && filename)
	{
		IplImage* temp = cvCloneImage(src);
		draw_outlets(temp, outlets);
		
		char buf[1024];
		sprintf(buf, "%s/output_filt/%s", output_path, filename);
		strcpy(buf + strlen(buf) - 3, "jpg"); 
		cvSaveImage(buf, temp);	
		
		cvReleaseImage(&temp);
	}
#endif //_VERBOSE
	
	PRINTF(" found %d holes, %d outlets\n", features.size(), outlets.size());
	
	if(homography == 0)
	{
		PRINTF("Homography mask not found.\n");
		return 0;
	}
	else if(outlets.size() != 4)
	{
        cvReleaseMat(&homography);
		PRINTF("Outlet tuple not found!\n");
		return 0;
	}
	else
	{

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

    cvReleaseMat(&homography);
	
	return 1;
}
