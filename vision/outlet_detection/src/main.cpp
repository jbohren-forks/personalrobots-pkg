/*
 *  main.cpp
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 1/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#include <stdio.h>
#include <sys/stat.h>

#include <vector>
#include <map>
using namespace std;

#include <cv.h>
#include <highgui.h>
#include <ml.h>

#include "learning.h"
#include "keypoint/detector.h"
#include "outlet_model.h"
#include "outlet_tuple.h"
#include "planar.h"

int LoadCameraParams(char* filename, CvMat** intrinsic_matrix, CvMat** distortion_coeffs)
{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==NULL) return 0;
	
    *intrinsic_matrix = (CvMat*)cvReadByName( fs,0,"camera_matrix");
	*distortion_coeffs = (CvMat*)cvReadByName( fs,0,"distortion_coefficients");
	
	return 1;
}

int main(int argc,char** argv)
{
	char path[1024], config_filename[1024], template_filename[1024], camera_filename[1024], roi_filename[1024], output_path[1024];
	if(argc < 6)
	{
		printf("Usage: outlet_model <path_to_images> <config_filename> <template_filename> <camera_config> <output_path> [<roi_filename>]\n");
		return(0);
	}
	
	strcpy(path, argv[1]);
	strcpy(config_filename, argv[2]);
	strcpy(template_filename, argv[3]);
	strcpy(camera_filename, argv[4]);
	strcpy(output_path, argv[5]);
	if(argc > 6)
	{
		strcpy(roi_filename, argv[6]);
	}
	else
	{
		strcpy(roi_filename, "");
	}
	
	char pathname[1024];
	sprintf(pathname, "mkdir %s", output_path);
	system(pathname);
	
	sprintf(pathname, "mkdir %s/output_filt", output_path);
	system(pathname);
	
#if defined(_VERBOSE)
	sprintf(pathname, "mkdir %s/output", output_path);
	system(pathname);
	
	sprintf(pathname, "mkdir %s/keyout", output_path);
	system(pathname);
	
	sprintf(pathname, "mkdir %s/holes", output_path);
	system(pathname);

	sprintf(pathname, "mkdir %s/warped", output_path);
	system(pathname);
#endif //_VERBOSE
	
#if defined(_LEARN_OUTLETS)
	train_outlet_model("../../../images/train", "../../../images/config_outlets_train.txt", 
					   "../../../images/config_outlets_roi_train.txt", "../../outlet_forest.xml");
	return 0;
#endif //_LEARN_OUTLETS
	
/*
#if defined(_EUROPE)
#if defined(_TRAIN)
	const char path[] = "../../../images/train";
	const char config_filename[] = "../../../images/config_train.csv";
	const char roi_filename[] = "../../../images/config_roi_train.csv";
#else
	const char path[] = "../../../images/test_bmp";
	const char config_filename[] = "../../../images/config_bmp.csv"; 
	const char roi_filename[] = "../../../images/config_roi_bmp.csv";
#endif //_TRAIN
#else
	//const char path[] = "../../../data/OutletWhiteHall";
	const char path[] = "../../../images/us_outlets_hr";
//	const char config_filename[] = "../../../data/OutletWhiteHall/config.csv";
	const char config_filename[] = "../../../images/config_us_outlets_hr.txt";
	const char roi_filename[] = "../../../images/config_roi_train.csv";
#endif //EUROPE
*/
	
	printf("Reading config file...\n");
	
	FILE* fp = fopen(config_filename, "rt");
	if(fp == 0)
	{
		printf("Config file not found! Exiting...\n");
		exit(1);
	}
	//printf("File opened, %d...\n", fp);

	char buf[1024];
	int ret = 0;
	
	outlet_roi_t outlet_roi;
	if(strlen(roi_filename) > 0)
	{
		// read outlet ROI
		printf("Reading outlet roi...");
		read_outlet_roi(roi_filename, outlet_roi);
		printf("done.\n");
	}
	
	// reading camera params
	CvMat* intrinsic_matrix = 0;
	CvMat* distortion_params = 0; 
	LoadCameraParams(camera_filename, &intrinsic_matrix, &distortion_params);
	
	IplImage* templ = cvLoadImage(template_filename);
	if(!templ)
	{
		printf("Template not found! Exiting...\n");
		return(0);
	}
	
	vector<int> labels;
	
	vector<Keypoint> _temp;
#if defined(_TRAIN)
	CvMat* predictors = 0;
	IplImage* grtemp = 0;
#endif
	
	while((ret=fscanf(fp, "%s\n", buf)) > 0)
	{
		char filename[1024];
		sprintf(filename, "%s/%s", path, buf);

#if defined(_SMALL)
		IplImage* _src = cvLoadImage(filename);
		if(_src == 0)
		{
			printf("File %s not found, skipping...\n", filename);
			continue;
		}
		
		IplImage* src = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
		cvResize(_src, src);
		cvReleaseImage(&_src);
#else
		IplImage* src = cvLoadImage(filename);
		if(src == 0)
		{
			printf("File %s not found, skipping...\n", filename);
			continue;
		}
		
		IplImage* _img = cvCloneImage(src);
		cvUndistort2(_img, src, intrinsic_matrix, distortion_params);
		cvReleaseImage(&_img);
		
#endif //_SMALL
		printf("Image %s:", buf);
		
		IplImage* grey = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
		cvCvtColor(src, grey, CV_RGB2GRAY);

		cvSmooth(grey, grey);
		
		
		std::vector<Keypoint> keypts;
#if 1
		vector<outlet_feature_t> features;
		vector<outlet_t> outlets;
 		detect_outlets(src, features, outlets, output_path, buf);
		
		// filter outlets using template match
		CvMat* homography = 0;
		CvPoint3D32f origin;
		CvPoint2D32f scale;
		
#if defined(_USE_TEMPL_MATCH)
		filter_outlets_templmatch(src, outlets, templ, output_path, buf, &homography, &origin, &scale);		
#else
		homography = cvCreateMat(3, 3, CV_32FC1);
		int ret = calc_image_homography(src, homography, 0, 0, &origin, &scale, output_path, buf);
		if(!ret)
		{
			cvReleaseMat(&homography);
			homography = 0;
		}
#endif //_USE_TEMPL_MATCH
				
#if defined(_VERBOSE)
		IplImage* temp = cvCloneImage(src);
		draw_outlets(temp, outlets);
		
		sprintf(filename, "%s/output_filt/%s", output_path, buf);
		strcpy(filename + strlen(filename) - 3, "jpg"); 
		cvSaveImage(filename, temp);	
		
		cvReleaseImage(&temp);
#endif //_VERBOSE
		
		if(strlen(roi_filename) > 0)
		{
			sprintf(filename, "%s/prec_rec.txt", output_path);
			write_pr(filename, buf, outlet_roi, outlets);
		}
		printf(" found %d holes, %d outlets\n", features.size(), outlets.size());
		
		outletfarr2keypointarr(features, keypts);
		
		if(strcmp(buf, "PB210031.jpg") == 0)
		{
			CvRect rect = outlet_rect(outlets[1]);
			rect = double_rect(rect);
			cvSetImageROI(grey, rect);
			test_homog_transform(grey);
			cvResetImageROI(grey);
		}
#else
		// Find keypoints
		StarDetector detector(cvSize(src->width, src->height), 7, 50); // use default parameters
		detector.DetectPoints(grey, std::back_inserter(keypts));
		
		// Retain the 800 strongest features
		static const int NUM_POINTS = 400;
		KeepBestPoints(keypts, NUM_POINTS);
#endif
		
#if defined(_VERBOSE)
		DrawKeypoints(src, keypts);
		sprintf(filename, "%s/keyout/%s", output_path, buf);
		cvSaveImage(filename, src);
#endif //_VERBOSE
		
#if defined(_TRAIN)		
		// now calculate labels and store image patches for RF training/prediction
		filter_negative_samples(outlet_roi[buf], features, 0.1f);
		printf("Left %d keypoints\n", features.size());
		
		calc_labels(outlet_roi[buf], features, labels);
		extract_intensity_features(grey, features, &predictors, 0, labels, buf);
		
		cvReleaseImage(&grtemp);
		grtemp = cvCloneImage(grey);
		_temp = keypts;
#endif //_TRAIN
		
		cvReleaseImage(&grey);

#if 0
		CvPoint2D32f corners[6*9];
		int corner_count = 0;
		int ret = cvFindChessboardCorners(src, cvSize(6, 9), corners, &corner_count);
		printf("find chessboard returned %d\n", ret);
		if(corner_count > 0)
		{
			cvDrawChessboardCorners(src, cvSize(6, 9), corners, 6*9, ret);
			cvNamedWindow("1", 1);
			cvShowImage("1", src);
			cvWaitKey(0);
		}
#endif
		
		if(homography == 0)
		{
			printf("Homography mask not found.\n");
		}
		else
		{
			calc_outlet_coords(outlets, homography, origin, scale);
			float mean, stddev;
			calc_outlet_dist_stat(outlets, mean, stddev);
			printf("Distance between holes: Mean = %f, stddev = %f\n", mean, stddev);
			/*		
			 CvPoitn3D32f origin;
			 float bar_length;
			 find_origin_chessboard(src, map_matrix, origin, bar_length);
			 */
		}
		printf(" done.\n");
		
		cvReleaseImage(&src);
	}
	
#if defined(_TRAIN)
	cvSave("../../predictors.csv", predictors);
	CvMat* labels_mat = vector2mat(labels);
	cvSave("../../labels.csv", labels_mat);
	
	printf("Training RF model...");
	CvRTrees* rtrees = train_rf(predictors, labels_mat); 
	printf("done.\n");
	
	//	FilterPoints(grtemp, _temp, rtrees);
	
	rtrees->save("../../forest.xml");
#endif //_TRAIN
	
	return 0;
}


