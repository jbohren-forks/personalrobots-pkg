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
using namespace std;

#include <cv.h>
#include <highgui.h>
#include <ml.h>

#include "learning.h"
#include "star_detector/detector.h"
#include "outlet_detector.h"

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
	char path[1024], config_filename[1024], camera_filename[1024], output_path[1024];
	if(argc < 5)
	{
		printf("Usage: outlet_model <path_to_images> <config_filename> <camera_config> <output_path>\n");
		return(0);
	}
	
	strcpy(path, argv[1]);
	strcpy(config_filename, argv[2]);
	strcpy(camera_filename, argv[3]);
	strcpy(output_path, argv[4]);

	
#if defined(_VERBOSE)
	char pathname[1024];
	sprintf(pathname, "mkdir %s", output_path);
	system(pathname);
	
	sprintf(pathname, "mkdir %s/output_filt", output_path);
	system(pathname);

	sprintf(pathname, "mkdir %s/output", output_path);
	system(pathname);
	
	sprintf(pathname, "mkdir %s/keyout", output_path);
	system(pathname);
	
	sprintf(pathname, "mkdir %s/holes", output_path);
	system(pathname);

	sprintf(pathname, "mkdir %s/warped", output_path);
	system(pathname);
#endif //_VERBOSE
		
	printf("Reading config file...\n");
	
	FILE* fp = fopen(config_filename, "rt");
	if(fp == 0)
	{
		printf("Config file not found! Exiting...\n");
		exit(1);
	}

	char buf[1024];
	int ret = 0;
	
	// reading camera params
	CvMat* intrinsic_matrix = 0;
	CvMat* distortion_params = 0; 
	LoadCameraParams(camera_filename, &intrinsic_matrix, &distortion_params);
	
	while((ret=fscanf(fp, "%s\n", buf)) > 0)
	{
		char filename[1024];
		sprintf(filename, "%s/%s", path, buf);

		IplImage* src = cvLoadImage(filename);
		if(src == 0)
		{
			printf("File %s not found, skipping...\n", filename);
			continue;
		}
		
		printf("\nImage %s:", buf);

		int64 t1 = cvGetTickCount();
		
		vector<outlet_t> outlets;
		int ret = detect_outlet_tuple(src, intrinsic_matrix, distortion_params, outlets, output_path, buf);
		
#if defined(_VERBOSE)
		IplImage* temp = cvCloneImage(src);
		draw_outlets(temp, outlets);
		
		sprintf(filename, "%s/output_filt/%s", output_path, buf);
		strcpy(filename + strlen(filename) - 3, "jpg"); 
		cvSaveImage(filename, temp);	
		
		cvReleaseImage(&temp);
#endif //_VERBOSE

		if(ret)
		{
#if defined(_VERBOSE)
		//FIND 3D TO OUTLETS
			CvPoint3D32f holes[3];
			get_outlet_coordinates(outlets[0], holes);
			// holes[0] contains ground hole
			// holes[1] contains left hole
			// hole[2] contains right hole
			printf("Ground: (%f, %f, %f)\n",holes[0].x,holes[0].y,holes[0].z);
			printf("Left:   (%f, %f, %f)\n",holes[1].x,holes[1].y,holes[1].z);
			printf("Right:  (%f, %f, %f)\n",holes[2].x,holes[2].y,holes[2].z);  
			
			printf("Ground1: (%f, %f, %f)\n", outlets[1].coord_hole_ground.x, outlets[1].coord_hole_ground.y,
				   outlets[1].coord_hole_ground.z);
			printf("Ground2: (%f, %f, %f)\n", outlets[2].coord_hole_ground.x, outlets[2].coord_hole_ground.y,
				   outlets[2].coord_hole_ground.z);
			
			printf(" found %d outlets\n", outlets.size());
#endif
		}
		else
		{
			printf("Failed to find outlet tuple.\n");
		}
		
		printf(" done.\n");
		
		int64 t2 = cvGetTickCount();
		printf("Time elapsed: %f\n", double(t2 - t1)/cvGetTickFrequency()*1e-6);
		
		cvReleaseImage(&src);
	}
		
	return 0;
}


