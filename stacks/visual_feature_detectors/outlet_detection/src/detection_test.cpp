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

#include <cv.h>
#include <highgui.h>

#include "outlet_detection/outlet_detector_test.h"

#define ONE_WAY_DETECTOR 0
#define L_DETECTOR 1
#define FERNS_L_DETECTOR 2
#define FERNS_ONE_WAY_DETECTOR 3


int _LoadCameraParams(char* filename, CvMat** intrinsic_matrix, CvMat** distortion_coeffs)
{

	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==NULL) return 0;
	
    *intrinsic_matrix = (CvMat*)cvReadByName( fs,0,"camera_matrix");
	*distortion_coeffs = (CvMat*)cvReadByName( fs,0,"distortion_coefficients");
    
    cvReleaseFileStorage(&fs);
	
	return 1;
}
//---------------
int run_l_detector(int argc,char** argv, TestResult* result = 0)
{
	char mode[1024], config_filename[1024], camera_filename[1024], output_path[1024], 
        test_path[1024], train_config[1024], pca_config[1024], output_test_config[1024];
	int accuracy = 5;


	if (result)
	{
		result->correct = 0;
		result->skipped = 0;
		result->total = 0;
		result->incorrect = 0;
	}
	
    
	if(argc != 8 && argc != 9)
	{
		return -1;
	}
    
	
	strcpy(mode, argv[2]);
	strcpy(test_path, argv[3]);
	strcpy(config_filename, argv[4]);
	strcpy(camera_filename, argv[5]);
    
    
    outlet_template_t outlet_template;
    if(argc == 8)
    {
        strcpy(output_path, argv[7]);
    }
    else
    {
        strcpy(train_config, argv[6]);
        strcpy(output_path, argv[7]);
    }

	if ((strcmp(mode,"test")==0) )
	{
		sscanf(argv[argc-1],"%d",&accuracy);
	}
	else
	{
		strcpy(output_test_config,argv[argc-1]);
	}
	
		// reading camera params
	CvMat* intrinsic_matrix = 0;
	CvMat* distortion_params = 0; 
	_LoadCameraParams(camera_filename, &intrinsic_matrix, &distortion_params);

	vector<outlet_test_elem> test_data;

	if (readTestFile(config_filename,test_path,test_data) > 0)
	{
		if (strcmp(mode,"modify")==0)
		{
			//Code for modify mode
			printf("Use <Space> and <Backspace> to get next and previous image\n");
			printf("Press <Enter> key to enter modification mode\n");
			printf("Press <Esc> key to approve outlet positions and start the test\n");
			char window_name[100];
			strcpy(window_name,"Image");

			int key;
			int i = 0;
			bool isEnd = false;
			IplImage* img = getRealOutletImage(test_data[0],intrinsic_matrix, distortion_params);
			cvNamedWindow(window_name);
			cvShowImage(window_name,img);
			while (!isEnd)
			{
				key = cvWaitKey();
				switch(key)
				{
					case 13:
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						setRealOutlet(test_data[i],intrinsic_matrix, distortion_params);
						img = getRealOutletImage(test_data[i],intrinsic_matrix, distortion_params);
						cvNamedWindow(window_name);
						cvShowImage(window_name,img);
						break;
					case 27: 
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						isEnd = true;
						break;
					case 32://SPACE
						if (i<((size_t)test_data.size()-1))
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[++i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The last image. Unable to get the next one\n");
						}
						break;
					case 8://Backspace
						if (i>0)
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[--i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The first image. Unable to get the previous one\n");
						}
						break;
				}
			}
		}
    
	    
	#if defined(_VERBOSE)
		char pathname[1024];
		sprintf(pathname, "mkdir %s", output_path);
		system(pathname);
	    
		sprintf(pathname, "mkdir %s/correspondence", output_path);
		system(pathname);

		sprintf(pathname, "mkdir %s/outlets", output_path);
		system(pathname);
	        
		sprintf(pathname, "mkdir %s/features", output_path);
		system(pathname);

	#endif //_VERBOSE
			

		//Running the test
		if (strcmp(mode,"modify")==0)
		{
			writeTestFile(output_test_config,test_data);
			printf("New test config was successfully generated into %s\n",output_test_config);
		}
		else
			runLOutletDetectorTest(intrinsic_matrix, distortion_params, train_config, test_data, output_path);


		if (strcmp(mode,"generate")==0)
		{
			convertTestToReal(test_data);
			writeTestFile(output_test_config,test_data);
			printf("New test config was successfully generated into %s\n",output_test_config);
		}

		if (strcmp(mode,"test")==0)
		{

			compareAllOutlets(test_data,accuracy);
			char filename[1024];
			if (output_path)
				sprintf(filename,"%s/results.txt",output_path);
			else
				sprintf(filename,"results.txt",output_path);
			writeTestResults(filename,test_data,result);
			printf ("Test results were successfully written into %s\n", filename);
		}
	    
		  
		cvReleaseMat(&intrinsic_matrix);
		cvReleaseMat(&distortion_params);
	        
	}
	else
	{
		printf("Unable to read test configuration file\n");
	}
		
	return 0;
}

//---------------
int run_ferns_l_detector(int argc,char** argv, TestResult* result = 0)
{
	char detector[1024];
	char mode[1024], config_filename[1024], camera_filename[1024], output_path[1024], 
        test_path[1024], train_config[1024], pca_config[1024], output_test_config[1024];
	int accuracy = 5;


	if (result)
	{
		result->correct = 0;
		result->skipped = 0;
		result->total = 0;
		result->incorrect = 0;
	}
	
    
	if(argc != 8 && argc != 9)
	{
		return -1;
	}
    
	
	strcpy(mode, argv[2]);
	strcpy(test_path, argv[3]);
	strcpy(config_filename, argv[4]);
	strcpy(camera_filename, argv[5]);
    
    
    outlet_template_t outlet_template;
    if(argc == 8)
    {
        strcpy(output_path, argv[7]);
    }
    else
    {
        strcpy(train_config, argv[6]);
        strcpy(output_path, argv[7]);
    }

	if ((strcmp(mode,"test")==0) )
	{
		sscanf(argv[argc-1],"%d",&accuracy);
	}
	else
	{
		strcpy(output_test_config,argv[argc-1]);
	}
	
		// reading camera params
	CvMat* intrinsic_matrix = 0;
	CvMat* distortion_params = 0; 
	_LoadCameraParams(camera_filename, &intrinsic_matrix, &distortion_params);

	vector<outlet_test_elem> test_data;

	if (readTestFile(config_filename,test_path,test_data) > 0)
	{
		if (strcmp(mode,"modify")==0)
		{
			//Code for modify mode
			printf("Use <Space> and <Backspace> to get next and previous image\n");
			printf("Press <Enter> key to enter modification mode\n");
			printf("Press <Esc> key to approve outlet positions and start the test\n");
			char window_name[100];
			strcpy(window_name,"Image");

			int key;
			int i = 0;
			bool isEnd = false;
			IplImage* img = getRealOutletImage(test_data[0],intrinsic_matrix, distortion_params);
			cvNamedWindow(window_name);
			cvShowImage(window_name,img);
			while (!isEnd)
			{
				key = cvWaitKey();
				switch(key)
				{
					case 13:
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						setRealOutlet(test_data[i],intrinsic_matrix, distortion_params);
						img = getRealOutletImage(test_data[i],intrinsic_matrix, distortion_params);
						cvNamedWindow(window_name);
						cvShowImage(window_name,img);
						break;
					case 27: 
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						isEnd = true;
						break;
					case 32://SPACE
						if (i<((size_t)test_data.size()-1))
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[++i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The last image. Unable to get the next one\n");
						}
						break;
					case 8://Backspace
						if (i>0)
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[--i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The first image. Unable to get the previous one\n");
						}
						break;
				}
			}
		}
    
	    
	#if defined(_VERBOSE)
		char pathname[1024];
		sprintf(pathname, "mkdir %s", output_path);
		system(pathname);

		sprintf(pathname, "mkdir %s/correspondence", output_path);
		system(pathname);

		sprintf(pathname, "mkdir %s/outlets", output_path);
		system(pathname);
	        
		sprintf(pathname, "mkdir %s/features", output_path);
		system(pathname);

	#endif //_VERBOSE
			


		//Running the test
		if (strcmp(mode,"modify")==0)
		{
			writeTestFile(output_test_config,test_data);
			printf("New test config was successfully generated into %s\n",output_test_config);
		}
		else
			runFernsLOutletDetectorTest(intrinsic_matrix, distortion_params, train_config, test_data, output_path);


		if (strcmp(mode,"generate")==0)
		{
			convertTestToReal(test_data);
			writeTestFile(output_test_config,test_data);
			printf("New test config was successfully generated into %s\n",output_test_config);
		}

		if (strcmp(mode,"test")==0)
		{

			compareAllOutlets(test_data,accuracy);
			char filename[1024];
			if (output_path)
				sprintf(filename,"%s/results.txt",output_path);
			else
				sprintf(filename,"results.txt",output_path);
			writeTestResults(filename,test_data,result);
			printf ("Test results were successfully written into %s\n", filename);
		}
	    
		  
		cvReleaseMat(&intrinsic_matrix);
		cvReleaseMat(&distortion_params);
	        
	}
	else
	{
		printf("Unable to read test configuration file\n");
	}
		
	return 0;
}
//---------------
int run_ferns_one_way_detector(int argc,char** argv, TestResult* result = 0)
{
	char detector[1024];
	char mode[1024], config_filename[1024], camera_filename[1024], output_path[1024], 
        test_path[1024], train_config[1024], pca_config[1024], output_test_config[1024];
	int accuracy = 5;


	if (result)
	{
		result->correct = 0;
		result->skipped = 0;
		result->total = 0;
		result->incorrect = 0;
	}
	
    
	if(argc != 8 && argc != 9)
	{
		return -1;
	}
    
	
	strcpy(mode, argv[2]);
	strcpy(test_path, argv[3]);
	strcpy(config_filename, argv[4]);
	strcpy(camera_filename, argv[5]);
    
    
    outlet_template_t outlet_template;
    if(argc == 8)
    {
        strcpy(output_path, argv[7]);
    }
    else
    {
        strcpy(train_config, argv[6]);
        strcpy(output_path, argv[7]);
    }

	if ((strcmp(mode,"test")==0) )
	{
		sscanf(argv[argc-1],"%d",&accuracy);
	}
	else
	{
		strcpy(output_test_config,argv[argc-1]);
	}
	
		// reading camera params
	CvMat* intrinsic_matrix = 0;
	CvMat* distortion_params = 0; 
	_LoadCameraParams(camera_filename, &intrinsic_matrix, &distortion_params);

	vector<outlet_test_elem> test_data;

	if (readTestFile(config_filename,test_path,test_data) > 0)
	{
		if (strcmp(mode,"modify")==0)
		{
			//Code for modify mode
			printf("Use <Space> and <Backspace> to get next and previous image\n");
			printf("Press <Enter> key to enter modification mode\n");
			printf("Press <Esc> key to approve outlet positions and start the test\n");
			char window_name[100];
			strcpy(window_name,"Image");

			int key;
			int i = 0;
			bool isEnd = false;
			IplImage* img = getRealOutletImage(test_data[0],intrinsic_matrix, distortion_params);
			cvNamedWindow(window_name);
			cvShowImage(window_name,img);
			while (!isEnd)
			{
				key = cvWaitKey();
				switch(key)
				{
					case 13:
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						setRealOutlet(test_data[i],intrinsic_matrix, distortion_params);
						img = getRealOutletImage(test_data[i],intrinsic_matrix, distortion_params);
						cvNamedWindow(window_name);
						cvShowImage(window_name,img);
						break;
					case 27: 
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						isEnd = true;
						break;
					case 32://SPACE
						if (i<((size_t)test_data.size()-1))
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[++i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The last image. Unable to get the next one\n");
						}
						break;
					case 8://Backspace
						if (i>0)
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[--i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The first image. Unable to get the previous one\n");
						}
						break;
				}
			}
		}
    
	    
	#if defined(_VERBOSE)
		char pathname[1024];
		sprintf(pathname, "mkdir %s", output_path);
		system(pathname);
	    
		sprintf(pathname, "mkdir %s/correspondence", output_path);
		system(pathname);

		sprintf(pathname, "mkdir %s/outlets", output_path);
		system(pathname);
        
		sprintf(pathname, "mkdir %s/features", output_path);
		system(pathname);

	#endif //_VERBOSE


		//Running the test
		if (strcmp(mode,"modify")==0)
		{
			writeTestFile(output_test_config,test_data);
			printf("New test config was successfully generated into %s\n",output_test_config);
		}
		else
			runFernsOneWayOutletDetectorTest(intrinsic_matrix, distortion_params,  outlet_template, train_config, test_data, output_path);

		if (strcmp(mode,"generate")==0)
		{
			convertTestToReal(test_data);
			writeTestFile(output_test_config,test_data);
			printf("New test config was successfully generated into %s\n",output_test_config);
		}

		if (strcmp(mode,"test")==0)
		{

			compareAllOutlets(test_data,accuracy);
			char filename[1024];
			if (output_path)
				sprintf(filename,"%s/results.txt",output_path);
			else
				sprintf(filename,"results.txt",output_path);
			writeTestResults(filename,test_data,result);
			printf ("Test results were successfully written into %s\n", filename);
		}
	    
		  
		cvReleaseMat(&intrinsic_matrix);
		cvReleaseMat(&distortion_params);
	        
	}
	else
	{
		printf("Unable to read test configuration file\n");
	}
		
	return 0;
}
//---------------
int run_one_way_detector(int argc,char** argv, TestResult* result = 0)
{
	char detector[1024];
	char mode[1024], config_filename[1024], camera_filename[1024], output_path[1024], 
        test_path[1024], train_config[1024], pca_config[1024], output_test_config[1024];
	int accuracy = 5;


	if (result)
	{
		result->correct = 0;
		result->skipped = 0;
		result->total = 0;
		result->incorrect = 0;
	}
	
    
	if(argc != 8 && argc != 9)
	{
		return -1;
	}
    
	
	strcpy(mode, argv[2]);
	strcpy(test_path, argv[3]);
	strcpy(config_filename, argv[4]);
	strcpy(camera_filename, argv[5]);
    
    
    outlet_template_t outlet_template;
    if(argc == 8)
    {
        strcpy(output_path, argv[7]);
    }
    else
    {
        strcpy(train_config, argv[6]);
        strcpy(output_path, argv[7]);
    }

	if ((strcmp(mode,"test")==0) )
	{
		sscanf(argv[argc-1],"%d",&accuracy);
	}
	else
	{
		strcpy(output_test_config,argv[argc-1]);
	}
	
		// reading camera params
	CvMat* intrinsic_matrix = 0;
	CvMat* distortion_params = 0; 
	_LoadCameraParams(camera_filename, &intrinsic_matrix, &distortion_params);

	vector<outlet_test_elem> test_data;

	if (readTestFile(config_filename,test_path,test_data) > 0)
	{
		if (strcmp(mode,"modify")==0)
		{
			//Code for modify mode
			printf("Use <Space> and <Backspace> to get next and previous image\n");
			printf("Press <Enter> key to enter modification mode\n");
			printf("Press <Esc> key to approve outlet positions and start the test\n");
			char window_name[100];
			strcpy(window_name,"Image");

			int key;
			int i = 0;
			bool isEnd = false;
			IplImage* img = getRealOutletImage(test_data[0],intrinsic_matrix, distortion_params);
			cvNamedWindow(window_name);
			cvShowImage(window_name,img);
			while (!isEnd)
			{
				key = cvWaitKey();
				switch(key)
				{
					case 13:
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						setRealOutlet(test_data[i],intrinsic_matrix, distortion_params);
						img = getRealOutletImage(test_data[i],intrinsic_matrix, distortion_params);
						cvNamedWindow(window_name);
						cvShowImage(window_name,img);
						break;
					case 27: 
						cvReleaseImage(&img);
						cvDestroyWindow(window_name);
						isEnd = true;
						break;
					case 32://SPACE
						if (i<((size_t)test_data.size()-1))
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[++i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The last image. Unable to get the next one\n");
						}
						break;
					case 8://Backspace
						if (i>0)
						{
							cvReleaseImage(&img);
							img = getRealOutletImage(test_data[--i],intrinsic_matrix, distortion_params);
							cvNamedWindow(window_name);
							cvShowImage(window_name,img);
						}
						else
						{
							printf("The first image. Unable to get the previous one\n");
						}
						break;
				}
			}
		}
    
	    
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
			

	if ((argc !=8) && (strcmp(mode,"modify")!=0))
	{
		outlet_template.load(train_config);
	}


		//Running the test
	if (strcmp(mode,"modify")==0)
	{
		writeTestFile(output_test_config,test_data);
		printf("New test config was successfully generated into %s\n",output_test_config);
	}
	else
		runOutletDetectorTest(intrinsic_matrix, distortion_params, outlet_template, test_data, output_path);


		if (strcmp(mode,"generate")==0)
		{
			convertTestToReal(test_data);
			writeTestFile(output_test_config,test_data);
			printf("New test config was successfully generated into %s\n",output_test_config);
		}

		if (strcmp(mode,"test")==0)
		{

			compareAllOutlets(test_data,accuracy);
			char filename[1024];
			if (output_path)
				sprintf(filename,"%s/results.txt",output_path);
			else
				sprintf(filename,"results.txt",output_path);
			writeTestResults(filename,test_data,result);
			printf ("Test results were successfully written into %s\n", filename);
		}
	    
		  
		cvReleaseMat(&intrinsic_matrix);
		cvReleaseMat(&distortion_params);
	        
	}
	else
	{
		printf("Unable to read test configuration file\n");
	}
		
	return 0;
}
//---------------
int run(int argc,char** argv, TestResult* result = 0)
{
	char detector[1024];

	strcpy(detector,argv[1]);
	if (strcmp(detector,"one_way_detector")==0)
		return run_one_way_detector(argc,argv,result);

	if (strcmp(detector,"ferns_l_detector")==0)
		return run_ferns_l_detector(argc,argv,result);
	if (strcmp(detector,"l_detector")==0)
		return run_l_detector(argc,argv,result);
	if (strcmp(detector,"ferns_one_way_detector")==0)
		return run_ferns_one_way_detector(argc,argv,result);

	return -1;


}

int main(int argc,char** argv)
{   
	char mode[1024];
	int accuracy = 5;
	char detector[1024];

	
	
	if(argc != 8 && argc != 9 && argc!=4 && argc!=5 && argc!=6)
	{
		printf("Usage: detection_test <detector_name> <mode[test|modify|generate]> <images_path> <test_config_filename> <camera_config> <output_path> <accuracy|output_test_config_filename|output_test_config_filename>\n");
        printf("Usage: detection_test <detector_name> <mode[test|modify|generate]> <images_path> <test_config_filename> <camera_config> <train_config_path> <output_path> <accuracy|output_test_config_filename|output_test_config_filename>\n");
		printf("Usage: detection_test <detector_name> <mode[test|modify|generate]> <test_set_path> [<accuracy>||] [<dataset_substring||>]\n");
		return 0;
	}
	else
	{
		
		strcpy(detector,argv[1]);
		strcpy(mode, argv[2]);
		if (((argc ==5)||(argc ==6)) && (strcmp(mode,"test")!=0))
		{
			printf("Usage: detection_test <detector_name> <mode[test|modify|generate]> <images_path> <test_config_filename> <camera_config> <output_path> <accuracy|output_test_config_filename|output_test_config_filename>\n");
			printf("Usage: detection_test <detector_name> <mode[test|modify|generate]> <images_path> <test_config_filename> <camera_config> <train_config_path> <output_path> <accuracy|output_test_config_filename|output_test_config_filename>\n");
			printf("Usage: detection_test <detector_name> <mode[test|modify|generate]> <test_set_path> [<accuracy>||] [<dataset_substring||>]\n");
			return 0;
		}
		else
		{
			if ((argc == 5)||(argc == 6))
			{
				sscanf(argv[4],"%d",&accuracy);
			}
		}
	}

	if ((argc == 4)||(argc==5)||(argc == 6))
	{
		char test_set_path[1024];
		char** command_line;
		char images_path[1024];
		char test_config_filename[1024];
		char camera_config[1024];
		char train_config[1024];
		char output_path[1024];
		char datalist_path[1024];

		
		strcpy(test_set_path,argv[3]);
		sprintf(datalist_path,"%s/data/datalist.txt",test_set_path);

		FILE* f = fopen(datalist_path,"r");
		if (f)
		{
			char pathname[1024];
			sprintf(pathname, "mkdir %s/results", test_set_path);
			system(pathname);

			char image_dir[1024];
			vector<TestResult*> test_results;
			vector<char*> dataset_names;
			bool doWork = true;
			while (fscanf(f,"%s\n",image_dir) !=EOF)
			{
				doWork = true;

				if (argc==6)
				{
					if (strstr(image_dir,argv[5]) == NULL)
						doWork = false;
				}
				if (doWork)
				{
					printf("======Working on dataset %s======\n",image_dir);
					sprintf(pathname, "mkdir %s/results/%s", test_set_path,image_dir);
					system(pathname);
					sprintf(images_path,"%s/data/%s",test_set_path,image_dir);
					sprintf(test_config_filename,"%s/data/%s/list.txt",test_set_path,image_dir);
					sprintf(camera_config,"%s/configs/%s/camera.yml",test_set_path,image_dir);
					sprintf(train_config,"%s/configs/%s",test_set_path,image_dir);
					sprintf(output_path,"%s/results/%s",test_set_path,image_dir);
					
					command_line = new char*[9];
					
					if (argc == 4)
					{
						//sprintf(command_line,"%s %s %s %s %s %s %s %s",argv[0],mode,images_path,test_config_filename,camera_config,train_config,test_set_path,test_config_filename);
						command_line[0] = new char[strlen(argv[0])];
						strcpy(command_line[0],argv[0]);
						command_line[1] = new char[strlen(detector)];
						strcpy(command_line[1],detector);
						command_line[2] = new char[strlen(mode)];
						strcpy(command_line[2],mode);
						command_line[3] = new char[strlen(images_path)];
						strcpy(command_line[3],images_path);
						command_line[4] = new char[strlen(test_config_filename)];
						strcpy(command_line[4],test_config_filename);
						command_line[5] = new char[strlen(camera_config)];
						strcpy(command_line[5],camera_config);
						command_line[6] = new char[strlen(train_config)];
						strcpy(command_line[6],train_config);
						command_line[7] = new char[strlen(output_path)];
						strcpy(command_line[7],output_path);
						command_line[8] = new char[strlen(test_config_filename)];
						strcpy(command_line[8],test_config_filename);


						run(9,command_line);
					}
					else
					{
						//sprintf(command_line,"%s %s %s %s %s %s %s %d",argv[0],mode,images_path,test_config_filename,camera_config,train_config,test_set_path,accuracy);
						command_line[0] = new char[strlen(argv[1])];
						strcpy(command_line[0],argv[1]);
						command_line[1] = new char[strlen(detector)];
						strcpy(command_line[1],detector);
						command_line[2] = new char[strlen(mode)];
						strcpy(command_line[2],mode);
						command_line[3] = new char[strlen(images_path)];
						strcpy(command_line[3],images_path);
						command_line[4] = new char[strlen(test_config_filename)];
						strcpy(command_line[4],test_config_filename);
						command_line[5] = new char[strlen(camera_config)];
						strcpy(command_line[5],camera_config);				
						command_line[6] = new char[strlen(train_config)];			
						strcpy(command_line[6],train_config);	
						command_line[7] = new char[strlen(output_path)];
						strcpy(command_line[7],output_path);												
						command_line[8] = new char[strlen(argv[4])]; //argc[4] == accuracy
						strcpy(command_line[8],argv[4]);

						TestResult* res = new TestResult();
						if (run(9,command_line,res) < 0 )
						{
							printf("Unable to %s dataset %s\n",mode,image_dir);
						}
						else
						{
							test_results.push_back(res);
							char* name = new char[strlen(image_dir)];
							strcpy(name,image_dir);
							dataset_names.push_back(name);
						}
					}
				}

			}

			if ((argc == 5)||(argc == 6))
			{
				char res_path[1024];
				sprintf(res_path,"%s/results/results.txt",test_set_path);
				FILE* fres = fopen(res_path,"w");
				if (fres)
				{
					printf("---------------------\nAll tests completed\nSummary:\n");
					int nTotal = 0;
					int nSkipped = 0;
					int nCorrect = 0;
					int nIncorrect = 0;
					fprintf(fres,"Detector: %s\n---------------------\n",detector);
					for (int i=0;i<(int)test_results.size();i++)
					{
						TestResult* res = test_results[i];
						nTotal += res->total;
						nCorrect += res->correct;
						nIncorrect += res->incorrect;
						nSkipped += res->skipped;
						fprintf(fres,"%s (correct/total): %d/%d\n",dataset_names[i],res->correct,res->correct+res->incorrect);
						printf("%s (correct/total): %d/%d\n",dataset_names[i],res->correct,res->correct+res->incorrect);
					}

					fprintf(fres,"---------------\n");
					printf("---------------\n");
					fprintf(fres,"Total images processed: %d\n",nTotal);
					fprintf(fres,"Images skipped: %d\n",nSkipped);
					fprintf(fres,"Correct detections: %d\n",nCorrect);
					fprintf(fres,"Incorrect detections: %d\n",nIncorrect);
					printf("Total images processed: %d\n",nTotal);
					printf("Images skipped: %d\n",nSkipped);
					printf("Correct detections: %d\n",nCorrect);
					printf("Incorrect detections: %d\n",nIncorrect);
					fclose(fres);

				}
				
			}
			fclose(f);

		}
		else
		{
			printf("Unable to open %s",datalist_path);
			return 0;
		}
		
	}
	else
	{
		run(argc,argv);
	}

	
	return 0;

}


