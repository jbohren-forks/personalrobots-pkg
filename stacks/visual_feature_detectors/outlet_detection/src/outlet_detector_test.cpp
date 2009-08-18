/* Test system for outlet detector
Created by: Alexey Latyshev
*/

#include <stdio.h>
#include <vector>

#include "outlet_detection/outlet_detector_test.h"
#include "outlet_detection/ferns_detector_outlets.h"
//#include "outlet_detection/one_way_descriptor.h"
//#include "outlet_detection/one_way_descriptor_base.h"
//#include "outlet_detection/one_way_outlets.h"
#include "outlet_detection/constellation.h"

#define DETECT_NA -1
#define DETECT_SKIP -2

#define REG_DO_NOTHING -1
#define REG_DRAW_RECT 0
#define REG_STOP_SELECT 1
#define REG_CLEAR_SELECT 2

int readTestFile(char* filename, char* test_path, vector<outlet_test_elem>& test_data)
{
	FILE* f = fopen(filename,"r");
	test_data.clear();
	if (f)
	{
		int n_holes = 0;
		int n_outlets = 0;
		char line[1024];
		int s_res = fscanf(f,"%s\n",line);
		char* tok;

		if ( (s_res!= 0)&&(s_res!=EOF))
		{
			n_outlets++;
			tok = strtok(line,",");	
			while (tok)
			{	
				n_holes++;
				tok = strtok(NULL,",");
				
			}
			n_holes--;
			n_holes/=2;

			fseek(f ,0 ,SEEK_SET);
			int i=0;
			while (fscanf(f,"%s\n",line) !=EOF)
			{
				
				tok = strtok(line,",");
				if (tok)
				{
					outlet_test_elem elem;
					test_data.push_back(elem);
					sprintf(test_data[i++].filename,"%s/%s",test_path,tok);
				}
				

			}

			if ((n_holes > 0)&&(n_holes % 3 == 0)&&((n_holes % 2 == 0)))
			{				
				fseek(f ,0 ,SEEK_SET);
				int ground_n = 2*n_holes /3;
				CvPoint* points = new CvPoint[n_holes];

				int i=0;
					
				while (fscanf(f,"%s\n",line) !=EOF)
				{
					tok = strtok(line,",");

					int j = 0;

					tok = strtok(NULL,",");
					while (tok)
					{
						if (j%2 == 0)
						{
							sscanf(tok,"%d",&(points[j++/2].x));
						}
						else
						{
							sscanf(tok,"%d",&(points[j++/2].y));
						}
						tok = strtok(NULL,",");
					}

					outlet_t outlet;
					if (j/2!=n_holes)
					{
						//delete[] points;
						test_data[i].n_matches = DETECT_SKIP;
						//fclose(f);
						//return 0;
					}
					else
					{
					
						for (j=0;j<ground_n/2;j++)
						{
							outlet.hole1 = points[2*j];
							outlet.hole2 = points[2*j+1];
							test_data[i].real_outlet.push_back(outlet);
							
						}
						for (j = ground_n;j<n_holes;j++)
						{
							test_data[i].real_outlet[j-ground_n].ground_hole = points[j];
						}
						test_data[i].n_matches = DETECT_NA;
					}
					
					i++;
				}

				delete[] points;
				fclose(f);
				return (int)(test_data.size());

			}
		}
			
		fclose(f);
		return (int)(test_data.size());
	}
	return 0;
}
//------------------------------------
int writeTestFile(char* filename, vector<outlet_test_elem>& data)
{
	FILE* f = fopen(filename,"w");
	if (f)
	{
		for (int i=0;i<(size_t)(data.size());i++)
		{
			//if (data[i].n_matches == DETECT_SKIP)
			//	continue;
			char* name = strrchr(data[i].filename,'/');
			name++;
			fprintf(f,"%s",name);
			for (int j=0;j<(size_t)(data[i].real_outlet.size());j++)
			{
				fprintf(f,",%d,%d,%d,%d",data[i].real_outlet[j].hole1.x,data[i].real_outlet[j].hole1.y,
					data[i].real_outlet[j].hole2.x,data[i].real_outlet[j].hole2.y);
			}
			for (int j=0;j<(size_t)(data[i].real_outlet.size());j++)
			{
				fprintf(f,",%d,%d",data[i].real_outlet[j].ground_hole.x,data[i].real_outlet[j].ground_hole.y);
			}
			fprintf(f,"\n");
		}
		fclose(f);
		return (size_t)(data.size());
	}

	return 0;
}
//--------------------------------
void convertTestToReal(vector<outlet_test_elem>& data)
{
	for (int i=0;i<(size_t)data.size();i++)
	{
		if (data[i].n_matches == DETECT_SKIP)
			continue;
		data[i].real_outlet = data[i].test_outlet;
	}
}
//--------------------------------
void showRealOutlet(const outlet_test_elem& test_elem,CvMat* intrinsic_matrix, CvMat* distortion_params)
{
	char window_name[100];
	strcpy(window_name,"Outlet");
	

	IplImage* img = cvLoadImage(test_elem.filename);

	if (img)
	{
		IplImage* img1 = cvCloneImage(img);
		cvUndistort2(img1,img,intrinsic_matrix,distortion_params);
		cvReleaseImage(&img1);
		CvScalar color[] = {CV_RGB(255, 255, 0), CV_RGB(0, 255, 255)};
		cvNamedWindow(window_name);
		int n = (size_t)(test_elem.real_outlet.size());

		for (int i=0;i<(size_t)(test_elem.real_outlet.size());i++)
		{
			cvCircle(img,test_elem.real_outlet[i].hole1,3,color[0]);
			cvCircle(img,test_elem.real_outlet[i].hole2,3,color[0]);
			cvCircle(img,test_elem.real_outlet[i].ground_hole,3,color[1]);
		}
		cvShowImage(window_name,img);
		cvWaitKey();
		cvDestroyWindow(window_name);
		cvReleaseImage(&img);
		
	}
}
//--------------------------------
IplImage* getRealOutletImage(const outlet_test_elem& test_elem,CvMat* intrinsic_matrix, CvMat* distortion_params)
{
	
	IplImage* img = cvLoadImage(test_elem.filename);
	if (img)
	{
		IplImage* img1 = cvCloneImage(img);
		cvUndistort2(img1,img,intrinsic_matrix,distortion_params);
		cvReleaseImage(&img1);
		CvScalar color[] = {CV_RGB(255, 255, 0), CV_RGB(0, 255, 255)};
		int n = (size_t)(test_elem.real_outlet.size());

		for (int i=0;i<(size_t)(test_elem.real_outlet.size());i++)
		{
			cvCircle(img,test_elem.real_outlet[i].hole1,3,color[0]);
			cvCircle(img,test_elem.real_outlet[i].hole2,3,color[0]);
			cvCircle(img,test_elem.real_outlet[i].ground_hole,3,color[1]);
		}

		return img;
		
	}
	return NULL;
}
//--------------------------------
void on_mouse_region( int event, int x, int y, int flags, void* param )
{
	switch( event )
	{
		case CV_EVENT_LBUTTONDOWN:
				((int*)param)[0] = x;
				((int*)param)[1] = y;
				((int*)param)[2] = x;
				((int*)param)[3] = y;
				((int*)param)[4] = REG_DRAW_RECT;
				break;

		case CV_EVENT_LBUTTONUP:
			if (((int*)param)[4] == REG_DRAW_RECT)
			{
				((int*)param)[2] = x;
				((int*)param)[3] = y;
				((int*)param)[4] = REG_STOP_SELECT;
			}
			else
				((int*)param)[4] = REG_CLEAR_SELECT;
			break;

		case CV_EVENT_RBUTTONDOWN:
			if (((int*)param)[4] == REG_DRAW_RECT)
			{
				((int*)param)[4] = REG_CLEAR_SELECT;
			}
			else
				((int*)param)[4] = REG_DO_NOTHING;
			break;

		case CV_EVENT_MOUSEMOVE:
			if (((int*)param)[4] == REG_DRAW_RECT)
			{
				((int*)param)[2] = x;
				((int*)param)[3] = y;
			}
			else
				if (((int*)param)[4] != REG_STOP_SELECT)
					((int*)param)[4] = REG_DO_NOTHING;
			break;
	}
}

void on_mouse_points( int event, int x, int y, int flags, void* param )
{
	switch( event )
	{

		case CV_EVENT_LBUTTONUP:
			cvSeqPush((CvSeq*)param,&cvPoint(x,y));
			break;

		case CV_EVENT_RBUTTONUP:
			{
				int n = ((CvSeq*)param)->total;
				if (n > 0)
					cvSeqPop(((CvSeq*)param));
			}
			break;

	}
}
void setRealOutlet(outlet_test_elem& test_elem, CvMat* intrinsic_matrix, CvMat* distortion_params)
{
	int holes_count = 0;
	char window_name[128];
	strcpy(window_name,"Select outlet holes");
	IplImage* img2 = cvLoadImage(test_elem.filename);
	if (img2 == NULL)
	{
		printf("Unable to load image %s",test_elem.filename);
		return;
	}
	IplImage* img;
	if (intrinsic_matrix)
	{
		img = cvCloneImage(img2);
		cvUndistort2(img,img2,intrinsic_matrix, distortion_params);
		cvReleaseImage(&img);
	}
	img = cvCloneImage(img2);
	cvNamedWindow(window_name,0);
	cvResizeWindow(window_name,img->width,img->height);
	cvShowImage(window_name,img2);
	int key = 0;
	bool isEnd = false;
#if 1
//Added zoom
	int* params = new int[5]; // Rect Coordinates + parameter (-1 - do nothing, 0 - draw rectangle, 1 - stop selection, 2 - clear selection)
	//CvRect region;
	//IplImage* region;
	CvRect region_rect;

	printf("-----------------------\n");
	printf("Select the region with outlet.\nPress left mouse button to start the selection and release it to end the selection\nPress right mouse button to try again\n");

	cvSetMouseCallback( window_name, on_mouse_region, params );
	while (!isEnd)
	{
		while (params[4] != REG_STOP_SELECT)
		{
			key = cvWaitKey(30);
			if (key == 27)
				break;
			if (params[4] == REG_DRAW_RECT)
			{
				cvReleaseImage(&img2);
				img2 = cvCloneImage(img);
				cvRectangle(img2,cvPoint(params[0],params[1]),cvPoint(params[2],params[3]),cvScalar(50,200,50),2);
				cvShowImage(window_name,img2);
			}
			if (params[4] == REG_CLEAR_SELECT)
			{
				cvReleaseImage(&img2);
				img2 = cvCloneImage(img);
				cvShowImage(window_name,img2);

			}
		}

		if (key!=27)
		{
			if ((params[2] != params[0]) && (params[3] != params[1]))
			{
				if (params[2] < params[0])
				{
					int temp = params[0];
					params[0] = params[2];
					params[2] = temp;
				}
				if (params[3] < params[1])
				{
					int temp = params[1];
					params[1] = params[3];
					params[3] = temp;
				}
				cvReleaseImage(&img2);
				img2 = cvCloneImage(img);
				region_rect.x = params[0];
				region_rect.y = params[1];
				region_rect.width = params[2] - params[0];
				region_rect.height = params[3] - params[1];
				cvSetImageROI(img2,region_rect);
				cvReleaseImage(&img);
				img = cvCreateImage(cvSize(img2->roi->width,img2->roi->height),img2->depth,img2->nChannels);
				cvCopy(img2,img);
				

				
				cvReleaseImage(&img2);
				img2 = cvCloneImage(img);
				//cvReleaseImage(&region);
				cvShowImage(window_name,img2);
				cvResizeWindow(window_name,img2->width*2,img2->height*2);
				isEnd = true;

			}
			else
			{
				printf("Incorrect selection, try again\n");
				isEnd = false;
			}
		}
		else
			isEnd = true;
	}
	


	delete[] params;
//End
#endif


	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* seq = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );

	printf("-----------------------\n");
	printf("Select holes by clicking with a left mouse button\nPress right mouse button to remove the point\nPress <Enter> to end the selection and <Esc> to cancel\n");
	printf("Power holes are the first ones and ground holes are the last ones\n");
	cvSetMouseCallback( window_name, on_mouse_points, seq );

	CvPoint* test_point;
	int total= seq->total;
	CvScalar color = CV_RGB(255, 255, 0);
	isEnd = false;
	key = 0;
	while ((!isEnd)&&(key!=27))
	{
		key = cvWaitKey(30);
		if (seq->total > total)
		{
			CvPoint* pt;
			for (int i=total; i< seq->total;i++)
			{
				pt = (CvPoint*)cvGetSeqElem(seq,i);
				cvCircle(img2,*pt,3,color);
			}
			total = seq->total;
			cvShowImage(window_name,img2);
		}
		else
			if (seq->total < total)
			{
				cvReleaseImage(&img2);
				img2 = cvCloneImage(img);
				//if (intrinsic_matrix)
				//{
				//	IplImage* img = cvCloneImage(img2);
				//	cvUndistort2(img,img2,intrinsic_matrix, distortion_params);
				//	cvReleaseImage(&img);
				//}
				CvPoint* pt;
				for (int i=0; i< seq->total;i++)
				{
					pt = (CvPoint*)cvGetSeqElem(seq,i);
					cvCircle(img2,*pt,3,color);
				}
				total = seq->total;
				cvShowImage(window_name,img2);
			}

		if (key==13)
		{
			if (total % 3 == NULL)
				isEnd = true;
			else
			{
				isEnd=false;
				printf("You have to select 3xn holes\n");
			}
		}
	}

	cvDestroyWindow(window_name);
	cvReleaseImage(&img2);
	cvReleaseImage(&img);

	if (key == 27)
	{
		cvReleaseMemStorage(&storage);
		holes_count = 0;
		return;
	}
	else
	{
		holes_count = seq->total;
		CvPoint* points = new CvPoint[holes_count];
		cvCvtSeqToArray(seq,points);
		if (holes_count % 3 == 0)
		{
			int j;
			int ground_n = holes_count/3*2;
			outlet_t outlet;
			test_elem.real_outlet.clear();
			for (j=0;j<ground_n/2;j++)
			{
				outlet.hole1 = points[2*j];
				outlet.hole2 = points[2*j+1];
				test_elem.real_outlet.push_back(outlet);
				
			}
			for (j = ground_n;j<holes_count;j++)
			{
				test_elem.real_outlet[j-ground_n].ground_hole = points[j];
			}
			test_elem.n_matches = DETECT_NA;
#if 1
//Added zoom
			for (int i=0;i<(size_t)test_elem.real_outlet.size();i++)
			{
				test_elem.real_outlet[i].hole1.x +=region_rect.x;
				test_elem.real_outlet[i].hole1.y +=region_rect.y;
				test_elem.real_outlet[i].hole2.x +=region_rect.x;
				test_elem.real_outlet[i].hole2.y +=region_rect.y;
				test_elem.real_outlet[i].ground_hole.x +=region_rect.x;
				test_elem.real_outlet[i].ground_hole.y +=region_rect.y;
			}
//End
#endif
		}



		
		cvReleaseMemStorage(&storage);
		delete[] points;
	}
}
//--------------------------------
int writeTestResults(char* filename, const vector<outlet_test_elem>& test_data)
{
	FILE* f = fopen(filename,"w");
	if (f)
	{
		int nCorrect = 0;
		int nNA = 0;
		int nTotal = (size_t)test_data.size();
		int nSkipped = 0;
		fprintf(f,"Outlet detector test results\n---------------------\n");
		for (int i=0;i<(size_t)test_data.size();i++)
		{
			if (test_data[i].n_matches == DETECT_SKIP)
			{
				//nTotal--;
				fprintf(f,"%s: image was skipped\n",test_data[i].filename);
				nSkipped++;
				continue;
			}
			if (test_data[i].n_matches != 3*(int)(test_data[i].real_outlet.size()))
			{
				if (test_data[i].n_matches == DETECT_NA)
				{
					fprintf(f,"%s: image wasn't tested\n",test_data[i].filename);
					nNA++;
				}
				else
					fprintf(f,"%s: incorrect detection\n",test_data[i].filename);
			}
			else
			{
				nCorrect++;
			}
		}
		if (nCorrect+nNA > 0)
			fprintf(f,"---------------------\n");
		fprintf(f,"Total images:%d\nSkipped:%d\nCorrect:%d\nIncorrect:%d\nNA:%d",nTotal,nSkipped,nCorrect,(int)test_data.size()-nCorrect-nNA,nNA);
		fclose(f);
		return nTotal-nSkipped;
	}

	return 0;
}
//---------------------------
int compareOutlets(outlet_test_elem& test_elem, int accuracy)
{
	if ((int)test_elem.real_outlet.size() != (int)test_elem.test_outlet.size())
	{
		if (((int)test_elem.test_outlet.size()==0)&&((int)test_elem.real_outlet.size()>0))
		{
			test_elem.n_matches = 0;
			return 0;
		}
		test_elem.n_matches = DETECT_NA;
		return DETECT_NA;
	}

	int nOutlets = 0;
	test_elem.n_matches = 0;
	nOutlets = (int)test_elem.real_outlet.size();
    int flag = 0;
	for (int i=0;i<nOutlets;i++)
	{
		float distance = (test_elem.real_outlet[i].hole1.x - test_elem.test_outlet[i].hole1.x)*(test_elem.real_outlet[i].hole1.x - test_elem.test_outlet[i].hole1.x)+
			(test_elem.real_outlet[i].hole1.y - test_elem.test_outlet[i].hole1.y)*(test_elem.real_outlet[i].hole1.y - test_elem.test_outlet[i].hole1.y);
		if (distance < accuracy*accuracy)
			test_elem.n_matches++;

		distance = (test_elem.real_outlet[i].hole2.x - test_elem.test_outlet[i].hole2.x)*(test_elem.real_outlet[i].hole2.x - test_elem.test_outlet[i].hole2.x)+
			(test_elem.real_outlet[i].hole2.y - test_elem.test_outlet[i].hole2.y)*(test_elem.real_outlet[i].hole2.y - test_elem.test_outlet[i].hole2.y);
		if (distance < accuracy*accuracy)
			test_elem.n_matches++;

		distance = (test_elem.real_outlet[i].ground_hole.x - test_elem.test_outlet[i].ground_hole.x)*(test_elem.real_outlet[i].ground_hole.x - test_elem.test_outlet[i].ground_hole.x)+
			(test_elem.real_outlet[i].ground_hole.y - test_elem.test_outlet[i].ground_hole.y)*(test_elem.real_outlet[i].ground_hole.y - test_elem.test_outlet[i].ground_hole.y);
		if (distance < accuracy*accuracy)
			test_elem.n_matches++;
    }

	return test_elem.n_matches;

}
//
int compareAllOutlets(vector<outlet_test_elem>& test_data, int accuracy)
{
	int res = 0;
	for (int i=0;i<(size_t)test_data.size();i++)
	{
		if (test_data[i].n_matches == DETECT_SKIP)
			continue;
        if(compareOutlets(test_data[i], accuracy) == (int)test_data[i].real_outlet.size())
			res++;
	}
	return res;
}

//----------------------------
//run test
void runOutletDetectorTest(CvMat* intrinsic_matrix, CvMat* distortion_params, 
							const outlet_template_t& outlet_templ,vector<outlet_test_elem>& test_data, char* output_path)
{
	char filename[1024];

	for (int i=0;i<(size_t)test_data.size();i++)
	{
		if (test_data[i].n_matches == DETECT_SKIP)
			continue;
		if (output_path)
		{
			char* name = strrchr(test_data[i].filename,'/');
			name++;
			sprintf(filename,"%s",name);
		}
		//printf("%s",test_data[i].filename);
		IplImage* img = cvLoadImage(test_data[i].filename);
		if (img == NULL)
		{
			printf("Unable to load image %s\n",test_data[i].filename);
			continue;
		}
		printf("%s\n",filename);
		if (output_path)
		{
			detect_outlet_tuple(img,intrinsic_matrix,distortion_params,test_data[i].test_outlet,outlet_templ,output_path,filename);

		}
		else
			detect_outlet_tuple(img,intrinsic_matrix,distortion_params,test_data[i].test_outlet,outlet_templ);

		if (((int)test_data[i].test_outlet.size() > 0)&&((int)test_data[i].real_outlet.size() > 0))
		{
			printf("Detected %d outlets, origin %d,%d, real origin %d,%d\n", (int)test_data[i].test_outlet.size(), test_data[i].test_outlet[0].ground_hole.x, 
				test_data[i].test_outlet[0].ground_hole.y, test_data[i].real_outlet[0].ground_hole.x, test_data[i].real_outlet[0].ground_hole.y);
		}

		cvReleaseImage(&img);
	}	
}

//----------------------------
//run Ferns+L Detector test
void runLOutletDetectorTest(CvMat* intrinsic_matrix, CvMat* distortion_params, 
							const char* config_path, vector<outlet_test_elem>& test_data, char* output_path)
{
	char filename[1024];

	vector<feature_t> train_features;
	char object_filename[1024];
	char scene_filename[1024];
	char outlet_filename[1024];
	read_training_base(config_path,outlet_filename,train_features);	

	sprintf(object_filename,"%s/%s",config_path,outlet_filename);

	
    Mat object = imread( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
	Mat intrinsic(intrinsic_matrix,true);
	Mat distortion(distortion_params,true);

	PlanarObjectDetector detector;
    Size patchSize(32, 32);
    LDetector ldetector(7, 20, 2, 2000, patchSize.width, 2);
    ldetector.setVerbose(true);

	ferns_l_detector_initialize(detector,ldetector,config_path,object, patchSize);

	for (int i=0;i<(size_t)test_data.size();i++)
	{
		if (test_data[i].n_matches == DETECT_SKIP)
			continue;
		if (output_path)
		{
			char* name = strrchr(test_data[i].filename,'/');
			name++;
			sprintf(filename,"%s",name);
		}
		//sprintf(scene_filename,"%s","D:/work/Argus/DATA/forearm_2009_06_17/cracked/frame0001.jpg");
		
		Mat image = imread( test_data[i].filename);

		if((!image.data) || (!object.data))
		{
			printf("Unable to load image %s\n",test_data[i].filename);
			continue;
		}
		printf("%s\n",filename);
		Mat undistortedImg;
		undistort(image,undistortedImg,intrinsic,distortion);
		if (output_path)
		{
			ferns_l_detect_outlets(undistortedImg, object, config_path, detector,ldetector,train_features,
				test_data[i].test_outlet, output_path, filename);

		}
		else
		{
			ferns_l_detect_outlets(undistortedImg, object, config_path, detector,ldetector,train_features,
				test_data[i].test_outlet);
		}
		
			//detect_outlet_tuple(img,intrinsic_matrix,distortion_params,test_data[i].test_outlet,outlet_templ);
       	if ((size_t)test_data[i].test_outlet.size() > 0)
			printf("Detected %d outlets, origin %d,%d, real origin %d,%d\n", (int)test_data[i].test_outlet.size(), test_data[i].test_outlet[0].ground_hole.x, 
				test_data[i].test_outlet[0].ground_hole.y, test_data[i].real_outlet[0].ground_hole.x, test_data[i].real_outlet[0].ground_hole.y);
		else
			printf ("Unable to detect outlet\n");    
	}	
}
//-------------
void runFernsLOutletDetectorTest(CvMat* intrinsic_matrix, CvMat* distortion_params, 
							const char* config_path, vector<outlet_test_elem>& test_data, char* output_path)
{
	char filename[1024];

	vector<feature_t> train_features;
	char object_filename[1024];
	char scene_filename[1024];
	char outlet_filename[1024];
	read_training_base(config_path,outlet_filename,train_features);	

	sprintf(object_filename,"%s/%s",config_path,outlet_filename);

	
    Mat object = imread( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
	Mat intrinsic(intrinsic_matrix,true);
	Mat distortion(distortion_params,true);

	PatchGenerator gen(0,256,15,true,0.6,1.5,-CV_PI,CV_PI,-CV_PI,CV_PI);
	FernClassifier detector;
    Size patchSize(32, 32);
    LDetector ldetector(7, 20, 2, 2000, patchSize.width, 2);
    ldetector.setVerbose(true);
	detector.setVerbose(true);
	Vector<Point2f> image_keypoints;
	Vector<Point2f> object_keypoints;

	Vector<Mat> objpyr;
	Vector<KeyPoint> objKeypoints;

	Mat tempobj = object.clone();

	GaussianBlur(tempobj, tempobj, Size(blurKSize, blurKSize), sigma, sigma);
	buildPyramid(tempobj, objpyr, ldetector.nOctaves-1);
	ldetector.setVerbose(true);
	printf("Getting the most stable object points...");
	ldetector.getMostStable2D(tempobj, objKeypoints, 100, gen);
	printf("Done\n");

	for (int i=0;i<(int)objKeypoints.size();i++)
	{
		object_keypoints.push_back(objKeypoints[i].pt);
	}

	if (!ferns_detector_load(detector,config_path))
	{
		printf("Ferns Classifier training...");
		ferns_detector_initialize(object_keypoints,detector,config_path,object, patchSize,gen);
		printf("Done\n");
	}
	//ferns_l_detector_initialize(detector,ldetector,config_path,object, patchSize);

	for (int i=0;i<(size_t)test_data.size();i++)
	{
		if (test_data[i].n_matches == DETECT_SKIP)
			continue;
		if (output_path)
		{
			char* name = strrchr(test_data[i].filename,'/');
			name++;
			sprintf(filename,"%s",name);
		}
		//sprintf(scene_filename,"%s","D:/work/Argus/DATA/forearm_2009_06_17/cracked/frame0001.jpg");
		
		Mat image = imread( test_data[i].filename);

		if((!image.data) || (!object.data))
		{
			printf("Unable to load image %s\n",test_data[i].filename);
			continue;
		}
		printf("%s\n",filename);
		Mat undistortedImg;
		undistort(image,undistortedImg,intrinsic,distortion);

		Vector<Mat> imgpyr;

		Mat tempimg(undistortedImg.rows,undistortedImg.cols, CV_8UC1);// = undistortedImg.clone();
		cvtColor(undistortedImg,tempimg,CV_BGR2GRAY);
		GaussianBlur(tempimg,tempimg, Size(blurKSize, blurKSize), sigma, sigma);
		buildPyramid(tempimg, imgpyr, ldetector.nOctaves-1);

		Vector<KeyPoint> imgKeypoints;
		ldetector(imgpyr, imgKeypoints, 3000);
		image_keypoints.clear();
		for (int j=0;j<(int)imgKeypoints.size();j++)
		{
			image_keypoints.push_back(imgKeypoints[j].pt);
		}
		
		//for(int j = 0; j < (int)image_keypoints.size(); j++ )
		//{
		//	circle( image, image_keypoints[j], 2, Scalar(0,0,255), -1 );
		//	circle( image, image_keypoints[j], 7, Scalar(0,255,0), 1 );
		//}

		//char path[1024];
		//sprintf(path,"%s/features/1.jpg",output_path);
		//imwrite(path, image);


		if (output_path)
		{
			ferns_detect_outlets(undistortedImg,object,config_path,detector,object_keypoints,image_keypoints,train_features,
				test_data[i].test_outlet, output_path, filename);



			//ferns_l_detect_outlets(undistortedImg, object, config_path, detector,ldetector,train_features,
			//	test_data[i].test_outlet, output_path, filename);

		}
		else
		{
			ferns_detect_outlets(undistortedImg,object,config_path,detector,object_keypoints,image_keypoints,train_features,
				test_data[i].test_outlet);
			//ferns_l_detect_outlets(undistortedImg, object, config_path, detector,ldetector,train_features,
			//	test_data[i].test_outlet);
		}
		
			//detect_outlet_tuple(img,intrinsic_matrix,distortion_params,test_data[i].test_outlet,outlet_templ);
       	if ((size_t)test_data[i].test_outlet.size() > 0)
			printf("Detected %d outlets, origin %d,%d, real origin %d,%d\n", (int)test_data[i].test_outlet.size(), test_data[i].test_outlet[0].ground_hole.x, 
				test_data[i].test_outlet[0].ground_hole.y, test_data[i].real_outlet[0].ground_hole.x, test_data[i].real_outlet[0].ground_hole.y);
		else
			printf ("Unable to detect outlet\n");
	}	
}



void get_one_way_keypoints(Mat& image, const outlet_template_t& outlet_template,Vector<Point2f>& keypoints)
{
	IplImage _image = image;
	IplImage* test_image; 
	if (_image.nChannels == 3)
	{
		test_image = cvCreateImage(cvSize(_image.width, _image.height), IPL_DEPTH_8U, 1);
		cvCvtColor(&_image, test_image, CV_RGB2GRAY);
	}
	else
	{
		test_image = cvCloneImage(&_image);
	}

	const CvOneWayDescriptorObject* descriptors = outlet_template.get_one_way_descriptor_base();
	int64 time1 = cvGetTickCount();
    
    vector<feature_t> features;
    GetHoleFeatures(test_image, features, outlet_template.GetHoleContrast());
    
    int64 time2 = cvGetTickCount();
    
    printf("Found %d test features, time elapsed: %f\n", (int)features.size(), float(time2 - time1)/cvGetTickFrequency()*1e-6);
   
    vector<feature_t> hole_candidates;
    int patch_width = descriptors->GetPatchSize().width/2;
    int patch_height = descriptors->GetPatchSize().height/2; 

    for(int i = 0; i < (int)features.size(); i++)
	{
        CvPoint center = features[i].pt;
        float scale = features[i].size;
        
        CvRect roi = cvRect(center.x - patch_width/2, center.y - patch_height/2, patch_width, patch_height);
        cvSetImageROI(test_image, roi);
        roi = cvGetImageROI(test_image);
        if(roi.width != patch_width || roi.height != patch_height)
        {
            continue;
        }
        
        int desc_idx = -1;
        int pose_idx = -1;
        float distance = 0;

        descriptors->FindDescriptor(test_image, desc_idx, pose_idx, distance);
        
        CvPoint center_new = descriptors->GetDescriptor(desc_idx)->GetCenter();
        CvScalar color = descriptors->IsDescriptorObject(desc_idx) ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0);
        int part_idx = descriptors->GetDescriptorPart(desc_idx);
		
		int min_ground_idx = (int)(descriptors->GetLabeledFeatures().size()) * 2 / 3; // 3 there is number of holes in the outlet (including ground hole)
        if(part_idx >= 0 && part_idx < min_ground_idx)
        {
            //color = CV_RGB(255, 255, 0);
        }
        
        if((part_idx >= min_ground_idx) && (part_idx <  (int)(descriptors->GetLabeledFeatures().size())))
        {
           // color = CV_RGB(0, 255, 255);
        }
        
        if(part_idx >= 0)
        {
            feature_t candidate = features[i];
            if(part_idx < min_ground_idx) candidate.class_id = 0;
            else candidate.class_id = 1;
            hole_candidates.push_back(candidate);                    
        }    
        cvResetImageROI(test_image);

    }
    
    int64 time3 = cvGetTickCount();
    printf("Features matched. Time elapsed: %f\n", float(time3 - time2)/cvGetTickFrequency()*1e-6);       
    
    //        printf("%d features before filtering\n", (int)hole_candidates.size());
    vector<feature_t> hole_candidates_filtered;
    float dist = calc_set_std(descriptors->_GetLabeledFeatures());
    FilterOutletFeatures(hole_candidates, hole_candidates_filtered, dist*4);
    hole_candidates = hole_candidates_filtered;

	keypoints.clear();
	for (int i=0;i<(int)hole_candidates.size();i++)
	{
		keypoints.push_back(Point2f(hole_candidates[i].pt));
	}
	//for (int i=0;i<(int)features.size();i++)
	//{
	//	keypoints.push_back(Point2f(features[i].pt));
	//}

	cvReleaseImage(&test_image);
}

void runFernsOneWayOutletDetectorTest(CvMat* intrinsic_matrix, CvMat* distortion_params, const outlet_template_t& outlet_template,
						   char* config_path, vector<outlet_test_elem>& test_data, char* output_path)
{
	
	char filename[1024];


	vector<feature_t> train_features;
	char object_filename[1024];
	char scene_filename[1024];
	char outlet_filename[1024];
	read_training_base(config_path,outlet_filename,train_features);	

	sprintf(object_filename,"%s/%s",config_path,outlet_filename);
	
    Mat object = imread( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
	Mat intrinsic(intrinsic_matrix,true);
	Mat distortion(distortion_params,true);

	PatchGenerator gen(0,256,15,true,0.6,1.5,-CV_PI,CV_PI,-CV_PI,CV_PI);
	FernClassifier detector;
    Size patchSize(32, 32);
  
	detector.setVerbose(true);
	Vector<Point2f> image_keypoints;
	Vector<Point2f> object_keypoints;

	//get_one_way_keypoints(object,outlet_template,object_keypoints);

	//2nd version - only labeled points
	vector<KeyPointEx> objKeypoints = outlet_template.get_one_way_descriptor_base()->GetLabeledFeatures();
	for (int i=0;i<(int)objKeypoints.size();i++)
	{
		object_keypoints.push_back(objKeypoints[i].pt);
	}

	////3d version - all points
	//for (int i=0;i<outlet_template.get_one_way_descriptor_base()->GetObjectFeatureCount();i++)
	//{
	//	object_keypoints.push_back(Point2f(outlet_template.get_one_way_descriptor_base()->GetDescriptor(i)->GetCenter()));
	//}
	

	if (!ferns_detector_load(detector,config_path))
	{
		printf("Ferns Classifier training...");
		ferns_detector_initialize(object_keypoints,detector,config_path,object, patchSize,gen);
		printf("Done\n");
	}
//!!!!!!!!!!!!!!!!!!!!!
	for (int i=0;i<(size_t)test_data.size();i++)
	{
		if (test_data[i].n_matches == DETECT_SKIP)
			continue;
		if (output_path)
		{
			char* name = strrchr(test_data[i].filename,'/');
			name++;
			sprintf(filename,"%s",name);
		}
		
		Mat image = imread( test_data[i].filename);

		if((!image.data) || (!object.data))
		{
			printf("Unable to load image %s\n",test_data[i].filename);
			continue;
		}
		printf("%s\n",filename);
		Mat undistortedImg;
		undistort(image,undistortedImg,intrinsic,distortion);


		image_keypoints.clear();
		
		get_one_way_keypoints(image,outlet_template,image_keypoints);

		
		for(int j = 0; j < (int)image_keypoints.size(); j++ )
		{
			circle( image, image_keypoints[j], 2, Scalar(0,0,255), -1 );
			circle( image, image_keypoints[j], 7, Scalar(0,255,0), 1 );
		}


		if (output_path)
		{
			ferns_detect_outlets(undistortedImg,object,config_path,detector,object_keypoints,image_keypoints,train_features,
				test_data[i].test_outlet, output_path, filename);

		}
		else
		{
			ferns_detect_outlets(undistortedImg,object,config_path,detector,object_keypoints,image_keypoints,train_features,
				test_data[i].test_outlet);

		}
		
		if ((size_t)test_data[i].test_outlet.size() > 0)
			printf("Detected %d outlets, origin %d,%d, real origin %d,%d\n", (int)test_data[i].test_outlet.size(), test_data[i].test_outlet[0].ground_hole.x, 
				test_data[i].test_outlet[0].ground_hole.y, test_data[i].real_outlet[0].ground_hole.x, test_data[i].real_outlet[0].ground_hole.y);
		else
			printf ("Unable to detect outlet\n");
	}	
}