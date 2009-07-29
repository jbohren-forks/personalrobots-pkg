/* Test system for outlet detector
Created by: Alexey Latyshev
*/

#include <stdio.h>
#include <vector>

#include "outlet_detection/outlet_detector_test.h"

#define DETECT_NA -1
#define DETECT_SKIP -2

int readTestFile(char* filename, vector<outlet_test_elem>& test_data)
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
					sprintf(test_data[i++].filename,"%s",tok);
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
			fprintf(f,"%s",data[i].filename);
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

	if (intrinsic_matrix)
	{
		IplImage* img = cvCloneImage(img2);
		cvUndistort2(img,img2,intrinsic_matrix, distortion_params);
		cvReleaseImage(&img);
	}
	cvNamedWindow(window_name);
	cvShowImage(window_name,img2);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* seq = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );

	printf("-----------------------\n");
	printf("Select holes by clicking with a left mouse button\nPress right mouse button to remove the point\nPress <Enter> to end the selection and <Esc> to cancel\n");
	printf("Power holes are the first ones and ground holes are the last ones\n");
	cvSetMouseCallback( window_name, on_mouse_points, seq );

	CvPoint* test_point;
	int key = 0;
	int total= seq->total;
	CvScalar color = CV_RGB(255, 255, 0);
	bool isEnd = false;
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
				img2 = cvLoadImage(test_elem.filename);
				if (intrinsic_matrix)
				{
					IplImage* img = cvCloneImage(img2);
					cvUndistort2(img,img2,intrinsic_matrix, distortion_params);
					cvReleaseImage(&img);
				}
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
		if (compareOutlets(test_data[i]) == (int)test_data[i].real_outlet.size())
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
		IplImage* img = cvLoadImage(test_data[i].filename);
		if (img == NULL)
		{
			printf("Unable to load image %s\n",test_data[i].filename);
			continue;
		}
		printf("%s\n",filename);
		if (output_path)
			detect_outlet_tuple(img,intrinsic_matrix,distortion_params,test_data[i].test_outlet,outlet_templ,output_path,filename);
		else
			detect_outlet_tuple(img,intrinsic_matrix,distortion_params,test_data[i].test_outlet,outlet_templ);
		cvReleaseImage(&img);
	}	
}