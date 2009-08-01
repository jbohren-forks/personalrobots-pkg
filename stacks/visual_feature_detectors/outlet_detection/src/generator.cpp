/* Configuration files generator for one way detector
Created by: Alexey Latyshev
*/

#include <stdio.h>

#include <cv.h>
#include <highgui.h>

// Constants for region selection
#define REG_DO_NOTHING -1
#define REG_DRAW_RECT 0
#define REG_STOP_SELECT 1
#define REG_CLEAR_SELECT 2

const int MOUSE_ACCURACY = 3;
const float ORANGE_CONTRAST = 1.3;
const float WHITE_CONTRAST = 1.5;



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
//-----------
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
				//if (n>0)
				//{
				//	CvPoint* arr = new CvPoint[n];
				//	cvCvtSeqToArray((CvSeq*)param,arr);
				//	float min_distance = (arr[0].x - x)*(arr[0].x - x)+(arr[0].y - y)*(arr[0].y - y);
				//	float min_index = 0;
				//	for (int i = 1;i<n;i++)
				//	{
				//		float distance = (arr[i].x - x)*(arr[i].x - x)+(arr[i].y - y)*(arr[i].y - y);
				//		if (distance < min_distance)
				//		{
				//			min_distance = distance;
				//			min_index = i;
				//		}
				//	}

				//	if (min_distance < MOUSE_ACCURACY*MOUSE_ACCURACY)
				//	{
				//		cvSeqRemove((CvSeq*)param,min_index);
				//	}
				//	delete[] arr;
				//}
			}
			break;


		//case CV_EVENT_LBUTTONDBLCLK :
		//	cvSeqPushFront((CvSeq*)param,&cvPoint(-1,-1));
		//	break;
	}
}
//-----------



IplImage* getOutletRegion(const IplImage* img)
{
	bool isEnd = false;
	IplImage* img2 = cvCloneImage(img);
	IplImage* region;
	char window_name[128];
	strcpy(window_name,"Config generator: select the region");
	cvNamedWindow(window_name);
	int key = 0;
	
	int* params = new int[5]; // Rect Coordinates + parameter (-1 - do nothing, 0 - draw rectangle, 1 - stop selection, 2 - clear selection)


	while (!isEnd)
	{
		for (int i=0;i<5;i++)
			params[i] = -1;
		cvReleaseImage(&img2);
		img2 = cvCloneImage(img);
		key = 0;
		cvShowImage(window_name,img2);
		printf("-----------------------\n");
		printf("Select the region.\nPress left mouse button to start the selection and release it to end the selection\nPress right mouse button to try again\n");

		cvSetMouseCallback( window_name, on_mouse_region, params );

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
				cvSetImageROI(img2,cvRect(params[0],params[1],params[2] - params[0],params[3] - params[1]));
				region = cvCreateImage(cvSize(img2->roi->width,img2->roi->height),img2->depth,img2->nChannels);
				cvCopy(img2,region);
				cvResizeWindow(window_name,img2->roi->width,img2->roi->height);
				cvShowImage(window_name,region);
				printf("Press <Enter> to approve this selection or <Esc> to choose another one\n");
				while ((key!=27)&&(key!=13))
				{
					key = cvWaitKey();
				}
				if (key == 13)
					isEnd = true;
				else
				{
					cvReleaseImage(&region);
					isEnd = false;
				}

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
	

	cvReleaseImage(&img2);
	cvDestroyWindow(window_name);
	delete[] params;

	if (key==27)
		return NULL;

	return region;



	
}


CvPoint* getHoles(const IplImage* img, int& holes_count, int holes_type = 0)
{
	char window_name[128];
	strcpy(window_name,"Config generator: select holes");
	IplImage* img2 = cvCloneImage(img);
	cvNamedWindow(window_name);
	cvShowImage(window_name,img2);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* seq = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );

	printf("-----------------------\n");
	printf("Select holes by clicking with a left mouse button\nPress right mouse button to remove the point\nPress <Enter> to end the selection and <Esc> to cancel\n");
	cvSetMouseCallback( window_name, on_mouse_points, seq );

	CvPoint* test_point;
	int key = 0;
	int total= seq->total;
	CvScalar color = (holes_type == 0 ? CV_RGB(255, 255, 0) : CV_RGB(0, 255, 255));
	while ((key != 13)&&(key!=27))
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
				CvPoint* pt;
				for (int i=0; i< seq->total;i++)
				{
					pt = (CvPoint*)cvGetSeqElem(seq,i);
					cvCircle(img2,*pt,3,color);
				}
				total = seq->total;
				cvShowImage(window_name,img2);
			}
	}

	cvDestroyWindow(window_name);
	cvReleaseImage(&img2);

	if (key == 27)
	{
		cvReleaseMemStorage(&storage);
		holes_count = 0;
		return NULL;
	}
	else
	{
		holes_count = seq->total;
		CvPoint* arr = new CvPoint[holes_count];
		cvCvtSeqToArray(seq,arr);
		cvReleaseMemStorage(&storage);
		return arr;
	}


}
int main(int argc,char** argv)
{

	char image_path[1024], config_dir_path[1024], pca_dir_path[1024],
		pca_config_filename[128], pca_hr_config_filename[128], pca_descriptors_filename[128];
	char command[1024];
	char filename[1152];

	char window_name[256];
	strcpy(window_name,"Config generator");

	int patch_width = 12;
	int patch_height = 12;
	int pose_count = 500;
	FILE* f;


	strcpy(pca_config_filename,"pca_features_small.yml");
	strcpy(pca_hr_config_filename,"pca_features_large.yml");
	strcpy(pca_descriptors_filename,"pca_descriptors.yml");


	if (argc < 4)
	{
		printf ("Usage: confgen <path_to_image> <config dir path> <relative pca dir path>\n");
		return 0;
	}

	strcpy(image_path, argv[1]);
	strcpy(config_dir_path, argv[2]);
	strcpy(pca_dir_path, argv[3]);

	IplImage* image = cvLoadImage(image_path,1);

	if (image == NULL)
	{
		printf ("Unable to load image %s ... exiting\n",image_path);
		return 0;
	}


    sprintf(command, "mkdir %s", config_dir_path);
    system(command);

	//if (_access(config_dir_path,0) == -1)
	//{
	//	printf ("Unable to create config directory %s... exiting\n",config_dir_path);
	//	return 0;
	//}

	//sprintf(filename,"%s/%s",config_dir_path,pca_dir_path);
	//if (_access(pca_dir_path,0) == -1)
	//{
	//	printf ("PCA directory %s doesn't exist ... exiting\n",filename);
	//	return 0;
	//}

	sprintf(filename,"%s/%s/%s",config_dir_path,pca_dir_path,pca_config_filename);
	f = fopen(filename,"r");
	if (f == NULL)
	{
		printf ("File %s doesn't exist in %s... exiting\n",pca_config_filename,pca_dir_path);
		return 0;
	}
	fclose(f);

	sprintf(filename,"%s/%s/%s",config_dir_path,pca_dir_path,pca_hr_config_filename);
	f = fopen(filename,"r");
	if (f == NULL)
	{
		printf ("File %s doesn't exist in %s... exiting\n",pca_hr_config_filename,pca_dir_path);
		return 0;
	}
	fclose(f);

	sprintf(filename,"%s/%s/%s",config_dir_path,pca_dir_path,pca_descriptors_filename);
	f = fopen(filename,"r");
	if (f == NULL)
	{
		printf ("File %s doesn't exist in %s... exiting\n",pca_descriptors_filename,pca_dir_path);
		return 0;
	}
	fclose(f);




	printf("-----------------------\n");
	printf("Select a region with outlet\n");
	IplImage* outlet = getOutletRegion(image);

	if (outlet == NULL)
	{
		printf("You didn't select the region with outlet. Cannot generate config... exiting\n");
		return NULL;
	}

	printf("-----------------------\n");
	printf("Select a region without outlet\n");
	IplImage* nonoutlet = getOutletRegion(image);

	if (nonoutlet == NULL)
	{
		printf("You didn't select the region without outlet. Cannot generate config... exiting\n");
		return NULL;
	}

	CvPoint* ground_holes;
	CvPoint* power_holes;
	int ground_holes_count;
	int power_holes_count;


	bool isEndPower = true;
	do 
	{
		printf("-----------------------\n");
		printf("Select power holes\n");

		power_holes = getHoles(outlet,power_holes_count);

		if (power_holes == NULL)
		{
			printf("You didn't select power holes. Cannot generate config... exiting\n");
			return NULL;
		}

		if (power_holes_count%2 == 1)
		{
			printf("You selected odd number of power holes. Try again\n");
			isEndPower = false;
			delete[] power_holes;
		}
	}
	while (!isEndPower);

	bool isEndGround = true;

	do 
	{
		printf("-----------------------\n");
		printf("Select ground holes\n");

		ground_holes = getHoles(outlet,ground_holes_count,1);

		if (ground_holes == NULL)
		{
			printf("You didn't select ground holes. Cannot generate config... exiting\n");
			return NULL;
		}

		if (power_holes_count%ground_holes_count == 1)
		{
			printf("The number of power holes must be greater then number of ground holes in 2 times. Try again\n");
			isEndGround = false;
			delete[] ground_holes;
		}
	}
	while (!isEndGround);

	printf("Type 1 if the outlet is orange\nPress 2 if the outlet is white\nThen press <Enter>\n");
	int color = 2;
	scanf("%d",&color);


	sprintf(filename,"%s/outlet.jpg",config_dir_path);
	cvSaveImage(filename,outlet);
	cvReleaseImage(&outlet);
	sprintf(filename,"%s/nonoutlet.jpg",config_dir_path);
	cvSaveImage(filename,nonoutlet);
	cvReleaseImage(&nonoutlet);

	sprintf(filename,"%s/outlet_config.yml",config_dir_path);
	f = fopen(filename,"w");

	if (f==NULL)
	{
		printf("Unable to open file %s/outlet_config.yml for writing... exiting",config_dir_path);
		return 0;
	}

	fprintf(f,"%%YAML:1.0\noutlet:\n   outlet filename: \"outlet.jpg\"\n");
	for (int i=0;i<power_holes_count;i++)
	{
		fprintf(f,"   power%d:\n     - %d\n     - %d\n",i+1,power_holes[i].x,power_holes[i].y);
	}
	for (int i=0;i<ground_holes_count;i++)
	{
		fprintf(f,"   ground%d:\n     - %d\n     - %d\n",i+1,ground_holes[i].x,ground_holes[i].y);
	}
	fprintf(f,"nonoutlet:\n   nonoutlet filename: \"nonoutlet.jpg\"\n");
	fclose(f);

	sprintf(filename,"%s/outlet_template.yml",config_dir_path);
	f = fopen(filename,"w");

	if (f==NULL)
	{
		printf("Unable to open file %s/outlet_template.yml for writing... exiting",config_dir_path);
		return 0;
	}
	fprintf(f,"%%YAML:1.0\noutlet count: %d\n",ground_holes_count);
	fprintf(f,"train path: \"%s\"\ntrain config: \"outlet_config.yml\"\n",config_dir_path);
	sprintf(filename,"%s/%s",pca_dir_path, pca_config_filename);
	fprintf(f,"pca config: \"%s\"\n",filename);
	sprintf(filename,"%s/%s",pca_dir_path, pca_hr_config_filename);
	fprintf(f,"pca hr config: \"%s\"\n",filename);
	sprintf(filename,"%s/%s",pca_dir_path, pca_descriptors_filename);
	fprintf(f,"pca descriptors: \"%s\"\n",filename);
	fprintf(f,"patch width: %d\npatch height: %d\npose count: %d\n",patch_width,patch_height,pose_count);
	fprintf(f,"outlet color: %s\nhole contrast: %f\n", color==1 ? "orange" : "white",color==1 ? ORANGE_CONTRAST : WHITE_CONTRAST);


	fclose(f);

	sprintf(filename,"%s/command_line_options.txt",config_dir_path);
	f = fopen(filename,"w");
	fprintf(f,"<path_to_images> <filelist.txt> <camera_config> %s [<output_path>]",config_dir_path);
	fclose(f);

	printf("Config generation complete!\n Now you can use following command to run outlet detection:\noutlet_model <path_to_images> <filelist.txt> <camera_config> %s [<output_path>]\n",config_dir_path);







	cvReleaseImage(&image);
	return 0;
}
