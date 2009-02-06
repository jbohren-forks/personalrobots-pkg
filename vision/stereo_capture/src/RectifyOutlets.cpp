#include <stdio.h>

#include <cv.h>
#include <highgui.h>

#include "outlet_tuple.h"


int main (int argc, char * const argv[]) 
{
	cvNamedWindow("Original",1);
	for(int i = 1; i < 62; i++)
	{
		char buf[1024];
		sprintf(buf, "/u/bradski/data/Outlets1.21.09/V%03d.png", i);
		IplImage* img = cvLoadImage(buf);
		CvPoint2D32f centers[4];
		find_outlet_centroids(img, centers);

		CvMat* map_matrix = cvCreateMat(3, 3, CV_32FC1);
		calc_outlet_homography(centers, map_matrix);
		
		IplImage* imgr = cvCloneImage(img);
		cvWarpPerspective(img, imgr, map_matrix);
	
#if 1
		cvNamedWindow("1", 1);
		cvShowImage("Original",img);
		cvShowImage("1", imgr);
		cvWaitKey(0);
#endif

		cvReleaseImage(&imgr);
		cvReleaseImage(&img);
	}
}
