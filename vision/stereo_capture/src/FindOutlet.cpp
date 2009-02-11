// Find outlets using stereo texture
// Gary Bradski, (c) Willow Garage 1/22/2009
//
#include "cv.h"
#include "highgui.h"
#include "stdio.h"
#include "math.h"

/* CANNY PRODUCES INTERESTING RESULTS ... WS THINKING EDGE HISTOGRAM BUT HAVEN'T TRIED YET ...
IplImage* doCanny(
    IplImage* in,
    double    lowThresh,
    double    highThresh,
    int    aperture)
{
    if (in->nChannels != 1)
        return(0); // Canny only handles gray scale images
    IplImage* out = cvCreateImage(
        cvGetSize( in ),
        in->depth, //IPL_DEPTH_8U,   
        1);
    cvCanny( in, out, lowThresh, highThresh, aperture );
    return( out );
};
 // IplImage* img_cny = doCanny( img_L, 10.0, 100.0, 3 );
 // cvShowImage("Disparity", img_cny );
*/

void help()
{
    printf("\n"
    "Segment outlets using stereo texture detection on walls\n"
    "Call:\n"
    "./FindOutlet Left_image Disarity_image Base_name \n"
    "Keyboard commands:\n"
 //   "\tn -- Change the histogram model base name\n"
    "\tt -- Show the found boxes, SPACE to continue ...\n"
    "\nMouse\n"
//    "\t Clicking on patch in disparity image will cause a histogram model to be saved.\n"
    "\n");
}
/////WORKING FUNCTIONS////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// JUST A MODIFIED CONNECTED COMPONENTS ROUTINE
//void cvconnectedDisparityComponents(IplImage *mask, int poly1_hull0, float perimScaleTooSmall, int *num, CvRect *bbs, CvPoint *centers)
// This will find the blobs in the image.  I 
//
// mask			Is a grayscale (8 bit depth) "raw" mask image which will be cleaned up
//
// OPTIONAL PARAMETERS:
// poly1_hull0				If set, approximate connected component by (DEFAULT) polygon, or else convex hull (0)
// areaTooSmall			Kill contours whose bounding box area is less than this
// areaTooLarg				Kill contours whose bounding box area is more than this
// aspectLimit				Enforce width > height. Then aspect = height/width.  Kill aspects < aspectLimit.
// num						Maximum number of rectangles and/or centers to return, on return, will contain number filled (DEFAULT: NULL)
// bbs						Pointer to bounding box rectangle vector of length num.  (DEFAULT SETTING: NULL)
// centers					Pointer to contour centers vectore of length num (DEFULT: NULL)
//
// KNOWN PROBLEM!!!  If a rejected contour compltely surrounds another contour, the whole area is deleted
//
#define CVCONTOUR_APPROX_LEVEL  2   // Approx.threshold - the bigger it is, the simpler is the boundary
#define CV_CVX_WHITE	CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK	CV_RGB(0x00,0x00,0x00)
void cvconnectedDisparityComponents(IplImage *mask, int poly1_hull0, float areaTooSmall, float areaTooLarge, float aspectLimit, int *num, CvRect *bbs, CvPoint *centers)
{
static CvMemStorage*	mem_storage	= NULL;
static CvSeq*			contours	= NULL;


//FIND CONTOURS AROUND ONLY BIGGER REGIONS
	if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
    else cvClearMemStorage(mem_storage);

	CvContourScanner scanner = cvStartFindContours(mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	CvSeq* c;
	int numCont = 0;
	CvRect bb;
	//FILTER LOOP:
	int ccnt = 0;
	while( (c = cvFindNextContour( scanner )) != NULL )
	{
		printf("CONTOR %d:\n",ccnt++);
		bb = cvBoundingRect(c);
		float wd = (float)(bb.width);
		float ht = (float)(bb.height);
		float carea = (float)(wd*ht);
		printf("  Area = %f vs too small(%f) and too large (%f)\n",carea,areaTooSmall,areaTooLarge);
		float aspect;
		if(wd > ht) //Keep this number (0,1]
			aspect = ht/wd;
		else
			aspect = wd/ht;
		printf("  ... aspectLimit(%f) vs aspect(%f)\n",aspectLimit,aspect);
		if( carea <  areaTooSmall || carea > areaTooLarge || (aspect < aspectLimit)) //Get rid of blob if it's too small or too large
		{
			printf("  DELETED\n"); //If bad contour surrounds a good one, both are delteted here :-(
			cvSubstituteContour( scanner, NULL );
		}
		else //Smooth it's edges if it's large enough
		{
			CvSeq* c_new;
			if(poly1_hull0) //Polygonal approximation of the segmentation
	            c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
			else //Convex Hull of the segmentation
				c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
            cvSubstituteContour( scanner, c_new );
			numCont++;
        }
	}
	contours = cvEndFindContours( &scanner );

	//COMPUTE LOOP:
  	// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
	cvZero( mask );
	IplImage *maskTemp;
	//CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
	if(num != NULL)
	{
		int N = *num, numFilled = 0, i=0;
		CvMoments moments;
		double M00, M01, M10;
		maskTemp = cvCloneImage(mask);
		for(i=0, c=contours; c != NULL; c = c->h_next,i++ )
		{
			if(i < N) //Only process up to *num of them
			{
				cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
				//Find the center of each contour
				if(centers != NULL)
				{
					cvMoments(maskTemp,&moments,1);
					M00 = cvGetSpatialMoment(&moments,0,0);
					M10 = cvGetSpatialMoment(&moments,1,0);
					M01 = cvGetSpatialMoment(&moments,0,1);
					centers[i].x = (int)(M10/M00);
					centers[i].y = (int)(M01/M00);
				}
				//Bounding rectangles around blobs
				if(bbs != NULL)
				{
					bbs[i] = cvBoundingRect(c);
				}
				cvZero(maskTemp);
				numFilled++;
			}
			//Draw filled contours into mask
			cvDrawContours(mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
		} //end looping over contours
		*num = numFilled;
		cvReleaseImage( &maskTemp);
	}
	//ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
	else
	{
		for( c=contours; c != NULL; c = c->h_next )
		{
			cvDrawContours(mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
		}
	}
}

/////////////////////////////////////////////////
// Analyze the disparity image that values should not be too far off from one another
// Id  -- 8 bit, 1 channel disparity image
// R   -- rectangular region of interest
// vertical -- This is a return that tells whether something is on a wall (descending disparities) or not.
// minDisparity -- disregard disparities less than this
//
double disparitySTD(IplImage *Id, CvRect &R, bool &vertical, double minDisparity = 0.5 )
{
	int ws = Id->widthStep;
	unsigned char *p = (unsigned char *)(Id->imageData);
	int rx = R.x;
	int ry = R.y;
	int rw = R.width;
	int rh = R.height;
	int nchan = Id->nChannels;
	p += ws*ry + rx*nchan; //Put at start of box
	double mean = 0.0,var = 0.0;
	double val;
	int cnt = 0;
	//For vertical objects, Disparities should decrease from top to bottom, measure that 
	double AvgTopDisp = 0.0;
	double AvgBotDisp = 0.0;
	int top_cnt = 0;
	int bot_cnt = 0;
	bool top = true;
	int halfwayY = rh/2;
	for(int Y=0; Y<rh; ++Y)
	{
		for(int X=0; X<rw; X++, p+=nchan)
		{
			val = (double)*p;
			if(val < minDisparity)
				continue;
			if(top){
				AvgTopDisp += val;
				top_cnt++;
			}
			else {
				AvgBotDisp += val;
				bot_cnt++;
			}
			mean += val;
			var += val*val;
			cnt++;
		}
		p+=ws-(rw*nchan);
		if(Y >= halfwayY)
			top = false; 
	}
	if(cnt == 0) //Error condition, no disparities, return impossible variance
	{
		vertical = false;
		return 10000000.0;
	}
	//FIND OUT IF THE OBJECT IS VERTICAL (Descending disparities) OR NOT:
	if(top_cnt == 0) top_cnt = 1;
	if(bot_cnt == 0) bot_cnt = 1;
	AvgTopDisp = AvgTopDisp/(double)top_cnt;
	AvgBotDisp = AvgBotDisp/(double)bot_cnt;
	if(AvgTopDisp >= AvgBotDisp)
		vertical = true;
	else
		vertical = false;	
	//DO THE VARIANCE MATH
	mean = mean/(double)cnt;
	var = (var/(double)cnt) - mean*mean;
	return(sqrt(var));
}
			
////////////// 
// UTILITY: Get an input string from a user (white space is ignored, ESC cancels, 255 chars max)
bool getUserString(char *ObjectLabel)
{
	char ObjectStore[256];
	int ii = (int)strlen(ObjectLabel);
	int jj;
	char c = 0;
	printf("\nObject path\\name: (don't use extensions or funny characters)\nESC to cancel\n");
	printf("%s\r",ObjectLabel); fflush (stdout);
	strcpy(ObjectStore,ObjectLabel);
	while((c != 27)&&(c != 13)&&(c != 10)&&(ii < 254)) //27=ESC, 13=CR, 10=LF, 
	{
		c = cvWaitKey()&0xFF;
		if(c == 13) break;
		if(c == ' ') continue;
		if(c == '\t') continue;
		if(c == 8) //Backspace
		{
			printf("\r");
			for(jj = 0; jj<=ii; jj++)
				printf(" ");
			printf("\r");
			ii -= 1;
			if (ii <= 0) 
				ii = 0;
			ObjectLabel[ii] = 0;
		}
		else
		{
			ObjectLabel[ii] = c;
			ii++;
			ObjectLabel[ii] = 0;
		}
		printf("%s\r",ObjectLabel); fflush (stdout);

	} 
	bool retval = true;
	if(c == 27) //ESC => restore old label state
	{
		strcpy(ObjectLabel,ObjectStore);
		retval = false;
	}
	printf("\nFinal label:\n%s\n",ObjectLabel);	
	return retval;
}	

/*  SIMPLE COLOR HISTOGRAMS DON'T WORK VERY WELL ...
//////////////
// Fill up a histogram using rectangle as the ROI
// I3C		-- 3 channel image (input to be processed)
// Iplanes	-- 3 single channel images of same size to split I3C into
// H3D		-- 3 d histogram
// R			-- Rectangle to use
//
void fill3DHistROI(IplImage *I3C, IplImage **Iplanes, CvHistogram *H3D, CvRect &R)
{
	//Set up to calculate histogram
	cvSetImageROI(I3C,R);
	cvSetImageROI(Iplanes[0],R);
	cvSetImageROI(Iplanes[1],R);
	cvSetImageROI(Iplanes[2],R);
	cvSplit(I3C, Iplanes[0], Iplanes[1], Iplanes[2], 0);
	//Calculate histogram
	cvCalcHist(Iplanes, H3D);//, 0, Imask);//Clear, then calculate histogram in the corrsponding mask area of the images
	cvNormalizeHist(H3D, (double)1.0); //Always deal with normalized histograms
	//Clean up
	cvResetImageROI(I3C);
	cvResetImageROI(Iplanes[0]);
	cvResetImageROI(Iplanes[1]);
	cvResetImageROI(Iplanes[2]);
}
*/	

////SLIDERS ETC///////////////////////
bool wupdate = true;  //Update whenever there is a change
int areaTooSmall = 25; //bounding box area

bool do_blobs = true;  
void on_areaSmall(int t)
{
    do_blobs = true;
}
int areaTooLarge = 160; //bounding box area
void on_areaLarge(int t)
{
    do_blobs = true;
}

int aspectLimit = 45;  //This will be (float)aspectLimit/100.0  
void on_aspectLimit(int t)
{
	do_blobs = true;
}

/*  I HAD USED THIS TO COLLECT HISTOGRAM TRAINING MODELS
 int mx, my;
bool mouse_lclick = false;
void on_mouseD(int event, int x, int y, int flags, void* param)
{
	if(event == CV_EVENT_LBUTTONDOWN)
	{
		mx = x;
		my = y;
		do_blobs = true;
		mouse_lclick = true;
	}
}	
*/
/////////////////////////////
int main( int argc, char** argv )
{
  if(argc < 4)
  {
     help();
    exit(0);
  }
  IplImage *img_L = NULL, *img_D = NULL,*img_Lt = NULL,*img_Dt = NULL;

  img_L = cvLoadImage( argv[1]);
  img_D = cvLoadImage( argv[2],CV_LOAD_IMAGE_GRAYSCALE);
 // img_Lt = cvLoadImage( argv[3]);
 // img_Dt = cvLoadImage( argv[4],CV_LOAD_IMAGE_GRAYSCALE);
	char tstbasename[512];
 	strcpy(tstbasename,argv[3]);
 	printf("Verify basename = %s\n",tstbasename);
 //	float ChiSqrThresh = atof(argv[4]);
 //	float BhattThresh = atof(argv[5]);

  if(!img_L || !img_D)
  {
    printf("Error: Couldn't load %s or %s\n",argv[1],argv[2]);
    exit(-1);
  }
  IplImage *img_Lcpy = cvCloneImage(img_L);
  IplImage *img_S = cvCloneImage(img_D);

  cvCopy(img_D,img_S);
  cvNamedWindow("Left", CV_WINDOW_AUTOSIZE );
  cvShowImage("Left", img_L);
  cvNamedWindow("Disparity", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("Segmentation",CV_WINDOW_AUTOSIZE);

  int Nerode = 1;
  int Ndialate = 9;
 /* if(argc > 3)
    Nerode = atoi(argv[3]);
  if(argc > 4)
    Ndialate = atoi(argv[4]);   
printf("Nerode = %d, Ndialate=%d\n",Nerode,Ndialate);
*/
  cvCreateTrackbar("areaTooSmall","Disparity",&areaTooSmall, 500, on_areaSmall);
  cvCreateTrackbar("areaTooLarge","Disparity",&areaTooLarge, 500, on_areaLarge);
  cvCreateTrackbar("aspectLimits","Disparity",&aspectLimit, 100, on_aspectLimit);
 // cvSetMouseCallback( "Disparity", on_mouseD);

 
  //PREPROCESS
  cvErode(img_S,img_S,NULL, Nerode); //Probably 1  NOTE: Morphology on the gray, not binary disparity Segmentation image "S"
  cvDilate(img_S,img_S,NULL,Ndialate); //Probably 9
  cvErode(img_S,img_S,NULL, Ndialate);
  IplImage *img_Scpy = cvCloneImage(img_S);
  cvNamedWindow("Preprocess",CV_WINDOW_AUTOSIZE);
  cvShowImage("Preprocess",img_S);
  
  //SET UP MODEL COLLECTION
/*
 *   char modelbasename[256];
  sprintf(modelbasename,"Hist");
 // int modelCount = 0;
  int dims =  3;
  int hist_size[] = {8,8,8}; 

	CvHistogram *Hbad1 = (CvHistogram *)cvLoad("Hbad1.xml"); 
	CvHistogram *Hbad2 = (CvHistogram *)cvLoad("Hbad2.xml"); 
	//	CvHistogram *Hmodel1 = cvCreateHist(dims, hist_size, CV_HIST_ARRAY, NULL, 1);
	CvHistogram *Hmodel1 = (CvHistogram *)cvLoad("Hist1.xml");//cvCreateHist(dims, hist_size, CV_HIST_ARRAY, NULL, 1);
	CvHistogram *Hmodel2 = (CvHistogram *)cvLoad("Hist2.xml"); 
	CvHistogram *Hmodel3 = (CvHistogram *)cvLoad("Hist3.xml"); 
	CvHistogram *Htest = cvCreateHist(dims, hist_size, CV_HIST_ARRAY, NULL, 1);
   IplImage* b_plane = cvCreateImage( cvGetSize(img_L), 8, 1 );
   IplImage* g_plane = cvCreateImage( cvGetSize(img_L), 8, 1 );
   IplImage* r_plane = cvCreateImage( cvGetSize(img_L), 8, 1 );
   IplImage* Iplanes[] = { b_plane, g_plane, r_plane };
*/
 
  //PROCESSING LOOP
  bool vertical = false;
  while(1)
  {
    //THRESHOLD FUN
    if(do_blobs){
		cvCopy(img_Scpy,img_S);
		cvCopy(img_Lcpy,img_L);
		int num = 20;
		CvRect bbs[20];
		CvPoint centers[20];
		
		cvconnectedDisparityComponents(img_S, 1, areaTooSmall*areaTooSmall,areaTooLarge*areaTooLarge,(float)(aspectLimit)/100.0, &num, bbs, centers);
				
		//DO FILTERING BASED ON BOX SIZE:
		for(int b=0; b<num; ++b)
		{
			printf("%d: ",b);
/* FROM WHEN I WAS COLLECTING HISTOGRAM MODELS ...
 * 			if(mouse_lclick)
			{
			   //If we are in a bounding box, collect a model
				if((mx >= bbs[b].x) &&
					(mx <= (bbs[b].x + bbs[b].width)) &&
					(my >= bbs[b].y) &&
					(my <= (bbs[b].y + bbs[b].height)))
				{
					fill3DHistROI(img_L, Iplanes, Hmodel1, bbs[b]);		
					printf("Out of ill3DHist\n");	
					CvPoint ul = cvPoint(bbs[b].x,bbs[b].y);
					CvPoint lr = cvPoint(bbs[b].x + bbs[b].width, bbs[b].y + bbs[b].height);
					cvRectangle(img_L,ul,lr,CV_RGB(0,255,0));
					char hname[512];
					sprintf(hname,"%s.xml",modelbasename);
					cvSave(hname, Hmodel1);
					printf("Saved %s to disk\n",hname);
				}
			}
*/				
			printf("** STD = %lf, Vertical=%d\n",disparitySTD(img_Scpy, bbs[b],vertical),vertical);
			printf("\n");
		 }
       do_blobs = false;
//       mouse_lclick = false;
       wupdate = true;
    }
  
	//KEYBOARD PROCESSING
	int k = cvWaitKey(10);
   	//End processing on ESC, q or Q
    if(k == 27 || k == 'q' || k == 'Q')
       	break;
     switch(k)
     {
 //    	case 'n':  //Change the model base name
 //    	   printf("Change the hisogram model basename (was %s)\n\n",modelbasename);
 //     	getUserString(modelbasename);
 //     	break;
      case 't': //Test
      {
			int numt = 20;
			CvRect bbst[20];
			CvPoint centerst[20];
		   cvNamedWindow("Results", CV_WINDOW_AUTOSIZE );
  		   cvNamedWindow("TestDisparity", CV_WINDOW_AUTOSIZE );
  			char leftName[512],dispName[512];
  			IplImage *img_Ltdisplay = NULL,*img_Dclone = NULL;

			printf("\nTESTING: (Corr, Inter, ChiSqr, Bhatt)\n");
			for(int img = 0; img<29; ++img)
			{
				numt = 20;
				sprintf(leftName,"%sL%d.png",tstbasename,img);
				sprintf(dispName,"%sd%d.png",tstbasename,img);
				printf("IMAGE %d (%s, %s)\n",img,leftName,dispName);
				img_Lt = cvLoadImage( leftName);
				img_Ltdisplay = cvCloneImage(img_Lt);
				img_Dt = cvLoadImage( dispName,CV_LOAD_IMAGE_GRAYSCALE);
				img_Dclone = cvCloneImage(img_Dt);
				  cvErode(img_Dt,img_Dt,NULL, Nerode); //Probably 1
				  cvDilate(img_Dt,img_Dt,NULL,Ndialate); //Probably 9
				  cvErode(img_Dt,img_Dt,NULL, Ndialate);				
				cvconnectedDisparityComponents(img_Dt, 1, areaTooSmall*areaTooSmall,
														areaTooLarge*areaTooLarge,(float)(aspectLimit)/100.0, 
														&numt, bbst, centerst);
			

// SIMPLE COLOR HISTOGRAM MATCHING DOESN'T WORK STABLY... VICTOR TRY SOMETHING ELSE
/*				double match;
				double bestmatch = 1.0; //min is better for chisqr
				double badmatch = 1.0; //Model for non-outlet
				for(int t = 0; t<numt; ++t)
				{
					bestmatch = badmatch = 1.0;
					printf("BOX %d:\n",t);
					fill3DHistROI(img_Lt, Iplanes, Htest, bbst[t]);	
					
					bestmatch = cvCompareHist(Hmodel1,Htest,CV_COMP_CHISQR);
					match = cvCompareHist(Hmodel2,Htest,CV_COMP_CHISQR);
					if(match < bestmatch) bestmatch = match;
					match = cvCompareHist(Hmodel3,Htest,CV_COMP_CHISQR);
					if(match < bestmatch) bestmatch = match;
					
					badmatch = cvCompareHist(Hbad1,Htest,CV_COMP_CHISQR);
					match = cvCompareHist(Hbad2,Htest,CV_COMP_CHISQR);
					if(match < badmatch) badmatch = match;
					printf("bestmatch(%lf), badmatch(%lf) < Thresh(%f)\n",bestmatch,badmatch,ChiSqrThresh);

					//DRAW SUCCESS
					CvPoint pt1 = cvPoint(bbst[t].x,bbst[t].y);
					int wi = bbst[t].width;
					int hi = bbst[t].height;
					CvPoint pt2 = cvPoint(bbst[t].x+wi,bbst[t].y+hi);
					int intensity = 100 + 40*t;
					if(intensity > 255) intensity = 255;
					
					cvLine(img_Ltdisplay,pt1,pt2,CV_RGB(intensity,0,0));
					if((bestmatch<ChiSqrThresh)&&(bestmatch < badmatch)&&(badmatch > BhattThresh))
						cvRectangle(img_Ltdisplay,pt1,pt2,CV_RGB(0,255,0));

					
				}//end for each box
*/
//INSTEAD, FOR NOW, JUST DRAW THE BOUNDING BOXES FOUND ... VICTOR, THIS SHOULD BE REPLACED BY REAL MATCHING
				for(int t=0; t<numt; ++t)
				{
					CvPoint pt1 = cvPoint(bbst[t].x,bbst[t].y);
					int wi = bbst[t].width;
					int hi = bbst[t].height;
					printf("Box %d: w=%d, h=%d\n",t,wi,hi);
					double Disp = disparitySTD(img_Dclone, bbst[t],vertical);
					printf("** STD = %lf, vertical=%d\n",Disp,vertical);
					if(Disp >= 4.8) continue;
					if(!vertical) continue;	
					CvPoint pt2 = cvPoint(bbst[t].x+wi,bbst[t].y+hi);
					int intensity = 255 - 40*t;
					if(intensity < 60) intensity = 60;
					cvRectangle(img_Ltdisplay,pt1,pt2,CV_RGB(intensity,0,0));		
				}			

				cvShowImage("Results", img_Ltdisplay);
				cvShowImage("TestDisparity",img_Dclone);
				int c;
				while(1)
				{
					c = cvWaitKey(33);
					if(c == 27 || c == 'q' | c == 'Q' || c == ' ')
       				break;
  				}
				if(c == 27 || c == 'q' || c == 'Q')
     				break;
 				cvReleaseImage( &img_Lt);
				cvReleaseImage( &img_Ltdisplay);
				cvReleaseImage( &img_Dt);
				cvReleaseImage( &img_Dclone);
			}//end for each image
			cvDestroyWindow("Results");
			cvDestroyWindow("TestDisparity");
			printf("DONE TESTING IMAGES.\n");
		} 
			break;
//		default:
//            ;
     	}
     
	//IF UPDATING WINDOW
    if(wupdate)
    {
        wupdate = false;
        cvShowImage("Left", img_L);
        cvShowImage("Segmentation", img_S);
        cvShowImage("Disparity", img_D );
    }
       
  }//END WHILE

  //CLEAN UP
  cvReleaseImage( &img_L);
  cvReleaseImage( &img_S);
  cvReleaseImage( &img_D);
  
//  cvReleaseHist( &Hmodel1);
//	cvReleaseHist( &Htest);
  
  cvDestroyWindow("Left");
  cvDestroyWindow("Disparity");
  cvDestroyWindow("Segmentation");
  exit(0);
}
