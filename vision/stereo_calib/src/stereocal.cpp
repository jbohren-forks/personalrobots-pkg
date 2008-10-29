/*#########################################
 * stereocal.cpp
 *
 * Main functions for stereo cal GUI
 *
 *#########################################
 */

/**
 ** stereocal.cpp
 **
 ** Kurt Konolige
 ** Senior Researcher
 ** Willow Garage
 ** 68 Willow Road
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@willowgarage.com
 **
 **/


#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>
#include <sys/time.h>
#include "stereogui.h"
#include "stereolib.h"
#include <opencv/cv.h>
#include <opencv/cxmisc.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

using namespace std;

//
// Stereo cal application
// Read in images
// Calibrate and write out parameters
//

stereogui *stg;			// GUI object

#define MAXIMAGES 20

// image storage, need it for undistortions
IplImage *imgs_left[MAXIMAGES];
IplImage *imgs_right[MAXIMAGES];
IplImage *rect_left[MAXIMAGES];
IplImage *rect_right[MAXIMAGES];

// corner storage
int num_x_ints = 8;		// number of x and y corners
int num_y_ints = 6;
int num_pts = 6*8;		// number of total points

CvPoint2D32f *leftcorners[MAXIMAGES];
CvPoint2D32f *rectleftcorners[MAXIMAGES];
CvPoint2D32f *rightcorners[MAXIMAGES];
CvPoint2D32f *rectrightcorners[MAXIMAGES];
int nleftcorners[MAXIMAGES];
int nrightcorners[MAXIMAGES];

// good images
bool goodleft[MAXIMAGES];
bool goodright[MAXIMAGES];
bool goodpair[MAXIMAGES];	// true if features found

// image size in pixels, shouldn't change across images
CvSize imsize_left, imsize_right;

// target square size
double squareSize = 0.100;	// target square size in m, default 100 mm

// rectification mappings
IplImage *mapx_left, *mapy_left, *mapx_right, *mapy_right;

// intrinsics and extrinsics
double K1[3][3], K2[3][3];	// camera matrices
double D1[5], D2[5];		// distortion coefficients, k1, k2, t1, t2, k3
double OM[3], T[3];		// stereo camera 3D relative pose
CvMat K_left  = cvMat(3, 3, CV_64F, K1 );
CvMat K_right = cvMat(3, 3, CV_64F, K2 );
CvMat D_left  = cvMat(1, 5, CV_64F, D1 );
CvMat D_right = cvMat(1, 5, CV_64F, D2 );
CvMat Rotv    = cvMat(3, 1, CV_64F, OM );
CvMat Transv  = cvMat(3, 1, CV_64F, T );

// helpers
void set_current_tab_index(int ind);
bool parse_filename(char *fname, char **lbase, char **rbase, int *num);
int load_left(char *fname);
int load_right(char *fname);


// stereo function headings and globals
double epi_scanline_error(bool horz = true); // <horz> = 0 for horizontal, 1 for vertical epilines
double get_ms();		// for timing

bool is_fixed_aspect = true;
bool is_zero_disparity = true;
bool use_kappa1 = true;
bool use_kappa2 = true;
bool use_kappa3 = false;
bool use_tau    = false;

// printing matrices
void PrintMat(CvMat *A);


// main program, just put up the GUI dialog

int
main(int argc, char **argv)	// no arguments
{
  Fl_Window *gwin;

  // initialize data structures
  for (int i=0; i<MAXIMAGES; i++)
    {
      imgs_left[i] = imgs_right[i] = rect_left[i] = rect_right[i] = NULL;
      leftcorners[i] = NULL;
      rectleftcorners[i] = NULL;
      rightcorners[i] = NULL;
      rectrightcorners[i] = NULL;
      nleftcorners[i] = 0;
      nrightcorners[i] = 0;
      goodpair[i] = goodleft[i] = goodright[i] = false;
    }
  mapx_left = mapy_left = mapx_right = mapy_right = NULL;



  // start up dialog window
  stg = new stereogui;
  gwin = stg->stereo_calibration;
  set_current_tab_index(1);
  gwin->show();

  while (fltk_check())		// process GUI commands
    {}
}


//
// utility fns
//

void
info_message(char *str, ...)	// print in the info line
{
  char buf[1024];
  va_list ptr;
  va_start(ptr,str);
  if (str)
    {
      vsprintf(buf,str,ptr);
      stg->info_message->value(buf);
      printf("%s\n",buf);
    }
}


int
get_current_tab_index()
{
  Fl_Tabs *tb = (Fl_Tabs *)stg->window_tab;
  Fl_Widget *wd;
  wd = tb->value();
  int ind = (int)wd->user_data();
  return ind;
}

void
set_current_tab_index(int ind)
{
  Fl_Tabs *tb = (Fl_Tabs *)stg->window_tab;
  Fl_Widget *wd;
  wd = tb->child(ind);
  tb->value(wd);
}


calImageWindow *
get_current_left_win()
{
  int ind = get_current_tab_index();
  if (ind < 0)
    return NULL;
  switch (ind)
    {
    case 0:
      return stg->calLeft1;
    case 1:
      return stg->calLeft2;
    case 2:
      return stg->calLeft3;
    case 3:
      return stg->calLeft4;
    case 4:
      return stg->calLeft5;
    case 5:
      return stg->calLeft6;
    case 6:
      return stg->calLeft7;
    case 7:
      return stg->calLeft8;
    case 8:
      return stg->calLeft9;
    case 9:
      return stg->calLeft10;
    case 10:
      return stg->calLeft11;
    case 11:
      return stg->calLeft12;
    case 12:
      return stg->calLeft13;
    case 13:
      return stg->calLeft14;
    case 14:
      return stg->calLeft15;
    case 15:
      return stg->calLeft16;
    case 16:
      return stg->calLeft17;
    case 17:
      return stg->calLeft18;
    case 18:
      return stg->calLeft19;
    case 19:
      return stg->calLeft20;
    }
  return NULL;
}

calImageWindow *
get_current_right_win()
{
  int ind = get_current_tab_index();
  if (ind < 0)
    return NULL;
  switch (ind)
    {
    case 0:
      return stg->calRight1;
    case 1:
      return stg->calRight2;
    case 2:
      return stg->calRight3;
    case 3:
      return stg->calRight4;
    case 4:
      return stg->calRight5;
    case 5:
      return stg->calRight6;
    case 6:
      return stg->calRight7;
    case 7:
      return stg->calRight8;
    case 8:
      return stg->calRight9;
    case 9:
      return stg->calRight10;
    case 10:
      return stg->calRight11;
    case 11:
      return stg->calRight12;
    case 12:
      return stg->calRight13;
    case 13:
      return stg->calRight14;
    case 14:
      return stg->calRight15;
    case 15:
      return stg->calRight16;
    case 16:
      return stg->calRight17;
    case 17:
      return stg->calRight18;
    case 18:
      return stg->calRight19;
    case 19:
      return stg->calRight20;
    }
  return NULL;
}


//
// callbacks
//

// Load left image into image pair
// Also find chessboard points

void
cal_load_left_cb(Fl_Button* b, void* arg)
{
  char *fname = fl_file_chooser("Load file", NULL, NULL);
  if (fname == NULL)
    return;

  printf("File name: %s\n", fname);

  load_left(fname);
}


int
load_left(char *fname)
{
  IplImage *dbg_corners = 0;


  // get window tab
  int ind = get_current_tab_index();

  IplImage *im;			// OpenCV image

  calImageWindow *cwin = get_current_left_win();

  im = cvLoadImage(fname);	// load image, could be color
  if (im == NULL)
    printf("Can't load file %s\n", fname);
  else
    {
      // make grayscale, display
      IplImage* img=cvCreateImage(cvGetSize(im),IPL_DEPTH_8U,1);
      dbg_corners = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3 );
      cvConvertImage(im,img);
      imgs_left[ind] = img;
      imsize_left = cvGetSize(im);
      info_message("Size: %d x %d, ch: %d", img->width, img->height, img->nChannels);
      cwin->DisplayImage((unsigned char *)img->imageData, img->width, img->height, img->width);

//*********************************
//      return ind;

      // find corners
      if (leftcorners[ind])
	delete [] leftcorners[ind];
      leftcorners[ind] = new CvPoint2D32f[num_x_ints*num_y_ints];
      rectleftcorners[ind] = new CvPoint2D32f[num_x_ints*num_y_ints];
      int numc = 0;
      int ret = cvFindChessboardCorners(img, cvSize(num_x_ints, num_y_ints),
		leftcorners[ind], &numc, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
      nleftcorners[ind] = numc;
      info_message("Found %d corners", numc);
      cwin->display2DFeatures(leftcorners[ind],numc,ret);

      // do subpixel calculation, if corners have been found
      if (ret)
	{
	  cvFindCornerSubPix(img, leftcorners[ind], numc,
			     cvSize(5,5),cvSize(-1,-1),
			     cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	}

      // set good/bad image
      goodleft[ind] = ret;

      cvReleaseImage(&im);
      return ind;
    }
  return -1;
}


// Load right image into image pair
// Also find chessboard points

void
cal_load_right_cb(Fl_Button* b, void* arg)
{
  char *fname = fl_file_chooser("Load file", NULL, NULL);
  if (fname == NULL)
    return;

  printf("File name: %s\n", fname);

  load_right(fname);
}


int
load_right(char *fname)
{
  // get window tab
  int ind = get_current_tab_index();

  IplImage *im;			// OpenCV image

  calImageWindow *cwin = get_current_right_win();


  im = cvLoadImage(fname);	// load image, could be color
  if (im == NULL)
    printf("Can't load file %s\n", fname);
  else
    {
      // convert to grayscale
      IplImage* img=cvCreateImage(cvGetSize(im),IPL_DEPTH_8U,1);
      cvConvertImage(im,img);
      imgs_right[ind] = img;
      imsize_right = cvGetSize(im);
      info_message("Size: %d x %d, ch: %d", img->width, img->height, img->nChannels);
      cwin->DisplayImage((unsigned char *)img->imageData, img->width, img->height, img->width);

      // find corners
      if (rightcorners[ind])
	delete [] rightcorners[ind];
      rightcorners[ind] = new CvPoint2D32f[num_x_ints*num_y_ints];
      rectrightcorners[ind] = new CvPoint2D32f[num_x_ints*num_y_ints];
      int numc = 0;
      int ret = cvFindChessboardCorners(img, cvSize(num_x_ints, num_y_ints),
					rightcorners[ind], &numc,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
      nrightcorners[ind] = numc;
      info_message("Found %d corners", numc);
      cwin->display2DFeatures(rightcorners[ind],numc,ret);

//*********************************
//      return ind;

      // do subpixel calculation, if corners have been found
      if (ret)
	{
	  cvFindCornerSubPix(img, rightcorners[ind], numc,
			   cvSize(5,5),cvSize(-1,-1),
			   cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	}

      // set good/bad image
      goodright[ind] = ret;

      cvReleaseImage(&im);
      return ind;
    }

  return -1;
}


// Load both images of a pair, or a single image

void cal_load_cb(Fl_Button*, void*)
{
  // get window tab
  char *fname = fl_file_chooser("Load file", NULL, NULL);
  if (fname == NULL)
    return;

  printf("File name: %s\n", fname);

  char *lname = NULL, *rname = NULL;
  bool ret;
  ret = parse_filename(fname, &lname, &rname, NULL);

  if (!ret)			// just load left
    load_left(fname);
  else
    {
      load_left(lname);
      load_right(rname);
    }
}


// Load a sequence
// Specify either right or left, will try to find the other one
// Names should be like this:  *NNN-L.eee *NNN-R.eee
//                   or this:  left*NNN.eee right*NNN.eee
// Sequence numbers should always have the same number of digits across
//    images, use leading zeros, e.g., image_001-L.bmp

void
cal_load_seq_cb(Fl_Button* b, void* arg)
{
  // get window tab
  int ind = get_current_tab_index();

  char *fname = fl_file_chooser("Load file", NULL, NULL);
  if (fname == NULL)
    return;

  printf("File name: %s\n", fname);

  char *lname = NULL, *rname = NULL;
  int seq;
  bool ret;
  ret = parse_filename(fname, &lname, &rname, &seq);

  if (!ret)			// just load left
    load_left(fname);
  else if (seq < 0)		// stereo only
    {
      load_left(lname);
      load_right(rname);
    }
  else if (rname == NULL)	// mono sequence
    {
      int indx = ind;
      while (ind < MAXIMAGES)
	{
	  char fname[1024];
	  int ret;
	  set_current_tab_index(ind);
	  sprintf(fname,lname,seq);
	  ret = load_left(fname);
	  if (ret < 0)
	    break;
	  ind++;
	  seq++;
	}
      if (indx != ind)
	set_current_tab_index(ind-1); // set to last found images

    }
  else				// stereo sequence
    {
      int indx = ind;
      while (ind < MAXIMAGES)
	{
	  char fname[1024];
	  int ret;
	  set_current_tab_index(ind);
	  sprintf(fname,lname,seq);
	  ret = load_left(fname);
	  if (ret < 0)
	    break;
	  sprintf(fname,rname,seq);
	  ret = load_right(fname);
	  if (ret < 0)
	    break;
	  ind++;
	  seq++;
	}
      if (indx != ind)
	set_current_tab_index(ind-1); // set to last found images
    }
}


//
// delete an image or image pair
//

void cal_delete_image(Fl_Button*, void*)
{
  // get window tab
  int ind = get_current_tab_index();
  calImageWindow *lwin = get_current_left_win();
  calImageWindow *rwin = get_current_right_win();

  if (imgs_right[ind])
    {
      cvReleaseImage(&imgs_right[ind]);
      imgs_right[ind] = NULL;
    }

  rwin->clearAll();

  if (rightcorners[ind])
    {
      delete [] rightcorners[ind];
      rightcorners[ind] = NULL;
    }
  if (rectrightcorners[ind])
    {
      delete [] rectrightcorners[ind];
      rectrightcorners[ind] = NULL;
    }
  nrightcorners[ind] = 0;

  goodright[ind] = false;

  if (imgs_left[ind])
    {
      cvReleaseImage(&imgs_left[ind]);
      imgs_left[ind] = NULL;
    }

  lwin->clearAll();

  if (leftcorners[ind])
    {
      delete [] leftcorners[ind];
      leftcorners[ind] = NULL;
    }
  if (rectleftcorners[ind])
    {
      delete [] rectleftcorners[ind];
      rectleftcorners[ind] = NULL;
    }
  nleftcorners[ind] = 0;

  goodleft[ind] = false;


}



//
// calibrate stereo
// uses Vadim's rewrite of Bouguet's MATLAB code
// gradient descent algorithm
//

void cal_calibrate_single(void);

void cal_calibrate_cb(Fl_Button*, void*)
{
  int i, j, nframes;
  bool is_horz = true;			// true if horizontal epipolar lines
  vector<CvPoint3D32f> objectPoints; // points on the targets
  vector<CvPoint2D32f> points[2]; // points on the image, left is index 0, right 1
  vector<int> npoints;		// number of points in each image
  vector<uchar> active[2];	// active images (found pattern), left is 0, right 1
  vector<CvPoint2D32f> temp(num_pts); // gather image points
  CvSize imageSize = imsize_left; // image size, should be same for left/right
  double R[3][3], E[3][3], F[3][3];
  CvMat _R = cvMat(3, 3, CV_64F, R );
  CvMat _E = cvMat(3, 3, CV_64F, E );
  CvMat _F = cvMat(3, 3, CV_64F, F );

  // find left/right pairs
  int n_left = 0, n_right = 0, n_pair = 0, n_tot = 0;

  for (int i=0; i<MAXIMAGES; i++)
    {
      goodpair[i] = false;
      if (goodleft[i])
	{
	  n_tot++;
	  n_left++;
	  if (goodright[i])
	    {
	      n_pair++;
	      goodpair[i] = true;
	      n_right++;
	    }
	}
      else if (goodright[i])
	{
	  n_tot++;
	  n_right++;
	}
    }

  if (n_pair < 4)
    {
      info_message("Fewer than 4 stereo pairs, calibrating single cameras only");
      cal_calibrate_single();
      return;
    }


  // set up image points from found corners
  for(i=0; i<n_tot; i++)
    {
      // set up active image vectors
      active[0].push_back((uchar)goodleft[i]);
      active[1].push_back((uchar)goodright[i]);

      int N;
      vector<CvPoint2D32f>& pts = points[0]; // left image points
      N = pts.size();
      pts.resize(N + num_pts, cvPoint2D32f(0,0)); // add to array

      int n = N;
      if (goodleft[i])		// have good image points?
	for (int j=0; j<num_pts; j++)
	  pts[n++] = leftcorners[i][j];

      vector<CvPoint2D32f>& pts2 = points[1]; // right image points
      N = pts2.size();
      pts2.resize(N + num_pts, cvPoint2D32f(0,0)); // add to array

      n = N;
      if (goodright[i])		// have good image points?
	for (int j=0; j<num_pts; j++)
	  pts2[n++] = rightcorners[i][j];
    }

    nframes = active[0].size();

    objectPoints.resize(nframes*num_pts);
    for( i = 0; i < num_y_ints; i++ )
        for( j = 0; j < num_x_ints; j++ )
            objectPoints[i*num_x_ints + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + num_pts, objectPoints.begin() + i*num_pts );

    npoints.resize(nframes,num_pts);

    CvMat _objectPoints = cvMat(1, objectPoints.size(), CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, points[0].size(), CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, points[1].size(), CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );

    cvSetIdentity(&K_left);
    cvSetIdentity(&K_right);

    int flags = 0;
    if (is_fixed_aspect)
      flags |= CV_CALIB_FIX_ASPECT_RATIO;
    if (!use_kappa1)
      flags |= CV_CALIB_FIX_K1;
    if (!use_kappa2)
      flags |= CV_CALIB_FIX_K2;
    if (!use_kappa3)
      flags |= CV_CALIB_FIX_K3;
    if (!use_tau)
      flags |= CV_CALIB_ZERO_TANGENT_DIST;
//  flags |= CV_CALIB_ZERO_TANGENT_DIST;

    cvStereoCalibrate( &_objectPoints, &_imagePoints1, &_imagePoints2, &_npoints,
		       &K_left, &D_left, &K_right, &D_right,
		       imageSize, &_R, &Transv, &_E, &_F,
		       cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1, 1e-5),
		       flags);


    // show params
    cvRodrigues2( &_R, &Rotv );
    printf("\nTranslation vector:\n");
    PrintMat(&Transv);

    printf("\nRotation vector:\n");
    PrintMat(&Rotv);

    printf("\nLeft camera matrix:\n");
    PrintMat(&K_left);

    printf("\nLeft distortion vector:\n");
    PrintMat(&D_left);

    printf("\nRight camera matrix:\n");
    PrintMat(&K_right);

    printf("\nRight distortion vector:\n");
    PrintMat(&D_right);

    printf("\n\n");

    // calculate rectification and "new" projection matrices

    double pl[3][4], pr[3][4];
    CvMat P_left   = cvMat(3, 4, CV_64F, pl );
    CvMat P_right  = cvMat(3, 4, CV_64F, pr );
    CvMat *R_left  = cvCreateMat(3,3,CV_64F);
    CvMat *R_right = cvCreateMat(3,3,CV_64F);

    flags = 0;
    if (is_zero_disparity)
      flags |= CV_CALIB_ZERO_DISPARITY;
    cvStereoRectify(&K_left, &K_right, &D_left, &D_right,
		    imageSize, &_R, &Transv,
		    R_left, R_right, &P_left, &P_right, NULL, flags);
    is_horz = fabs(pr[1][3]) <= fabs(pr[0][3]);

    printf("\n\nRectified image parameters\n");
    if (is_horz)
      printf("(Horizontal epilines)\n");
    else
      printf("(Vertical epilines)\n");


    printf("\nLeft camera projection matrix:\n");
    PrintMat(&P_left);

    printf("\nRight camera projection matrix:\n");
    PrintMat(&P_right);

    printf("\nLeft camera rectification matrix:\n");
    PrintMat(R_left);

    printf("\nRight camera rectification matrix:\n");
    PrintMat(R_right);

    printf("\n\n");


    // undistort images

    // Set up rectification mapping
    mapx_left = cvCreateImage(imsize_left, IPL_DEPTH_32F, 1 );
    mapy_left = cvCreateImage(imsize_left, IPL_DEPTH_32F, 1 );

    // rectified K matrix
    CvMat *Krect  = cvCreateMat(3,3,CV_64F);
    CvMat kstub;
    cvCopy(cvGetSubRect(&P_left,&kstub,cvRect(0,0,3,3)),Krect);

    //    cvInitUndistortMap(&K_left, &D_left, mapx_left, mapy_left);
    cvInitUndistortRectifyMap(&K_left, &D_left, R_left, Krect, mapx_left, mapy_left);

    // Save rectified images in array and display
    // Also calculated rectified corner points
    // Should probably enable re-dispaly of original images
    for (int i=0; i<MAXIMAGES; i++)
      {
	if (!goodleft[i]) continue;
	if (rect_left[i] == NULL)
	  rect_left[i] = cvCloneImage(imgs_left[i]);
	cvRemap(imgs_left[i], rect_left[i], mapx_left, mapy_left);
	IplImage *img = rect_left[i];
	set_current_tab_index(i);
	calImageWindow *cwin = get_current_left_win();
	cwin->clear2DFeatures();
	if (nleftcorners[i]>0)
	  {
	    CvMat ipts = cvMat(1,nleftcorners[i],CV_32FC2,leftcorners[i]);
	    CvMat opts = cvMat(1,nleftcorners[i],CV_32FC2,rectleftcorners[i]);
	    cvUndistortPoints(&ipts,&opts,&K_left,&D_left,R_left,&P_left);
	    cwin->display2DFeatures(rectleftcorners[i],nleftcorners[i],1);
	  }
	cwin->DisplayImage((unsigned char *)img->imageData, img->width, img->height, img->width);
      }

    // Set up rectification mapping
    mapx_right = cvCreateImage(imsize_right, IPL_DEPTH_32F, 1 );
    mapy_right = cvCreateImage(imsize_right, IPL_DEPTH_32F, 1 );

    //    cvInitUndistortMap(&K_right, &D_right, mapx_right, mapy_right);
    cvInitUndistortRectifyMap(&K_right, &D_right, R_right, Krect, mapx_right, mapy_right);

    // Save rectified images in array and display
    // Should probably enable re-dispaly of original images
    for (int i=0; i<MAXIMAGES; i++)
      {
	if (!goodright[i]) continue;
	if (rect_right[i] == NULL)
	  rect_right[i] = cvCloneImage(imgs_right[i]);
	cvRemap(imgs_right[i], rect_right[i], mapx_right, mapy_right);
	IplImage *img = rect_right[i];
	set_current_tab_index(i);
	calImageWindow *cwin = get_current_right_win();
	cwin->clear2DFeatures();
	if (nrightcorners[i]>0)
	  {
	    CvMat ipts = cvMat(1,nrightcorners[i],CV_32FC2,rightcorners[i]);
	    CvMat opts = cvMat(1,nrightcorners[i],CV_32FC2,rectrightcorners[i]);
	    cvUndistortPoints(&ipts,&opts,&K_right,&D_right,R_right,&P_right);
	    cwin->display2DFeatures(rectrightcorners[i],nrightcorners[i],1);
	  }
	cwin->DisplayImage((unsigned char *)img->imageData, img->width, img->height, img->width);
      }


    // compute and show epipolar line RMS error
    double err = epi_scanline_error(is_horz);
    printf("RMS error from scanline: %f pixels\n\n", err);
}



// calibrate single camera(s)

void cal_calibrate_single(void)
{
  // find left/right pairs
  int n_left = 0, n_right = 0, n_pair = 0;

  for (int i=0; i<MAXIMAGES; i++)
    {
      goodpair[i] = false;
      if (goodleft[i])
	{
	  n_left++;
	  if (goodright[i])
	    {
	      n_pair++;
	      goodpair[i] = true;
	    }
	}
      if (goodright[i])
	n_right++;
    }


  // calibrate left camera
  if (n_left < 3)		// not enough left images
    {
      printf("Only %d good left images, need 3\n", n_left);
      return;
    }

  // create matrices, total pts x 2 or 3
  // each row is a point
  CvMat *opts_left = cvCreateMat(n_left*num_pts, 3, CV_32FC1);
  CvMat *ipts_left = cvCreateMat(n_left*num_pts, 2, CV_32FC1);
  CvMat *npts_left = cvCreateMat(n_left, 1, CV_32SC1);

  // fill them from corner data
  int ii = 0;
  for (int i=0; i<MAXIMAGES; i++)
    {
      if (!goodleft[i]) continue; // skip over bad images
      int k = ii*num_pts;	// matrix row index
      for (int j=0; j<num_pts; j++, k++)
	{
	  CV_MAT_ELEM(*ipts_left, float, k, 0) = (leftcorners[i])[j].x;
	  CV_MAT_ELEM(*ipts_left, float, k, 1) = (leftcorners[i])[j].y;
	  CV_MAT_ELEM(*opts_left, float, k, 0) = j/num_x_ints; // corner x pos
	  CV_MAT_ELEM(*opts_left, float, k, 1) = j%num_x_ints; // corner y pos
	  CV_MAT_ELEM(*opts_left, float, k, 2) = 0.0f; // corner z pos
	}
      CV_MAT_ELEM(*npts_left, int, ii, 0) = num_pts; // number of points
      ii++;
    }

  // save for debug
  cvSave("ipts_left.xml", ipts_left);
  cvSave("opts_left.xml", opts_left);

  // now call the cal routine
  CvMat* intrinsics_left = cvCreateMat(3,3,CV_32FC1);
  CvMat* distortion_left = cvCreateMat(4,1,CV_32FC1);
  // focal lengths have 1/1 ratio
  CV_MAT_ELEM(*intrinsics_left, float, 0, 0 ) = 1.0f;
  CV_MAT_ELEM(*intrinsics_left, float, 1, 1 ) = 1.0f;
  printf("cvCalibrateCamera2\n");
  cvCalibrateCamera2(opts_left, ipts_left, npts_left,
		     imsize_left, intrinsics_left,
		     distortion_left, NULL, NULL,
		     0);

  // Save for debug
  printf("Saving intrinsics and distortion in <Intrinsics_left.xml>\n  \
and <Distortion_left.xml>");
  cvSave("Intrinsics_left.xml",intrinsics_left);
  cvSave("Distortion_left.xml",distortion_left);

  // Set up rectification mapping
  mapx_left = cvCreateImage(imsize_left, IPL_DEPTH_32F, 1 );
  mapy_left = cvCreateImage(imsize_left, IPL_DEPTH_32F, 1 );
  printf("cvInitUndistortMap\n");
  cvInitUndistortMap(intrinsics_left, distortion_left, mapx_left, mapy_left);

  // Save rectified images in array and display
  // Should probably enable re-dispaly of original images
  for (int i=0; i<MAXIMAGES; i++)
    {
      if (!goodleft[i]) continue;
      if (rect_left[i] == NULL)
	rect_left[i] = cvCloneImage(imgs_left[i]);
      cvRemap(imgs_left[i], rect_left[i], mapx_left, mapy_left);
      IplImage *img = rect_left[i];
      set_current_tab_index(i);
      calImageWindow *cwin = get_current_left_win();
      cwin->clear2DFeatures();
      cwin->DisplayImage((unsigned char *)img->imageData, img->width, img->height, img->width);
    }


  // Calibrate right camera
  if (n_right < 3)		// not enough right images
    {
      printf("Only %d good right images, need 3\n", n_right);
      return;
    }

  // create matrices, total pts x 2 or 3
  // each row is a point
  CvMat *opts_right = cvCreateMat(n_right*num_pts, 3, CV_32FC1);
  CvMat *ipts_right = cvCreateMat(n_right*num_pts, 2, CV_32FC1);
  CvMat *npts_right = cvCreateMat(n_right, 1, CV_32SC1);

  // fill them from corner data
  ii = 0;
  for (int i=0; i<MAXIMAGES; i++)
    {
      if (!goodright[i]) continue;
      int k = ii*num_pts;	// matrix row index
      for (int j=0; j<num_pts; j++, k++)
	{
	  CV_MAT_ELEM(*ipts_right, float, k, 0) = (rightcorners[i])[j].x;
	  CV_MAT_ELEM(*ipts_right, float, k, 1) = (rightcorners[i])[j].y;
	  CV_MAT_ELEM(*opts_right, float, k, 0) = j/num_x_ints; // corner x pos
	  CV_MAT_ELEM(*opts_right, float, k, 1) = j%num_x_ints; // corner y pos
	  CV_MAT_ELEM(*opts_right, float, k, 2) = 0.0f; // corner z pos
	}
      CV_MAT_ELEM(*npts_right, int, ii, 0) = num_pts; // number of points
      ii++;
    }

  // save for debug
  cvSave("ipts.xml", ipts_right);
  cvSave("opts.xml", opts_right);

  // now call the cal routine
  CvMat* intrinsics_right = cvCreateMat(3,3,CV_32FC1);
  CvMat* distortion_right = cvCreateMat(4,1,CV_32FC1);
  // focal lengths have 1/1 ratio
  CV_MAT_ELEM(*intrinsics_right, float, 0, 0 ) = 1.0f;
  CV_MAT_ELEM(*intrinsics_right, float, 1, 1 ) = 1.0f;
  printf("cvCalibrateCamera2\n");
  cvCalibrateCamera2(opts_right, ipts_right, npts_right,
		     imsize_right, intrinsics_right,
		     distortion_right, NULL, NULL,
		     0);

  // Save for debug
  printf("Saving intrinsics and distortion in <Intrinsics_right.xml>\n  \
and <Distortion_right.xml>");
  cvSave("Intrinsics_right.xml",intrinsics_right);
  cvSave("Distortion_right.xml",distortion_right);

  // Set up rectification mapping
  mapx_right = cvCreateImage(imsize_right, IPL_DEPTH_32F, 1 );
  mapy_right = cvCreateImage(imsize_right, IPL_DEPTH_32F, 1 );
  printf("cvInitUndistortMap\n");
  cvInitUndistortMap(intrinsics_right, distortion_right, mapx_right, mapy_right);

  // Save rectified images in array and display
  // Should probably enable re-dispaly of original images
  for (int i=0; i<MAXIMAGES; i++)
    {
      if (!goodright[i]) continue;
      if (rect_right[i] == NULL)
	rect_right[i] = cvCloneImage(imgs_right[i]);
      cvRemap(imgs_right[i], rect_right[i], mapx_right, mapy_right);
      IplImage *img = rect_right[i];
      set_current_tab_index(i);
      calImageWindow *cwin = get_current_right_win();
      cwin->clear2DFeatures();
      cwin->DisplayImage((unsigned char *)img->imageData, img->width, img->height, img->width);
    }

}


//
// epipolar constraint check
// uses the left and right corner arrays
// <horz> is true (default) for horizontal epilines
//

double epi_scanline_error(bool horz)
{
  double sum = 0.0;
  int cnt = 0;
  for (int i=0; i<MAXIMAGES; i++)
    if (goodpair[i])		// have stereo images with corners found?
      {
	for (int j=0; j<nleftcorners[i]; j++)
	  {
	    double dd;
	    if (horz)
	      dd = (rectleftcorners[i][j].y - rectrightcorners[i][j].y);
	    else
	      dd = (rectleftcorners[i][j].x - rectrightcorners[i][j].x);
	    dd = dd*dd;
	    sum += dd;
	    cnt++;
	  }
      }
  if (cnt == 0) return 0.0;
  return sqrt(sum/((double)cnt));
}


//
// calculate epipolar error
// assumes images have already been loaded
// assumes horizontal epilines
//

void cal_epipolar_cb(Fl_Button*, void*)
{
  double err = epi_scanline_error();
  info_message("Epiline RMS error: %f pixels", err);
}



//
// do stereo using new stereolib
// assumes rectified images in left,right windows
//

void do_stereo_cb(Fl_Button*, void*)
{
  uint8_t *lim, *rim, *flim, *frim, *buf;
  int16_t *disp;

  // get window tab
  int ind = get_current_tab_index();
  // get images
  IplImage *limg = imgs_left[ind];
  IplImage *rimg = imgs_right[ind];
  if (limg == NULL || rimg == NULL)
    {
      info_message("No stereo pair at index %d", ind);
      return;
    }
  lim = (uint8_t *)limg->imageData;
  rim = (uint8_t *)rimg->imageData;
  int xim = imsize_left.width;
  int yim = imsize_left.height;

  // some parameters
  int ftzero = 31;		// max 31 cutoff for prefilter value
  int dlen   = 64;		// 64 disparities
  int corr   = 15;		// correlation window size
  int tthresh = 10;		// texture threshold
  int uthresh = 15;		// uniqueness threshold

  // allocate buffers
  buf  = (uint8_t *)malloc(yim*dlen*(corr+5)); // local storage for the algorithm
  flim = (uint8_t *)malloc(xim*yim); // feature image
  frim = (uint8_t *)malloc(xim*yim); // feature image
  disp = (int16_t *)malloc(xim*yim*2); // disparity image

  // prefilter
  double t0 = get_ms();
  do_prefilter(lim, flim, xim, yim, ftzero, buf);
  do_prefilter(rim, frim, xim, yim, ftzero, buf);
  double t1 = get_ms();

  // stereo
  do_stereo(flim, frim, disp, NULL, xim, yim,
	    ftzero, corr, corr, dlen, tthresh, uthresh, buf);
  double t2 = get_ms();

  printf("Prefilter time: %0.1f ms\n", t1-t0);
  printf("Stereo time:    %0.1f ms\n", t2-t1);

  // display disparity image
  calImageWindow *cwin = get_current_right_win();
  cwin->clear2DFeatures();
  cwin->DisplayImage((unsigned char *)disp, xim, yim, xim, DISPARITY, dlen*16);


}



// other callbacks

void cal_save_image_cb(Fl_Button*, void*) {}
void cal_capture_cb(Fl_Button*, void*) {}
void cal_save_all_cb(Fl_Button*, void*) {}

void cal_save_params_cb(Fl_Button*, void*){}
void cal_ok_cb(Fl_Button*, void*){}

void cal_fixed_aspect_cb(Fl_Check_Button *w, void*)
{
  if (w->value())
    is_fixed_aspect = true;
  else
    is_fixed_aspect = false;
}


void cal_fixed_kappa2_cb(Fl_Check_Button *w, void*)
{
  if (w->value())
    use_kappa2 = true;
  else
    use_kappa2 = false;
}

void cal_fixed_kappa3_cb(Fl_Check_Button *w, void*)
{
  if (w->value())
    use_kappa3 = true;
  else
    use_kappa3 = false;
}

void cal_fixed_tau_cb(Fl_Check_Button *w, void*)
{
  if (w->value())
    use_tau = true;
  else
    use_tau = false;
}

void cal_zero_disparity_cb(Fl_Check_Button *w, void*)
{
  if (w->value())
    is_zero_disparity = true;
  else
    is_zero_disparity = false;
}


// callbacks for changing the number of intersections in target
// and the target size

void cal_check_size_cb(Fl_Value_Input *w, void*)
{
  squareSize = w->value()/1000.0; // size in mm
  info_message("Target square size is %d mm", (int)w->value());
}

void cal_check_x_cb(Fl_Value_Input *w, void*)
{
  num_x_ints = (int)w->value();
  info_message("Target X corners: %d", num_x_ints);
  num_pts = num_x_ints*num_y_ints;
}


void cal_check_y_cb(Fl_Value_Input *w, void*)
{
  num_y_ints = (int)w->value();
  info_message("Target Y corners: %d", num_y_ints);
  num_pts = num_x_ints*num_y_ints;
}



// parsing file names

// parses a sequence number, if it exists
// <n> is just after the rightmost number of the putative sequence
// returns the number of chars in the sequence
// also sets <num> to the current value of the sequence
//

int
parse_sequence(char *fname, int n, int *num)
{
  int j = n-1;
  int sum = 0;
  int fact = 1;
  while (j >= 0)
    {
      if (isdigit(fname[j]))	// ok, have a sequence digit
	sum += fact*(fname[j]-'0');
      else
	break;
      j--;
      fact *= 10;
    }

  if (j == n-1)			// didn't find a digit
    return -1;

  *num = sum;
  return n-1-j;			// number of digits
}

// parses a file name to see if it is a sequence or stereo pair
//    or both
// returns TRUE if so
// lname is non-null if mono or stereo image is indicated
// rname is non-null if stereo is indicated
// num is -1 for no sequence, else sequence start number
//

bool
parse_filename(char *fname, char **lname, char **rname, int *num)
{
  if (num)
    *num = -1;			// default
  char *lbase = NULL, *rbase = NULL;
  if (fname == NULL)
    return false;
  int n = strlen(fname);
  if (n < 5)
    return false;

  // find suffix
  int sufn = 0;
  for (int i=n-1; i>n-6; i--)
    {
      if (fname[i] == '.')
	{
	  sufn = i;
	  break;
	}
    }

  if (sufn < 3)
    return false;


  // find start of base file name
  int basen = 0;
  for (int i=sufn-1; i>=0; i--)
    {
      if (fname[i] == '/')
	{
	  basen = i;
	  break;
	}
    }
  if (basen > 0)		// go to first letter of base name
    basen++;



  // check for -L/R type endings
  int stn = sufn-2;		// start of -L/R
  int seq = -1;

  if (strncmp(&fname[stn],"-L",2) == 0)
    {
      // check for sequence
      if (num)
	seq = parse_sequence(fname,stn,num);
      if (seq < 0)		// nope
	{
	  // just return stereo names
	  lbase = new char[n+1];
	  rbase = new char[n+1];
	  strcpy(lbase,fname);
	  strcpy(rbase,fname);
	  rbase[stn+1] = 'R';
	  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
	  *lname = lbase;
	  *rname = rbase;
	  return true;
	}
      else			// yup, add in sequence
	{
	  lbase = new char[n-seq+5];
	  rbase = new char[n-seq+5];
	  strcpy(lbase,fname);
	  lbase[stn-seq] = 0;	// terminate before sequence
	  strcat(lbase,"%0");
	  sprintf(&lbase[stn-seq+2],"%d",seq);
	  strcat(lbase,"d-L");
	  strcat(lbase,&fname[sufn]); // suffix
	  strcpy(rbase,lbase);
	  rbase[stn-seq+4+1] = 'R';
	  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
	  *lname = lbase;
	  *rname = rbase;
	  return true;

	}

    }

  // check for left/right type beginnings
  else if (strncmp(&fname[basen],"left",4) == 0)
    {
      // check for sequence
      if (num)
        seq = parse_sequence(fname,sufn,num);
      if (seq < 0)		// nope
	{
	  // just return stereo names
	  lbase = new char[n+1];
	  rbase = new char[n+2];
	  strcpy(lbase,fname);
	  strcpy(rbase,fname);
	  strcpy(&rbase[basen],"right");
	  strcpy(&rbase[basen+5],&fname[basen+4]);
	  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
	  *lname = lbase;
	  *rname = rbase;
	  return true;
	}
      else			// yup, add in sequence
	{
	  lbase = new char[n-seq+5];
	  rbase = new char[n-seq+6];
	  strcpy(lbase,fname);
	  lbase[sufn-seq] = 0;	// terminate before sequence
	  strcat(lbase,"%0");
	  sprintf(&lbase[sufn-seq+2],"%d",seq);
	  strcat(lbase,"d");
	  strcat(lbase,&fname[sufn]); // suffix
	  strcpy(rbase,lbase);
	  strcpy(&rbase[basen],"right");
	  strcpy(&rbase[basen+5],&lbase[basen+4]);
	  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
	  *lname = lbase;
	  *rname = rbase;
	  return true;

	}

    }

  return false;

}

// utilities

void PrintMat(CvMat *A)
{
  int i, j;
  for (i = 0; i < A->rows; i++)
    {
      switch (CV_MAT_DEPTH(A->type))
	{
	case CV_32F:
	case CV_64F:
	  for (j = 0; j < A->cols; j++)
	    printf ("%8.5f ", (float)cvGetReal2D(A, i, j));
	  break;
	case CV_8U:
	case CV_16U:
	  for(j = 0; j < A->cols; j++)
	    printf ("%6d",(int)cvGetReal2D(A, i, j));
	  break;
	default:
	  break;
	}
      printf("\n");
    }
}

// gets time in ms
double get_ms()
{
  struct timeval t0;
  gettimeofday(&t0,NULL);
  double ret = t0.tv_sec * 1000.0;
  ret += ((double)t0.tv_usec)*0.001;
  return ret;
}

