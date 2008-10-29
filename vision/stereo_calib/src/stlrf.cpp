/*#########################################
 * stlrf.cpp
 *
 * Main functions for stereo/lrf cal GUI
 *
 *#########################################
 */

/**
 ** stlrf.cpp
 **
 ** Kurt Konolige
 ** Senior Researcher
 ** Willow Garage
 ** 68 Willow Road
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@willowgarage.com
 **
 **
 ** Jindong Chen
 ** jdchen@willowgarage.com
 **/


#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>
#include <sys/time.h>
#include "stereolrfgui.h"
#include <opencv/cv.h>
#include <opencv/cxmisc.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
// Willow Garage extension
#include <PoseEstimateDisp.h>
#include <CvPoseEstErrMeasDisp.h>
#include <CvMatUtils.h>
using namespace cv::willow;

#include "cloud_transform.h"

using namespace std;

//
// Stereo cal application
// Read in images
// Calibrate and write out parameters
//

stereolrfgui *stg;			// GUI object

#define MAXIMAGES 20

// image storage, need it for undistortions
IplImage *imgs_left[MAXIMAGES];
IplImage *imgs_right[MAXIMAGES];
IplImage *imgs_lrf[MAXIMAGES];
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
CvPoint2D32f *lrfcorners[MAXIMAGES];
// 3 D points in cartesian space from lrf
CvPoint3D32f *lrf3dcorners[MAXIMAGES];

int nleftcorners[MAXIMAGES];
int nrightcorners[MAXIMAGES];
int nlrfcorners[MAXIMAGES];

// good images
bool goodleft[MAXIMAGES];
bool goodright[MAXIMAGES];
bool goodlrf[MAXIMAGES];
bool goodpair[MAXIMAGES];	// true if features found in stereo pair
bool goodtriple[MAXIMAGES];	// true if features found in stereo + lrf

// image size in pixels, shouldn't change across images
CvSize imsize_left, imsize_right, imsize_lrf;

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
// projection matrix for left camera and right camera
double _PL[3][4], _PR[3][4];
CvMat P_left   = cvMat(3, 4, CV_64F, _PL );
CvMat P_right  = cvMat(3, 4, CV_64F, _PR );

double _RotStlrf[9], _ShiftStlrf[3];
CvMat RotStlrf   = cvMat(3, 3, CV_64F, _RotStlrf);
CvMat ShiftStlrf = cvMat(3, 1, CV_64F, _ShiftStlrf);

// helpers
void set_current_tab_index(int ind);
bool parse_filename(char *fname, char **lbase, char **rbase, char **lrfbase, int *num);
int load_left(char *fname);
int load_right(char *fname);
int load_lrf(char *fname);
laser_scan read_lrf_xml_file(const char *filename);

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

const string fnLeft       = "left";
const string fnRight      = "right";
const string fnLaserRange = "laser";

const int fnLeftLabelSize  = sizeof(fnLeft);
const int fnRightLabelSize = sizeof(fnRight);
const int fnLaserRangeSize = sizeof(fnLaserRange);

static double RansacInlierThreshold = 9.9;

static IplImage *imgErrVectorLeftCam      = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
static IplImage *imgErrorDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);




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
  stg = new stereolrfgui;
  gwin = stg->stereo_lrf_calibration;
  set_current_tab_index(0);
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


calImageWindow *
get_current_lrf_win()
{
  int ind = get_current_tab_index();
  if (ind < 0)
    return NULL;
  switch (ind)
    {
    case 0:
      return stg->calLrf1;
    case 1:
      return stg->calLrf2;
    case 2:
      return stg->calLrf3;
    case 3:
      return stg->calLrf4;
    case 4:
      return stg->calLrf5;
    case 5:
      return stg->calLrf6;
    case 6:
      return stg->calLrf7;
    case 7:
      return stg->calLrf8;
    case 8:
      return stg->calLrf9;
    case 9:
      return stg->calLrf10;
    case 10:
      return stg->calLrf11;
    case 11:
      return stg->calLrf12;
    case 12:
      return stg->calLrf13;
    case 13:
      return stg->calLrf14;
    case 14:
      return stg->calLrf15;
    case 15:
      return stg->calLrf16;
    case 16:
      return stg->calLrf17;
    case 17:
      return stg->calLrf18;
    case 18:
      return stg->calLrf19;
    case 19:
      return stg->calLrf20;
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


void
cal_load_lrf_cb(Fl_Button* b, void* arg)
{
  char *fname = fl_file_chooser("Load file", NULL, NULL);
  if (fname == NULL)
    return;

  printf("File name: %s\n", fname);

  load_lrf(fname);
}

CvPoint3D32f getClosest3DPoint(std::vector<mapped_points>& points, CvPoint2D32f uv){
	float dist = FLT_MAX; // large enough initial dist;
	CvPoint3D32f best3DPoint;
	vector<mapped_points>::const_iterator iter_pts;
	for (iter_pts = points.begin(); iter_pts != points.end(); iter_pts++) {
		float dx = (uv.x - (*iter_pts).points2d.x);
		float dy = (uv.y - (*iter_pts).points2d.y);

		float d = dx*dx + dy*dy;
		if (d < dist) {
			// keep this point
			best3DPoint = (*iter_pts).points3d;
			dist = d;
		}
	}
	return best3DPoint;
}

void get3DPointsOnBoard(CvPoint2D32f *corners, int numCorners, vector<mapped_points>& points,
		vector<CvPoint3D32f>& ptsOnBoard){
	// points 0, num_x_ints-1, (num_y_ints-1)*num_x_ints, and num_x_ints*num_y_ints-1 are the 4 corners
	// for now, get the larger of the u of the two left corners, smaller of the u of the two right corners
	// the lower of the two upper corners, and the higher of the two lower corners,
	float u_left, u_right, v_upper, v_lower;
	u_left = (corners[0].x>corners[(num_y_ints-1)*num_x_ints].x) ? corners[0].x :
		corners[(num_y_ints-1)*num_x_ints].x;
	u_right = (corners[num_x_ints-1].x < corners[num_x_ints*num_y_ints-1].x)?
			corners[num_x_ints-1].x : corners[num_x_ints*num_y_ints-1].x;

	v_upper = (corners[0].y > corners[num_x_ints-1].y) ?
			corners[0].y : corners[num_x_ints-1].y;
	v_lower = (corners[(num_y_ints-1)*num_x_ints].y < corners[num_x_ints*num_y_ints-1].y)?
			corners[(num_y_ints-1)*num_x_ints].y : corners[num_x_ints*num_y_ints-1].y;

	vector<mapped_points>::const_iterator iter_pts;
	for (iter_pts = points.begin(); iter_pts != points.end(); iter_pts++) {
		float u = (*iter_pts).points2d.x;
		float v = (*iter_pts).points2d.y;

		if (u>= u_left && u<= u_right &&
				v >= v_upper && v <= v_lower) {

			ptsOnBoard.push_back((*iter_pts).points3d);
		}
	}
	return;
}

int
load_lrf(char *fname)
{
  // figure out what file to load
  int len = strlen(fname);
  char* dotpos = strrchr(fname, '.');
  char xmlfilename[len+3]; // more than enough for xml file name
  int offset = dotpos-fname+1;
  strncpy(xmlfilename, fname, offset);
  // paste over the right suffix
  // load the xml file of 3d points
  strcpy(&xmlfilename[offset],"xml");
  laser_scan ls = read_lrf_xml_file(xmlfilename);

  // load the dat file for now
//  strcpy(&xmlfilename[offset],"dat");
//  laser_scan ls = read_lrf_dat_file(xmlfilename);

  synthetic_image si;

  if (ls.points) {
//	  si = image_from_point_cloud(ls, 640, 480);
	  si = image_from_point_cloud(ls, 320, 240);
  } else {
	  si.image = NULL;
  }

  IplImage *dbg_corners = 0;

  // get window tab
  int ind = get_current_tab_index();

  IplImage *im=NULL;			// OpenCV image

  calImageWindow *cwin = get_current_lrf_win();

//  im = cvLoadImage(fname);	// load image, could be color

  if (im == NULL) {
	  im = si.image;
	  if (im == NULL) {
		  im = cvLoadImage(fname);
		  if (im == NULL) {
			  cerr << "cannot open file: "<<fname;
			  return -1;
		  }
	  }
  }

  {
	  // make grayscale, display
	  IplImage* img=cvCreateImage(cvGetSize(im),IPL_DEPTH_8U,1);
	  dbg_corners = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3 );
	  cvConvertImage(im,img);
	  imgs_lrf[ind] = img;
	  imsize_lrf = cvGetSize(im);
	  info_message("Size: %d x %d, ch: %d", img->width, img->height, img->nChannels);
	  cwin->DisplayImage((unsigned char *)img->imageData, img->width, img->height, img->width);

	  // find corners
	  if (lrfcorners[ind])
		  delete [] lrfcorners[ind];
	  lrfcorners[ind] = new CvPoint2D32f[num_x_ints*num_y_ints];
	  if (lrf3dcorners[ind])
		  delete [] lrf3dcorners[ind];
	  lrf3dcorners[ind] = new CvPoint3D32f[num_x_ints*num_y_ints];
	  int numc = 0;
	  int ret = cvFindChessboardCorners(img, cvSize(num_x_ints, num_y_ints),
			  lrfcorners[ind], &numc, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
	  nlrfcorners[ind] = numc;
	  info_message("Found %d corners", numc);
	  cwin->display2DFeatures(lrfcorners[ind],numc,ret);

	  // do subpixel calculation, if corners have been found
	  if (ret)
	  {
		  cvFindCornerSubPix(img, lrfcorners[ind], numc,
				  cvSize(5,5),cvSize(-1,-1),
				  cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

		  // fill out the corners in 3d
		  cout << "saving uv and xyz of the corners"<<endl;
		  for (int ci=0; ci < numc; ci++) {
			  CvPoint2D32f uv = lrfcorners[ind][ci];

			  CvPoint3D32f xyz = getClosest3DPoint(si.points, uv);
			  lrf3dcorners[ind][ci].x = xyz.x;
			  lrf3dcorners[ind][ci].y = xyz.y;
			  lrf3dcorners[ind][ci].z = xyz.z;
#if 0
			  printf("%12.5f %12.5f %12.5f, %12.5f, %12.5f\n", uv.x, uv.y, lrf3dcorners[ind][ci].x, lrf3dcorners[ind][ci].y, lrf3dcorners[ind][ci].z);
#endif
		  }

		  //
		  // get all the 3d points that we know for sure are on the checkerboard
		  //
		  vector<CvPoint3D32f> ptsOnBoard;
		  get3DPointsOnBoard(lrfcorners[ind], numc, si.points, ptsOnBoard);
		  cout << "Found "<<ptsOnBoard.size() << " 3d points on board"<<endl;
		  CvPoint3D32f _ptsOnBoard[ptsOnBoard.size()];
		  vector<CvPoint3D32f>::const_iterator iter;
		  int k=0;
		  for (iter = ptsOnBoard.begin(); iter != ptsOnBoard.end(); iter++) {
			  _ptsOnBoard[k++] = (*iter);
		  }
		  CvMat pointsOnBoard = cvMat(ptsOnBoard.size(), 1, CV_32FC3, _ptsOnBoard);

		  char pointsOnBoardFilename[len+7];
		  strncpy(pointsOnBoardFilename, fname, offset-1);
		  strncpy(&pointsOnBoardFilename[offset-1], "-Pts", 4);
		  // paste over the right suffix
		  // load the xml file of 3d points
		  strcpy(&pointsOnBoardFilename[offset-1+4],".xml");

		  cvSave(pointsOnBoardFilename, &pointsOnBoard);


	  }

	  // set good/bad image
	  goodlrf[ind] = ret;

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

  char *lname = NULL, *rname = NULL, *lrfname = NULL;
  bool ret;
  ret = parse_filename(fname, &lname, &rname, &lrfname, NULL);

  if (!ret)			// just load left
    load_left(fname);
  else
    {
      load_left(lname);
      load_right(rname);
      load_lrf(lrfname);
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

  char *lname = NULL, *rname = NULL, *lrfname = NULL;
  int seq;
  bool ret;
  ret = parse_filename(fname, &lname, &rname, &lrfname, &seq);

  if (!ret)			// just load left
    load_left(fname);
  else if (seq < 0)		// stereo only
    {
      load_left(lname);
      load_right(rname);
      load_lrf(lrfname);
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
	  sprintf(fname, lrfname, seq);
	  ret = load_lrf(fname);
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
  calImageWindow *lrfwin = get_current_lrf_win();

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

  if (imgs_lrf[ind]) {
	  cvReleaseImage(&imgs_lrf[ind]);
	  imgs_lrf[ind] = NULL;
  }
  lrfwin->clearAll();

  if (lrfcorners[ind]) {
	  delete [] lrfcorners[ind];
	  lrfcorners[ind] = NULL;
  }
  nlrfcorners[ind] = 0;
  goodlrf[ind] = false;
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

  CvMat *R_left  = cvCreateMat(3,3,CV_64F);
  CvMat *R_right = cvCreateMat(3,3,CV_64F);

  flags = 0;
  if (is_zero_disparity)
	  flags |= CV_CALIB_ZERO_DISPARITY;
  cvStereoRectify(&K_left, &K_right, &D_left, &D_right,
		  imageSize, &_R, &Transv,
		  R_left, R_right, &P_left, &P_right, NULL, flags);
  is_horz = fabs(_PR[1][3]) <= fabs(_PR[0][3]);

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
{cout << "Not implemented yet"<<endl;}



// other callbacks

void cal_save_image_cb(Fl_Button*, void*) {cout << "Not implemented yet"<<endl;}
void cal_capture_cb(Fl_Button*, void*) {cout << "Not implemented yet"<<endl;}
void cal_save_all_cb(Fl_Button*, void*) {cout << "Not implemented yet"<<endl;}

void cal_save_params_cb(Fl_Button*, void*){cout << "Not implemented yet"<<endl;}
void cal_ok_cb(Fl_Button*, void*){cout << "Not implemented yet"<<endl;}

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

void cal_ransac_threshold_cb(Fl_Value_Input *w, void*)
{
  RansacInlierThreshold = (double)w->value()/10.0; // size in mm
  info_message("RANSAC threshold in pixel is %f pixels", RansacInlierThreshold);
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
parse_filename(char *fname, char **lname, char **rname, char **lrfname, int *num)
{
  if (num)
    *num = -1;			// default
  char *lbase = NULL, *rbase = NULL;
  char *lrfbase = NULL;
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

  if (strncmp(&fname[stn],"-L",2) == 0 ||
	  strncmp(&fname[stn],"-R",2) == 0 ||
	  strncmp(&fname[stn],"-J",2) == 0)
  {
	  // check for sequence
	  if (num)
		  seq = parse_sequence(fname,stn,num);
	  if (seq < 0)		// nope
	  {
		  // just return stereo names
		  lbase = new char[n+1];
		  rbase = new char[n+1];
		  lrfbase = new char[n+1];
		  strcpy(lbase,fname);
		  strcpy(rbase,fname);
		  strcpy(lrfbase, fname);
		  lbase[stn+1] = 'L';
		  rbase[stn+1] = 'R';
		  lrfbase[stn+1] = 'J';
		  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
		  printf("lrfbase is [%s]\n", lrfbase);
		  *lname = lbase;
		  *rname = rbase;
		  *lrfname = lrfbase;
		  return true;
	  }
	  else			// yup, add in sequence
	  {
		  lbase   = new char[n-seq+5];
		  rbase   = new char[n-seq+5];
		  lrfbase = new char[n-seq+5];
		  strcpy(lbase,fname);
		  lbase[stn-seq] = 0;	// terminate before sequence
		  strcat(lbase,"%0");
		  sprintf(&lbase[stn-seq+2],"%d",seq);
		  strcat(lbase,"d-L");
		  strcat(lbase,&fname[sufn]); // suffix
		  strcpy(rbase,lbase);
		  strcpy(lrfbase, lbase);
		  lbase[stn-seq+4+1] = 'L';
		  rbase[stn-seq+4+1] = 'R';
		  lrfbase[stn-seq+4+1] = 'J';
		  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
		  printf("lrfbase is [%s]\n", lrfbase);
		  *lname = lbase;
		  *rname = rbase;
		  *lrfname = lrfbase;
		  return true;

	  }

  }

  // check for left/right type beginnings
  else if (strncmp(&fname[basen],"left",4) == 0 || strncmp(&fname[basen],"right",5) == 0
		  || strncmp(&fname[basen],"laser",5) == 0)
    {
	  int basen2 = (strncmp(&fname[basen],"left",4) == 0 )?(basen+4) : (basen+5);
      // check for sequence
      if (num)
        seq = parse_sequence(fname,sufn,num);
      if (seq < 0)		// nope
	{
	  // just return stereo names
	  lbase   = new char[n+1];
	  rbase   = new char[n+2];
	  lrfbase = new char[n+2];
	  strcpy(lbase,fname);
	  strcpy(rbase,fname);
	  strcpy(lrfbase, fname);
	  strcpy(&lbase[basen], "left");
	  strcpy(&rbase[basen], "right");
	  strcpy(&lrfbase[basen], "laser");
	  strcpy(&lbase[basen+4],&fname[basen2]);
	  strcpy(&rbase[basen+5],&fname[basen2]);
	  strcpy(&lrfbase[basen+5],&fname[basen2]);
	  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
	  printf("lrfbase is [%s]\n", lrfbase);
	  *lname   = lbase;
	  *rname   = rbase;
	  *lrfname = lrfbase;
	  return true;
	}
      else			// yup, add in sequence
	{
	  lbase = new char[n-seq+5];
	  rbase = new char[n-seq+6];
	  lrfbase = new char[n-seq+6];
	  strcpy(lbase,fname);
	  lbase[sufn-seq] = 0;	// terminate before sequence
	  strcat(lbase,"%0");
	  sprintf(&lbase[sufn-seq+2],"%d",seq);
	  strcat(lbase,"d");
	  strcat(lbase,&fname[sufn]); // suffix
	  strcpy(rbase,lbase);
	  strcpy(&lbase[basen],"left");
	  strcpy(&rbase[basen],"right");
	  strcpy(&lrfbase[basen],"laser");
	  strcpy(&lbase[basen+4],&lbase[basen2]);
	  strcpy(&rbase[basen+5],&lbase[basen2]);
	  strcpy(&rbase[basen+5],&lbase[basen2]);
	  printf("lbase is [%s]\nrbase is [%s]\n", lbase, rbase);
	  printf("lrfbase is [%s]\n", lrfbase);
	  *lname = lbase;
	  *rname = rbase;
	  *lrfname = lrfbase;
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

bool cal_poseEstimation() {
	int status = false;
	vector<uchar> active[3];	// active images (found pattern), left is 0, right 1, laser 2
	CvMat uvds1;
	CvMat xyzs0;
	CvMat xyzs0z;

	// compute the disparity coordinates of the corners from the stereo pair.
	// find good triple (left/right/laser)
	int n_lrf = 0, n_triple = 0, n_tot = 0;
	int n_points = 0;
	for (int i=0; i<MAXIMAGES; i++)
	{
		goodtriple[i] = false;
		if (goodleft[i] && goodright[i])
		{
			n_tot++;
			if (goodlrf[i])
			{
				goodtriple[i] = true;
				n_triple++;
				n_lrf++;
				n_points += num_pts;
			}
		}
		else if (goodlrf[i])
		{
			n_tot++;
			n_lrf++;
		}
	}
	//TODO: do we need to check how many good triples there are
	if (n_triple < 4) {
		info_message("Warning: Fewer than 4 stereo+laser triples. Continue for now...");
	}

	double _uvds1[n_points*3];
	double _xyzs0[n_points*3];
	double _xyzs0z[n_points*3];

	// set up image points from found corners
	int n = 0;
	for(int i=0; i<n_tot; i++)	  {
		// set up active image vectors
		active[0].push_back((uchar)goodleft[i]);
		active[1].push_back((uchar)goodright[i]);
		active[2].push_back((uchar)goodlrf[i]);

		if (goodtriple[i]) { // have good image points?
			for (int j=0; j<num_pts; j++) {
#if 0 // the input images from stereo camera are already rectified
				_uvds1[n*3 + 0] = rectleftcorners[i][j].x;
				_uvds1[n*3 + 1] = rectleftcorners[i][j].y;
				_uvds1[n*3 + 2] = rectleftcorners[i][j].x - rectrightcorners[i][j].x;
#else
				_uvds1[n*3 + 0] = leftcorners[i][j].x;
				_uvds1[n*3 + 1] = leftcorners[i][j].y;
				_uvds1[n*3 + 2] = leftcorners[i][j].x - rightcorners[i][j].x;
#endif

				_xyzs0[n*3 + 0] = lrf3dcorners[i][j].x;
				_xyzs0[n*3 + 1] = lrf3dcorners[i][j].y;
				_xyzs0[n*3 + 2] = lrf3dcorners[i][j].z;

				n++;
			}
		}
		assert(n % num_pts == 0);
	}
	assert(n == n_triple*num_pts);

	// a rotation matrix to turn from x forward, z up to
	// z forward and y down
#if 0
	double _X2Z[9] = {
			0, -1,  0,
			0,  0, -1,
			1,  0,  0
	};
#else
	double _X2Z[9] = {
			1,  0,  0,
			0,  1,  0,
			0,  0,  1
	};
#endif
	CvMat X2Z = cvMat(3, 3, CV_64F, _X2Z);
	cvInitMatHeader(&xyzs0,  n, 3, CV_64FC1, _xyzs0);
	cvInitMatHeader(&xyzs0z, n, 3, CV_64FC1, _xyzs0z);
	cvInitMatHeader(&uvds1,  n, 3, CV_64FC1, _uvds1);

	cvGEMM(&xyzs0, &X2Z, 1.0, NULL, 0, &xyzs0z, CV_GEMM_B_T);

	// scale xyzs1z up from M to mm
	cvScale(&xyzs0z, &xyzs0z, 1000.0);

	// TODO: jdc experimenting correcting bias in z (depth)
#if 0
	CvMat xyzs0z_c3;
	cvReshape(&xyzs0z, &xyzs0z_c3, 3, 0);
	cout << "Before: "<< _xyzs0z[2]<<endl;
	cvAddS(&xyzs0z_c3, cvScalar(0, 0, -11.23), &xyzs0z_c3);
	cout << "After: "<< _xyzs0z[2]<<endl;
#endif

	PoseEstimateDisp peDisp;
#if 0
	double Fx = cvmGet(&P_left, 0, 0);
	double Fy = cvmGet(&P_left, 1, 1);
	double Tx = - 1000.*cvmGet(&P_right, 0, 3)/Fx;
	double Clx = cvmGet(&P_left, 0, 2);
	double Crx = cvmGet(&P_right, 0, 2);
	double Cy  = cvmGet(&P_left, 1, 2);
#else
	// TODO: shall read from file
	// copy from calibration file
	double Fx = 729.;
	double Fy = 729.;
	double Tx =  43812.14/Fx;
	double Clx = 323.9809;
	double Crx = 323.9809;
	double Cy  = 247.874;
#endif
	peDisp.setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);

	peDisp.setInlierErrorThreshold(RansacInlierThreshold);

	cout << "inlier errorthreshold set to : "<< RansacInlierThreshold << endl;

	CvMat *inliers0, *inliers1;

//	cvSave("xyzs.xml", &xyzs0z);
//	cvSave("uvds.xml", &uvds1);

	int numInliers = peDisp.estimateMixedPointClouds(&xyzs0z, &uvds1, 0, NULL,
	    &RotStlrf, &ShiftStlrf, true);

	if (numInliers>0) peDisp.getInliers(inliers0, inliers1);


	// now lets transform the rotation and shift matrix into ROS/verdere
	double _XRightYDownToXForwardYLeft[] =
	{
			 0.,  0.,  1.,
			-1.,  0.,  0.,
			 0., -1.,  0.,
	};
	CvMat XRightYDownToXForwardYLeft = cvMat(3, 3, CV_64F, _XRightYDownToXForwardYLeft);
	double _Rvidere[9], _Tvidere[3];
	CvMat  Rvidere = cvMat(3, 3, CV_64F, _Rvidere);
	CvMat  Tvidere = cvMat(3, 1, CV_64F, _Tvidere);
	// Rvidere = XRightYDownToXForwardYLeft * RotStlrf
	cvMatMul(&XRightYDownToXForwardYLeft, &RotStlrf, &Rvidere);

	// Tvidere = XRightYDownToXForwardYLeft * ShiftStlrf
	cvMatMul(&XRightYDownToXForwardYLeft, &ShiftStlrf, &Tvidere);

	cout << "Number of inliers in pose estimation: "<<numInliers<<" from "<<n<<" input points"<<endl;

	cout << "Rotation Matrix between laser range and left stereo camera (in ROS Frame):"<<endl;
	PrintMat(&Rvidere);
	cout << "Rotation Matrix between laser range and left stereo camera (in Small V frame):"<<endl;
	PrintMat(&RotStlrf);
	cout << "Shfit (translation) Matrix between laser range and left stereo camera (in ROS Frame):"<<endl;
	PrintMat(&Tvidere);
	cout << "Shfit (translation) Matrix between laser range and left stereo camera (in Small V Frame):"<<endl;
	PrintMat(&ShiftStlrf);

	double _rod[3], _dir[3];;
	CvMat rod = cvMat(3, 1, CV_64F, _rod);
	CvMat dir = cvMat(3, 1, CV_64F, _dir);
	cvRodrigues2(&RotStlrf, &rod);
	double norm = cvNorm(&rod);
	cvNormalize(&rod, &dir);
	cout << "Rodrigues, dir: "<<endl;
	CvMatUtils::printMat(&dir);
	cout << "Rodrigues rot angle in degree: "<< norm/CV_PI*180. <<endl;

	CvPoseEstErrMeasDisp peErrMeas;
	peErrMeas.setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);
	peErrMeas.setTransform(RotStlrf, ShiftStlrf);
	peErrMeas.measureMixed(xyzs0z, uvds1);

	printf("Waiting key...\n");
	cvWaitKey(0);
	return status;
}

void cal_calibrate_stlrf_cb(Fl_Button*, void*) {
        cout << "Perform stereo + lrf calibration"<<endl;
	cal_poseEstimation();
}

laser_scan read_lrf_xml_file(const char *filename)
{
	CvMat* lrf = (CvMat*) cvLoad(filename);
	if (lrf == NULL) {
		cerr << "Cannot open xml file: "<<filename<<endl;
    	laser_scan ls;
    	ls.points = NULL;
    	ls.intensity = NULL;
    	return ls;
    }
	int numPts = lrf->rows;
    string s;
    vector<CvPoint3D32f> *points    = new vector<CvPoint3D32f>(numPts);
    vector<double>       *intensities = new vector<double>(numPts);

    for (int i=0;i<numPts; i++) {
    	double x, y, z, intensity;
    	x = cvmGet(lrf, i, 0);
    	y = cvmGet(lrf, i, 1);
    	z = cvmGet(lrf, i, 2);
    	intensity = cvmGet(lrf, i, 3);
    	points->push_back(cvPoint3D32f(x, y, z));
    	intensities->push_back(intensity);
    }

    laser_scan ret;
    ret.points = points;
    ret.intensity = intensities;
    return ret;
}

