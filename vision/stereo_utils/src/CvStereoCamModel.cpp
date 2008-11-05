#include "CvStereoCamModel.h"

#include <float.h>
#include <iostream>
using namespace std;

#include <opencv/cxtypes.h>
#include <opencv/cxcore.h>
// #include "CvMatUtils.h"
#include <vector>     //Gary added for collecting valid x,y,d points

#define DISPLAY 1
#define VERBOSE 1

#define cvFuncName __PRETTY_FUNCTION__

static const IplConvKernel* DilateKernel = cvCreateStructuringElementEx(15, 15, 7, 7, CV_SHAPE_RECT);
static const IplConvKernel* OpenKernel   = cvCreateStructuringElementEx(3, 3, 1, 1,   CV_SHAPE_RECT);
static const IplConvKernel* CloseKernel  = cvCreateStructuringElementEx(7, 7, 3, 3,   CV_SHAPE_RECT);

#define IMG_DEPTH uchar

static const int OCC_EMPTY  = 0;
static const int OCC_FILLED = 1;

void CvStereoCamModel::init() {
  // lucky that we can pull off the member initialization here
  // w/o delegating constructor - no real class member here.
  parameterized_        = false;
  mat_cart_to_screen_left_  = cvMat(3, 4, CV_64FC1, matdata_cart_to_screen_left_);
  mat_cart_to_screen_right_ = cvMat(3, 4, CV_64FC1, matdata_cart_to_screen_right_);
  mat_cart_to_disp_        = cvMat(4, 4, CV_64FC1, matdata_cart_to_disp_);
  mat_disp_to_cart_        = cvMat(4, 4, CV_64FC1, matdata_disp_to_cart_);
  Iz8U_                  = NULL;
  mem_storage_           = NULL;
}

CvStereoCamModel::CvStereoCamModel()
{
  init();
}

CvStereoCamModel::CvStereoCamModel(double Fx, double Fy, double Tx,
      double Clx, double Crx, double Cy, double dispUnitScale)
{
  init();
  setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy, dispUnitScale);
}

CvStereoCamModel::CvStereoCamModel(const CvStereoCamModel& camModel)
{
  init();
  setCameraParams(camModel);
}

CvStereoCamModel::~CvStereoCamModel()
{
	if(Iz8U_) {
		cvReleaseImage(&Iz8U_);
		cvDestroyWindow("Depth");
	}
}

void CvStereoCamModel::constructMat3DToScreen(double Fx, double Fy, double Tx,
    double Cx, double Cy, CvMat& mat){
  // Note that this matrix projects 3d points to screen coordinates. Hence
  // it has nothing to do with disparity and do not need to worry about
  // unit scaling factor in disparity maps
  double data [] = {
      Fx, 0, Cx, -Fx*Tx,
      0, Fy, Cy, 0,
      0,  0,  1, 0
  };

  CvMat _P = cvMat(3, 4, CV_64FC1, data);
  cvCopy(&_P, &mat);
}

void CvStereoCamModel::constructProjectionMatrices(){
  constructMat3DToScreen(Fx_, Fy_, Clx_, Cy_,   0, mat_cart_to_screen_left_);
  constructMat3DToScreen(Fx_, Fy_, Crx_, Cy_, Tx_, mat_cart_to_screen_right_);


  // construct the matrix that maps from Cartesian coordinates to
  // disparity coordinates
  double data [] = {
      Fx_,   0,    Clx_,              0,
      0,     Fy_,  Cy_,               0,
      0,     0,    (Clx_-Crx_)/Du_,   Fx_*Tx_/Du_,
      0,     0,    1,                 0
  };
  CvMat _P = cvMat(4, 4, CV_64FC1, data);
  cvCopy(&_P, &mat_cart_to_disp_);

 	// construct the matrix that maps from disparity coordinates to
	// Cartesian coordinates
#if 1
    // the following is computationally cleaner and may introduce less error
	double Q[] =
	{
		1,         0,          0,               -Clx_,
		0,   Fx_/Fy_,          0,        -Cy_*Fx_/Fy_,
		0,         0,          0,                 Fx_,
		0,         0,      Du_/Tx_,     -(Clx_-Crx_)/Tx_
	};
#else
	// the following form is more symmetrical, and goes along with
	// mCartToDisp
	double Q[] =
	{
		1./Fx_,    0,          0,           -Clx_/Fx_,
		0,   	1./Fy_,        0,           -Cy_/Fy_,
		0,         0,          0,                1,
		0,         0,      Du_./(Tx_*Fx_),    -(Clx_-Crx_)/(Fx_*Tx_)
	};
#endif

	CvMat _Q = cvMat(4, 4, CV_64F, Q);
	cvCopy(&_Q, &mat_disp_to_cart_);
}

void CvStereoCamModel::setCameraParams(double Fx, double Fy, double Tx,
    double Clx, double Crx, double Cy, double dispUnitScale){
  __BEGIN__

  // sanity check
  if (Fx <= 0 || Fy <= 0 || Tx <= 0 || Clx <= 0 || Crx <= 0 || Cy <= 0 ||
      dispUnitScale <= 0 ) {
    CV_ERROR( CV_StsBadArg,
        "At least one of the arguments  is invalid (shall be all greater than zero)" );
  }

  Fx_  = Fx;
  Fy_  = Fy;
  Clx_ = Clx;
  Crx_ = Crx;
  Cy_  = Cy;
  Tx_  = Tx;
  Du_  = dispUnitScale;

  this->constructProjectionMatrices();

  parameterized_ = true;
  __END__
}

void CvStereoCamModel::setCameraParams(const CvStereoCamModel& cm) {
  setCameraParams(cm.Fx_, cm.Fy_, cm.Tx_, cm.Clx_, cm.Crx_, cm.Cy_, cm.Du_);
}

void CvStereoCamModel::getParams(double* Fx, double* Fy, double* Tx, double* Clx, double* Crx,
    double* Cy, double* dispUnitScale) const {
  if (Fx)  *Fx  = Fx_;
  if (Fy)  *Fy  = Fy_;
  if (Tx)  *Tx  = Tx_;
  if (Clx) *Clx = Clx_;
  if (Crx) *Crx = Crx_;
  if (Cy)  *Cy  = Cy_;
  if (dispUnitScale) *dispUnitScale = Du_;
}


void CvStereoCamModel::dispToCart(const CvMat* uvds, CvMat * XYZs) const {
  __BEGIN__

  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  if (uvds == NULL || XYZs == NULL) {
    CV_ERROR( CV_StsBadArg, "neither argument uvds nor XYZs shall be NULL");
  }

  CvMat uvds0;
  CvMat XYZs0;

	cvReshape(uvds, &uvds0, 3, 0);
	cvReshape(XYZs, &XYZs0, 3, 0);
	cvPerspectiveTransform(&uvds0, &XYZs0, &mat_disp_to_cart_);

	__END__
}

//Id has to be  16SC1, Ixyz has to be 32F
void CvStereoCamModel::dispToCart(const IplImage *Id, IplImage *Ixyz) const {
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
	cvReprojectImageTo3D(Id, Ixyz,&mat_disp_to_cart_);
  __END__
}

//Id accepts ROI which then applies to the other images
void CvStereoCamModel::disp8UToCart32F(const IplImage *Id, float ZnearMM,
    float ZfarMM, IplImage *Iz, IplImage *Ix, IplImage *Iy) const {
  __BEGIN__
  //CHECK AND SET UP
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  } else if((!Id)||(!Iz)) {
	  CV_ERROR(CV_StsBadArg, "Id or Iz is NULL");
	} else {
	  int w = Id->width, h = Id->height;
	  if((w != Iz->width)||(h != Iz->height)) {
	    //Image dimensions have to match oi vey, haven't checked Ix, Iy
	    CV_ERROR(CV_StsBadArg, "Image dimensions have to match");
	  } else {
	    if((Id->nChannels != 1)||(Iz->nChannels != 1)) {
	      //All one channel
	      CV_ERROR(CV_StsBadArg, "Images shall all be one channel");
	    } else {
	      if(ZnearMM < 0.0) ZnearMM = 0.0; //Adjusting for illegal values of depth to search in MM
	      if(ZfarMM < ZnearMM) ZfarMM = ZnearMM;

	      int dFar  = (int)(getDisparity(ZfarMM)  + .5);
	      int dNear = (int)(getDisparity(ZnearMM) + .5);

	      CvRect roi = cvGetImageROI(Id);
	      int ws = Id->widthStep, roiw = roi.width, roih = roi.height, roix = roi.x, roiy = roi.y;
	      int Id_jumpby = ws - roiw;                      //This is to step the pointer over possible ragged withSteps
	      unsigned char *Idptr = (unsigned char *)(Id->imageData + roiy*ws + roix);
	      unsigned char disp8U;
	      int goodpoints = 0;

	      //FILL POINTS
	      vector<double> pts;  //Collect good points here
	      vector<int> xy;      //Collect original x,y's here
	      pts.reserve(roiw*roih*3); //Allocate all the space we'll need in one shot
	      xy.reserve(roiw*roih*3);
	      int x,y,i;
	      for(y=roiy; y<roih+roiy; ++y) {
	        for(x=roix; x<roiw+roix; ++x) {
	          disp8U = *Idptr++;
	          if((dNear > disp8U) && (disp8U > dFar)) { //NOTE: Zero is just not allowed. Don't go there, Greek chaos.
	            pts.push_back((double)x);
	            pts.push_back((double)y);
	            pts.push_back((double)disp8U);
	            xy.push_back(x);
	            xy.push_back(y);
	            ++goodpoints;
	          }
	        }
	        Idptr += Id_jumpby;                                    //Step over unused widthStep if any (mostly this is zero)
	      }
	      cvSetZero(Iz); //Start with a clean image
	      if(goodpoints) {
	        //SET UP FOR PERSPECTIVE TRANSFORM
	        CvMat d2c; //Disparity to Cart header holder
	        cvInitMatHeader(&d2c, goodpoints, 1, CV_64FC3, &pts[0]); //We now have a CvMat "view" into vector pts

	        //DO PERSPECTIVE TRANSFORM
	        cvPerspectiveTransform(&d2c, &d2c, &mat_cart_to_disp_);  //Note that cvPerspectiveTransform can be done in place!

	        //PUT THE POINTS OUT IN THE X,Y & Z IMAGES
	        float *Izptr = (float *)Iz->imageData, *Ixptr = (float *)Ix->imageData, *Iyptr = (float *)Iy->imageData; //pt to image data
	        vector<double>::iterator ptspos; ptspos = pts.begin(); //Set up iterators to our data
	        vector<int>::iterator xypos; xypos = xy.begin();
	        int offset;
	        for(i = 0; i< goodpoints; ++i) {
	          x = *xypos; ++xypos;  //x in grid
	          y = *xypos; ++xypos;  //y in grid
	          offset = y*w + x;     //pointer offset
	          *(Ixptr + offset) = (float)*ptspos; ++ptspos; //X in camera cords
	          *(Iyptr + offset) = (float)*ptspos; ++ptspos; //Y in camera cords
	          *(Izptr + offset) = (float)*ptspos; ++ptspos; //Z in camera cords
	        }
	      }
	    }
	  }
	}
	//Done
	__END__
	return;
}

void CvStereoCamModel::cartToDisp(const CvMat* XYZs, CvMat* uvds) const {
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  if (uvds == NULL || XYZs == NULL) {
    CV_ERROR( CV_StsBadArg, "neither argument uvds nor XYZs shall be NULL");
  }
	CvMat xyzs0;
	CvMat uvds0;
	cvReshape(XYZs, &xyzs0, 3, 0);
	cvReshape(uvds, &uvds0, 3, 0);
	cvPerspectiveTransform(&xyzs0, &uvds0, &mat_cart_to_disp_);
	__END__
}

void CvStereoCamModel::cartToLeftCam(const CvMat* XYZs, CvMat* uvs) const {
  // this is not yet an efficient implementation. Will be replaced by Vadim's
  // new code for 3d to 2d perspective transformation.
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  if (uvs == NULL || XYZs == NULL) {
    CV_ERROR( CV_StsBadArg, "neither argument uvds nor XYZs shall be NULL");
  }
  {
    int numPoints = XYZs->rows;
    CvMat xyzs0;
    cvReshape(XYZs, &xyzs0, 3, 0);
    CvMat* uvdsC3 = cvCreateMat(numPoints, 1, CV_MAT_TYPE(xyzs0.type));
    CvMat uvdsC1;

    double* uvds_data = new double[3*numPoints];


    cvPerspectiveTransform(&xyzs0, uvdsC3, &mat_cart_to_disp_);

    // copy channels 1, 2 to uvs
    // reshape the matrix to be 1 channel, Nx3
    cvReshape(uvdsC3, &uvdsC1, 1, numPoints);
    CvMat uvsC1;
    cvGetCols(&uvdsC1, &uvsC1, 0, 2);
    CvMat uvsC1a;
    cvReshape(uvs, &uvsC1a, 1, numPoints);
    cvCopy(&uvsC1, &uvsC1a);
    cvReleaseMat(&uvdsC3);
  }

  __END__
}

static inline double magnitude(CvPoint3D32f& p)
{
  return std::pow(std::pow((double)p.x, 2.0) + std::pow((double)p.y, 2.0) + std::pow((double)p.z, 2.0), 0.5);
}

static inline bool in_bounds(CvPoint& pt, int image_width, int image_height)
{
   if ((0 <= pt.x) && (pt.x < image_width) && (0 <= pt.y) && (pt.y < image_height))
     return true;
   else
     return false;
}

static inline uint int2uchar(uint n)
{
    return (uint) lrint(((double) n) / 25.0);
}

bool fill_missing_pixel(int x, int y, IplImage *original_d16_c1, IplImage *occupancy_d8_c1)
{
    int image_width  = original_d16_c1->width;
    int image_height = original_d16_c1->height;

    //Search for neighbors
    vector<CvPoint> neighbors;

    //top
    int xt = x;
    int yt = max(0, min(image_height, y+1));
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));

    //bottom
    xt = x;
    yt = max(0, min(image_height, y-1));
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));

    //left
    xt = max(0, min(image_width, x+1));
    yt = y;
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));


    //right
    xt = max(0, min(image_width, x-1));
    yt = y;
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));

    //Check if we can use edge sensing
    bool         left_neighbor,  right_neighbor,  top_neighbor,  bottom_neighbor;
    float        left_v,         right_v,         top_v,         bottom_v;
    CvPoint      left_neighborp, right_neighborp, top_neighborp, bottom_neighborp;
    //CvPoint3D32f left_loc,       right_loc,       top_loc,       bottom_loc;

    left_neighbor = right_neighbor = top_neighbor = bottom_neighbor = false;
    for (unsigned int i = 0; i < neighbors.size(); i++)
    {
        if (neighbors.at(i).x < x && neighbors.at(i).y == y)
        {
            left_neighbor  = true;
            left_neighborp = neighbors.at(i);
            left_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*left_neighborp.y))[left_neighborp.x];
        }
        if (neighbors.at(i).x > x && neighbors.at(i).y == y)
        {
            right_neighbor  = true;
            right_neighborp = neighbors.at(i);
            right_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*right_neighborp.y))[right_neighborp.x];
        }
        if (neighbors.at(i).y < y && neighbors.at(i).x == x)
        {
            top_neighbor  = true;
            top_neighborp = neighbors.at(i);
            top_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*top_neighborp.y))[top_neighborp.x];
        }
        if (neighbors.at(i).y > y && neighbors.at(i).x == x)
        {
            bottom_neighbor  = true;
            bottom_neighborp = neighbors.at(i);
            bottom_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*bottom_neighborp.y))[bottom_neighborp.x];
        }
    }

    if (left_neighbor && right_neighbor && top_neighbor && bottom_neighbor)
    {
        //Edge sensing
        //http://scien.stanford.edu/class/psych221/projects/99/tingchen/algodep/edgesense.html
        float horizontal_gradient = (left_v + right_v) / 2.0;
        float vertical_gradient   = (top_v + bottom_v) / 2.0;
        float threshold = (horizontal_gradient + vertical_gradient) / 2.0;
        if (horizontal_gradient < threshold && vertical_gradient > threshold)
        {
            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
                (IMG_DEPTH) lrint((left_v + right_v) / 2.0);
        } else if (horizontal_gradient > threshold && vertical_gradient < threshold)
        {
            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
                (IMG_DEPTH) lrint((top_v + bottom_v) / 2.0);
        } else {
            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
                (IMG_DEPTH) lrint((top_v + bottom_v + left_v + right_v) / 4.0);
        }
        return true;
    } else if (left_neighbor && right_neighbor)
    {
        ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
            (IMG_DEPTH) lrint((left_v + right_v) / 2.0);
        return true;
    } else if (top_neighbor && bottom_neighbor)
    {
        ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] = (IMG_DEPTH) lrint((top_v + bottom_v) / 2.0);
        return true;
    } else {
        if (neighbors.size() > 1)
        {
            //Calculcate weights
            vector<float> weights;
            double sum_intensity    = 0;
            double sum_weights      = 0;
            float size              = std::pow(.5f, 2);

            for (unsigned int i = 0; i < neighbors.size(); i++)
            {
                CvPoint p = neighbors[i];
                float weight = exp(-(std::pow((float)(p.x-x), 2) + pow((float)(p.y-y), 2)) / size);
                weight = max((float).01, weight);
                sum_intensity += weight * ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*p.y))[p.x];
                sum_weights   += weight;

            }

            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] = (IMG_DEPTH) (sum_intensity / sum_weights);
            return true;
        } else {
            return false;
        }
    }
    return false;
}


void fill_missing_pixels(IplImage *original_d16_c1, IplImage *occupancy_d8_c1)
{
    int count = 0;
    bool making_progress = true;
    vector<CvPoint> *failed  = new vector<CvPoint>();
    vector<CvPoint> *failed2 = new vector<CvPoint>();
    for (int yi=0; yi < original_d16_c1->height; yi++)
    for (int xi=0; xi < original_d16_c1->width;  xi++)
    {
        if (OCC_EMPTY == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yi))[xi])
            failed->push_back(cvPoint(xi,yi));
    }

    while (making_progress)
    {
        making_progress = false;
        vector<CvPoint>::iterator iter;
        vector<CvPoint> succeeded;
        for (iter = failed->begin(); iter != failed->end(); iter++)
        {
            CvPoint p = *iter;
            if (fill_missing_pixel(p.x, p.y, original_d16_c1, occupancy_d8_c1))
            {
                making_progress = true;
                succeeded.push_back(cvPoint(p.x, p.y));
            } else {
                failed2->push_back(cvPoint(p.x, p.y));
            }

            if (DISPLAY && count % 10000 == 0)
            {
              cvShowImage("img", original_d16_c1);
              cvWaitKey(5);
            }
            count++;
        }
        //printf("found %d pixels to fill\n", succeeded.size());

        //Mark pixels as occupied here instead of above, as results won't be affected by
        //update order of fill-in sweeps
        for (iter = succeeded.begin(); iter != succeeded.end(); iter++)
        {
            CvPoint p = *iter;
            ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*p.y))[p.x] = OCC_FILLED;
        }

        delete failed;
        failed = failed2;
        failed2 = new vector<CvPoint>();
    }

    delete failed;
    delete failed2;
}


void CvStereoCamModel::point3dToImage(
    const CvMat *XYZIs,
    cv::WImage1_b *intensity_map,
    cv::WImage3_f *xyz_map
) const {
  __BEGIN__
  //CV_ERROR( CV_StsNotImplemented, "Coming soon");
  {
    if (DISPLAY)
    {
      cvNamedWindow("img", 1);
    }
    int image_width = intensity_map->Width();
    int image_height= intensity_map->Height();
    int numPoints = XYZIs->rows;
    CvMat XYZIs_C1;
    CvMat XYZs_C1;
    double uvs_data[2*numPoints];
    /// TODO: how do I create a cvmat of 2-channel and that go with XYZIs
    CvMat uvs = cvMat(numPoints, 1, CV_64FC2, uvs_data);
    unsigned short uvs_16u_data[2*numPoints];
    CvMat uvs_16u = cvMat(numPoints, 1, CV_16UC2, uvs_16u_data);
    IplImage* xyz_32FC3 =  xyz_map->Ipl();

    // get a view of the input matrix as numPoints x4, 1-channel matrix
    cvReshape(XYZIs, &XYZIs_C1, 1, numPoints);
    // get a view of the first 3 columns of XYZIs_C1, i.e. the X, Y, Z's
    cvGetCols(&XYZIs_C1, &XYZs_C1, 0, 3);

    //Whether the corresponding pixel in 'img' is filled
    IplImage *occupancy = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_8U, 1);
    cvSet(occupancy, cvScalar(OCC_EMPTY));

    ////////////////////////////////////////////////////////////////////////////////////////
    // Project 3D xyz points into camera uv points and plot associated intensity in image
    ////////////////////////////////////////////////////////////////////////////////////////
    //Turn 3D points into image points
    IplImage *sums = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_64F, 1);
    cvSet(sums, cvScalar(0));
    IplImage *hits = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_32S, 1);
    cvSet(hits, cvScalar(0));

    int out_bound_pixels = 0;
    int behind_camera    = 0;

    int max_hits = 0;
    unsigned int max_sum = 0;
    if (VERBOSE)
      cout << "reading scans...\n";
    //For each 3D point
    if (VERBOSE)
      cout << "looping over points " << numPoints << "... and render over "<<image_width<<","<<image_height<<"\n";

    // converts the x, y positions to left camera image pixel locations.
    cartToLeftCam(&XYZs_C1, &uvs);
    cvConvert(&uvs, &uvs_16u);

    for (int i = 0; i < numPoints; i++)
    {
      CvPoint3D32f point3d;
      point3d.x = cvmGet(&XYZIs_C1, i, 0);
      point3d.y = cvmGet(&XYZIs_C1, i, 1);
      point3d.z = cvmGet(&XYZIs_C1, i, 2);

      // CvPoint3D32f cvpt = transform_to_cv_frame(scan_point);
      if (point3d.z > 0 && magnitude(point3d) > 0.08)
      {
        CvPoint screen_pt = cvPoint(uvs_16u_data[i*2], uvs_16u_data[i*2+1]);
        if (in_bounds(screen_pt, image_width, image_height))
        {
          unsigned int associated_intensity = (uint)lrint(cvmGet(&XYZIs_C1, i, 3));

          //Add to sums
          if (associated_intensity > 0)
          {
            ((char*)         (occupancy->imageData + occupancy->widthStep*screen_pt.y))[screen_pt.x] = OCC_FILLED;
            double current_intensity = ((double*)(sums->imageData + sums->widthStep*screen_pt.y))[screen_pt.x];
            ((double *)      (sums->imageData + sums->widthStep*screen_pt.y))          [screen_pt.x] = current_intensity + associated_intensity;
            ((unsigned int *)(hits->imageData + hits->widthStep*screen_pt.y))          [screen_pt.x] += 1;

            int cur_hits = ((unsigned int*)(hits->imageData + hits->widthStep*screen_pt.y))[screen_pt.x];
            if (max_hits < cur_hits)
            {
              max_hits = cur_hits;
              max_sum = max_hits;
            }

            // fill out the entry on the xyz map. With this implementation
            // the same pixel location may get overridden again and again.
            int row = screen_pt.y;
            int col = screen_pt.x;
            CV_IMAGE_ELEM( xyz_32FC3, float, row, col*3     ) = point3d.x;
            CV_IMAGE_ELEM( xyz_32FC3, float, row, col*3 + 1 ) = point3d.y;
            CV_IMAGE_ELEM( xyz_32FC3, float, row, col*3 + 2 ) = point3d.z;
          }

        } else
        {
          out_bound_pixels++;
        }
      } else
      {
        behind_camera++;
      }
    }

    if (VERBOSE)
    {
      cout << "out_bound_pixels: " << out_bound_pixels << endl;
      cout << "behind_camera: "    << behind_camera << endl;
      cout << "max hits: "    << max_hits << endl;
      cout << "averging hits...\n";
    }

    IplImage* img = intensity_map->Ipl();
    //Average hits in each cell and set to new image
    for (int y = 0; y < sums->height; y++)
      for (int x = 0; x < sums->width;  x++)
      {
        double current_intensity       = ((double*)(sums->imageData + sums->widthStep*y))[x];
        unsigned int num_hits          = ((unsigned int*)(hits->imageData + hits->widthStep*y))[x];
        if (current_intensity != 0) {
          ((IMG_DEPTH*) (img->imageData + img->widthStep*y))[x] =
            int2uchar(lrint((double) current_intensity / (double) num_hits));
        }
        else {
          ((IMG_DEPTH*) (img->imageData + img->widthStep*y))[x] =  0;
        }
      }

    if (DISPLAY)
    {
      //Show image before filling it in
      cvShowImage("img", img);
      printf("Waiting key...\n");
      cvWaitKey(33);
    }

    //Loop over unfilled pixels & set them to average of 4 neighbors
    fill_missing_pixels(img, occupancy);

    if (DISPLAY)
      cvShowImage("img", img);
  }
  __END__
}

double  CvStereoCamModel::getDeltaU(double deltaX, double Z) const {
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  if (Z == 0) {
    return DBL_MAX;
  }
  __END__
  return Fx_*deltaX/Z;
}

double  CvStereoCamModel::getDeltaX(double deltaU, double d) const {
  double dx;
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  {
    double dn = (d*Du_ - (Clx_ -Crx_));
    if (dn==0) {
      dx = 0.;
    } else {
      dx = deltaU * Tx_/dn;
    }
  }
  __END__
  return dx;
}
double  CvStereoCamModel::getDeltaV(double deltaY, double Z) const {
  double dv;
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  if (Z == 0) {
    dv = DBL_MAX;
  } else {
    dv = Fy_*deltaY/Z;
  }

  __END__
  return dv;
}

double  CvStereoCamModel::getDeltaY(double deltaV, double d) const {
  double dy;
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  {
    double dn = (d*Du_ - (Clx_ -Crx_))*Fy_;
    if (dn==0) {
      dy =  0.;
    } else {
      dy = deltaV * Tx_ *Fx_ /dn;
    }
  }
  __END__
  return dy;
}

double CvStereoCamModel::getZ(double d) const {
  double z;
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  } else {
    // symbolically
    // Fx_*Tx_/(d*Du_ - (Clx_ - Crx_))
    double m32 = matdata_disp_to_cart_[3*4 + 2];
    double m33 = matdata_disp_to_cart_[3*4 + 3];
    double m23 = matdata_disp_to_cart_[2*4 + 3];
    double dn = m32 * d - m33;
    if (dn == 0) {
      z = DBL_MAX;
    } else {
      z = m23/dn;
    }
  }

  __END__
  return z;
}

double CvStereoCamModel::getDisparity(double Z) const {
  double d;
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
  if (Z==0) {
    d = DBL_MAX;
  } else {

    // symbolically ((Clx_-Crx_) + Fx_*Tx_/Z)/Du_

    double m22 = matdata_cart_to_disp_[2*4 + 2];
    double m23 = matdata_cart_to_disp_[2*4 + 3];

    d = m22 + m23/Z;
  }
  __END__
  return d;
}

//Iz is single channel 32F.  Zero values are not displayed. Passing a null image turns this off
void CvStereoCamModel::dspl_depth_image(IplImage *Iz, double Zmin, double Zmax)
{
  __BEGIN__
  if (parameterized_ == false) {
    CV_ERROR( CV_StsInternal, "object not parameterized");
  }
	assert(Iz->nChannels == 1);
	//SHUT DOWN WINDOW IF Iz is NULL
	if(!Iz) {
		if(Iz8U_) {
			cvDestroyWindow("Depth");
			cvReleaseImage(&Iz8U_);
		}
		CV_ERROR(CV_StsBadArg, "argument Iz is NULL");
	}
	{
	  //CHECK IF WE NEED TO INIT
	  if((!Iz8U_) || (Iz8U_->width != Iz->width) || (Iz8U_->height != Iz->height)) {
	    if(Iz8U_) {
	      cvReleaseImage(&Iz8U_);
	      cvDestroyWindow("Depth");
	    }
	    Iz8U_ = cvCreateImage(cvSize(Iz->width,Iz->height),IPL_DEPTH_8U,1);
	    cvNamedWindow("Depth",0);
	  }
	  //SET UP PARAMETERS
	  int w = Iz->width, h = Iz->height, ws = Iz8U_->widthStep;
	  int jumpby = ws - w;
	  Zmin *= 1000.0; Zmax *= 1000.0; //depth is in meters, depth map is in milimeters
	  bool mZ = false, MZ = false;    // if Zmin or Zmax == 0, then compute their values from min and max on depth map
	  if(Zmin == 0.0) { mZ =  true; Zmin = 99999999.0;}
	  if(Zmax == 0.0) MZ = true;
	  float *fptr = (float *)(Iz->imageData),fv;
	  if(mZ || MZ){
	    for(int i = 0; i<w*h; ++i) {
	      fv = *fptr++;
	      if(fv <= 0) continue;
	      if((mZ) && (Zmin > fv)) Zmin = fv;
	      if((MZ) && (Zmax < fv)) Zmax = fv;
	    }
	  }
	  if(Zmax <= Zmin) Zmax += 1.0; //Avoid max <= min
	  double scaleit = 255.0 / (Zmax - Zmin);
	  unsigned char *cptr = (unsigned char *)(Iz8U_->imageData);
	  fptr = (float *)(Iz->imageData);

	  //LOOP
	  for(int y=0; y<h; ++y){
	    for(int x=0; x<w; ++x) {
	      fv = *fptr++;
	      if((fv <= 0.0)||(fv > Zmax)){
	        *cptr++ = 0;
	      }
	      else if(fv < Zmin) {
	        *cptr++ = 255;
	      }
	      else {
	        *cptr++ = (unsigned char)((double)255.0 - scaleit*((double)fv - Zmin));	  //Invert so that closer objects are brighter
	      }
	    }
	    cptr += jumpby;
	  }
	  //SHOW THE PRETTY DEPTH IMAGE
	  cvShowImage("Depth",Iz8U_);
	}
	__END__;
}


// compute a depth mask according to the minZ and maxZ
void CvStereoCamModel::getDepthMask(// disparity image
				    const IplImage* dispImg,
				    // pre-allocate image buffer for the depth mask
				    IplImage* depthMask,
				    // mininum z in mask
				    double minZ,
				    // max z in mask
				    double maxZ,
				    // post processing options
				    PostProcessOptions post_process_options
) const {
    double maxDisp = getDisparity(minZ);
    double minDisp = getDisparity(maxZ);

    // printf("range mask [%f, %f] => [%f, %f]\n", minZ, maxZ, minDisp, maxDisp);

    // fill in the mask according to disparity or depth
    cvInRangeS(dispImg, cvScalar(minDisp), cvScalar(maxDisp), depthMask);

    switch (post_process_options) {
    case NO_POST_PROCESS:
      break;
    case POLYGONES:
      // two simple morphology operation seem to be good enough. But
      // but connected component analysis provides blob with better shape
      cvMorphologyEx(depthMask, depthMask, NULL, (IplConvKernel *)OpenKernel, CV_MOP_OPEN, 1);
      //cvMorphologyEx(depthMask, depthMask, NULL, DilateKernel, CV_MOP_CLOSE, 1);
      cvDilate(depthMask, depthMask, (IplConvKernel *)DilateKernel, 1);
      break;
    case CONVEX_HULLS: {
      const float perimScale = 16;
      const int maxNumBBoxes = 25;
      int numCComp = maxNumBBoxes;
      connectedComponents(depthMask, 0,
          (IplConvKernel *)OpenKernel,
          (IplConvKernel *)CloseKernel,
          perimScale, &numCComp,
          (CvRect *)NULL, // bboxes,
          (CvPoint *)NULL);
      cvDilate(depthMask, depthMask, (IplConvKernel *)DilateKernel, 1);
    }
    }
}

//Just some convienience macros
#define CV_CVX_WHITE	CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK	CV_RGB(0x00,0x00,0x00)



/// The following function is modified based on Gary Bradsky's function cvconnectedComponents in cv_yuv_codebook.cpp
//void cvconnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int *num, CvRect *bbs, CvPoint *centers)
// This cleans up the forground segmentation mask derived from calls to cvbackgroundDiff
//
// mask			Is a grayscale (8 bit depth) "raw" mask image which will be cleaned up
//
// OPTIONAL PARAMETERS:
// @param poly1_hull0	If set, approximate connected component by (DEFAULT) polygon, or else convex hull (0)
// @param perimScale 	Len = image (width+height)/perimScale.  If contour len < this, delete that contour (DEFAULT: 4)
// @param openKernel   the kernel used to remove noise in raw mask. If NULL, 3x3 rectangle is used
// @param closeKernel  the kernel used to merge blobs in raw mask. If NULL, 3x3 rectangle is used
// @param num			Maximum number of rectangles and/or centers to return, on return, will contain number filled (DEFAULT: NULL)
// @param bbs			Pointer to bounding box rectangle vector of length num.  (DEFAULT SETTING: NULL)
// @param centers		Pointer to contour centers vectors of length num (DEFAULT: NULL)
//
void CvStereoCamModel::connectedComponents(IplImage *mask, int poly1_hull0,
			   IplConvKernel* openKernel,
			   IplConvKernel* closeKernel,
			   float perimScale, int *num, CvRect *bbs, CvPoint *centers) const
{
  //CLEAN UP RAW MASK
  // open followed by a close
  cvMorphologyEx( mask, mask, NULL, openKernel,  CV_MOP_OPEN,  CVCLOSE_ITR );
  cvMorphologyEx( mask, mask, NULL, closeKernel, CV_MOP_CLOSE, CVCLOSE_ITR );

  //FIND CONTOURS AROUND ONLY BIGGER REGIONS
  if( mem_storage_==NULL ) mem_storage_ = cvCreateMemStorage(0);
  else cvClearMemStorage(mem_storage_);

  CvContourScanner scanner = cvStartFindContours(mask,mem_storage_,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  CvSeq* c;
  int numCont = 0;
  while( (c = cvFindNextContour( scanner )) != NULL )
    {
      double len = cvContourPerimeter( c );
      double q = (mask->height + mask->width) /perimScale;   //calculate perimeter len threshold
      if( len < q ) //Get rid of blob if it's perimeter is too small
	{
	  cvSubstituteContour( scanner, NULL );
	}
      else //Smooth it's edges if it's large enough
	{
	  CvSeq* c_new;
	  if(poly1_hull0) //Polygonal approximation of the segmentation
	    c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage_,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
	  else //Convex Hull of the segmentation
	    c_new = cvConvexHull2(c,mem_storage_,CV_CLOCKWISE,1);
	  cvSubstituteContour( scanner, c_new );
	  numCont++;
	}
    }
  CvSeq* contours = cvEndFindContours( &scanner );

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
