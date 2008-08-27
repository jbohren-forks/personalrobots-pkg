#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Point3DFloat32.h>
#include <std_msgs/VisualizationMarker.h>
#include <ros/node.h>
#include <iostream>
#include <smartScan.h>
#include <dorylus.h>
#include <scene_labeler_stereo.h>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include <string>
#include <smartScan.h>
#include <iostream>
#include <fstream>

class Descriptor 
{
 public:

  string name;

  //Unfortunately ss cannot be const because it might create vtkdata.
  virtual bool operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, NEWMAT::Matrix** result, bool debug=false)
  {
    return false;
  }

  virtual void display(const NEWMAT::Matrix& result)
  {
    cout << name << " has no display function yet." << endl;
  }

};


/* class FixedSpinLarge : public Descriptor */
/* { */
/*  public: */
/*   FixedSpinLarge()  */
/*     { */
/*       name = string("FixedSpinLarge"); */
/*     } */
  
/*   NEWMAT::Matrix operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, bool debug=false); */
/* }; */

class FixedSpinMedium : public Descriptor
{

 public:
  float support_;
  float pixelsPerMeter_;
  int height_, width_;
  scan_utils::Grid2D *psi_;

  FixedSpinMedium()
    {
      name = string("FixedSpinMedium");
      support_ = .05;  //.1
      pixelsPerMeter_ = 200;

      psi_ = new scan_utils::Grid2D(ceil(2*support_*pixelsPerMeter_), ceil(support_*pixelsPerMeter_));
      psi_->getSize(height_, width_);
    }

  ~FixedSpinMedium()
    {
      delete psi_;
    }
    

  bool operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, NEWMAT::Matrix** result, bool debug=false);
  void display(const NEWMAT::Matrix& result);
};


class FixedSpinLarge : public Descriptor
{

 public:
  float support_;
  float pixelsPerMeter_;
  int height_, width_;
  scan_utils::Grid2D *psi_;

  FixedSpinLarge()
    {
      name = string("FixedSpinLarge");
      support_ = .20;
      pixelsPerMeter_ = 50;

      psi_ = new scan_utils::Grid2D(ceil(2*support_*pixelsPerMeter_), ceil(support_*pixelsPerMeter_));
      psi_->getSize(height_, width_);
    }

  ~FixedSpinLarge()
    {
      delete psi_;
    }
    

  bool operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, NEWMAT::Matrix** result, bool debug=false);
  void display(const NEWMAT::Matrix& result);
};
