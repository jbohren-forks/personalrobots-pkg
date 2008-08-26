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
  FixedSpinMedium()
    {
      name = string("FixedSpinMedium");
    }

  bool operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, NEWMAT::Matrix** result, bool debug=false);
};
