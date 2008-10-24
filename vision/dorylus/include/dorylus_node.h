#include <std_msgs/PointCloud.h>
#include <std_msgs/Point32.h>
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
#include <calonder_descriptor/matcher.h>
#include <calonder_descriptor/rtree_classifier.h>
#include <cvwimage.h>


class Descriptor 
{
 public:

  string name_;
  unsigned int result_size_; 

  //Unfortunately ss cannot be const because it might create vtkdata.
  virtual bool operator()(SmartScan &ss, IplImage &img, float x, float y, float z, int row, int col, NEWMAT::Matrix** result, bool debug=false)
  {
    return false;
  }

  virtual void display(const NEWMAT::Matrix& result)
  {
    cout << name_ << " has no display function yet." << endl;
  }

};


class SpinImage : public Descriptor
{

 public:
  float support_;
  float pixelsPerMeter_;
  int height_, width_;
  scan_utils::Grid2D *psi_;
  bool fixed_;
  
 SpinImage(string name, float support, float pixelsPerMeter, bool fixed) 
   : support_(support), pixelsPerMeter_(pixelsPerMeter), fixed_(fixed)
    {
      name_ = name;

/*       name = string("FixedSpinMedium"); */
/*       support_ = .05;  //.1 */
/*       pixelsPerMeter_ = 200; */

/*       name = string("FixedSpinLarge"); */
/*       support_ = .20; */
/*       pixelsPerMeter_ = 50; */


      psi_ = new scan_utils::Grid2D(ceil(2*support_*pixelsPerMeter_), ceil(support_*pixelsPerMeter_));
      psi_->getSize(height_, width_);
      result_size_ = height_*width_;
    }

  ~SpinImage()
    {
      delete psi_;
    }
    

  bool operator()(SmartScan &ss, IplImage &img, float x, float y, float z, int row, int col, NEWMAT::Matrix** result, bool debug=false);
  void display(const NEWMAT::Matrix& result);
};



class Calonder : public Descriptor
{
 public:
  features::RTreeClassifier rt_;
  int start_, end_; //0-indexed, not 1-indexed like newmat. 0, -1 is the entire thing; 0, 100 is just the first 100.  end_ is not included.

  Calonder(string name, string rt_file, int start, int end)
    {

      start_ = start;
      if(end == -1)
	end_ = rt_.classes();
      else
	end_ = end;


      name_ = name;
      rt_.read(rt_file.c_str());
      //result_size_ = rt_.classes();
      result_size_ = end_ - start_;
      assert(start_ >= 0 && end_ <= rt_.classes() && start <= end);
    }

  ~Calonder()
    {
    }
    
  bool operator()(SmartScan &ss, IplImage &img, float x, float y, float z, int row, int col, NEWMAT::Matrix** result, bool debug=false);
  void display(const NEWMAT::Matrix& result);
};
