#include <daisy/daisy.h>
#include <descriptors.h>



/***************************************************************************
***********  DAISY
****************************************************************************/

class Daisy : public ImageDescriptor {
 public:
  
  Daisy(double rad = 15, int rad_q_no = 3, int th_q_no = 8, int hist_th_q_no = 8);
  ~Daisy();
  void compute(IplImage* img, const cv::Vector<cv::Keypoint>& points, vvf& results);
 
 private:
  double rad_;
  int rad_q_no_;
  int th_q_no_;
  int hist_th_q_no_;
  daisy dai;
};

