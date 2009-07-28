#ifndef IMAGE_DESCRIPTORS_GPL_H
#define IMAGE_DESCRIPTORS_GPL_H

#include <daisy/daisy.h>
#include <descriptors.h>
#include <time.h>

#define START_TIMER() clock_t timer_start_aoeu = clock();
#define STOP_TIMER() cout << "Timer: " << (double)(clock() - timer_start_aoeu) / (double)CLOCKS_PER_SEC << " seconds." << endl;


/***************************************************************************
***********  DAISY
****************************************************************************/

class Daisy : public ImageDescriptor {
 public:
  
  Daisy(double rad = 15, int rad_q_no = 3, int th_q_no = 8, int hist_th_q_no = 8, Daisy* im_provider = NULL);
  ~Daisy();
  void compute(IplImage* img, const cv::Vector<cv::Keypoint>& points, vvf& results);
  void clearImageCache();
 
 protected:
  //! Radius of the descriptor.
  double rad_;
  //! Quantization of the radius, i.e. how many rings to use (does not include the center circle).
  int rad_q_no_;
  //! Quantization of the angle.
  int th_q_no_;
  //! Quantization of gradient orientations.
  int hist_th_q_no_;
  daisy dai_;
  //! Image format used by Daisy.
  uchar* im_;
  Daisy* im_provider_;
};

#endif
