#include <image_descriptors_gpl/image_descriptors_gpl.h>

using namespace kutility;
using namespace std;
using namespace cv;

/****************************************************************************
*************  ImageDescriptor::Daisy
****************************************************************************/


Daisy::Daisy(double rad, int rad_q_no, int th_q_no, int hist_th_q_no) :
  ImageDescriptor(),
  rad_(rad),
  rad_q_no_(rad_q_no),
  th_q_no_(th_q_no),
  hist_th_q_no_(hist_th_q_no)
{
  char buf[200];
  sprintf(buf, "DAISY_rad%g_radq%d_thq%d_histq%d", rad_, rad_q_no_, th_q_no_, hist_th_q_no_);
  name_.assign(buf);

  dai.set_parameters(rad_, rad_q_no_, th_q_no_, hist_th_q_no_);
  dai.set_normalization(NRM_FULL);
  dai.initialize_single_descriptor_mode();
  int ws = dai.compute_workspace_memory();
  float* workspace = new float[ ws ];
  dai.set_workspace_memory( workspace, ws);

  result_size_ = dai.descriptor_size();
}

Daisy::~Daisy() {

}

void Daisy::compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results) {
  // -- Stupid way to pass img to daisy.
  char filename[] = "temp.pgm";
  IplImage* gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
  cvCvtColor(img, gray, CV_BGR2GRAY);
  cvSaveImage(filename, gray);
  uchar* im = NULL;
  load_gray_image(filename, im, img->height, img->width);
  dai.set_image(im, img->height, img->width);


  results.clear();
  results.resize(points.size());
  int nValid = 0;
  for(size_t i=0; i<points.size(); ++i) {
    // -- Get the descriptor.
    float* thor = new float[result_size_];
    memset(thor, 0, sizeof(float)*result_size_);
    dai.get_descriptor((int)points[i].pt.y, (int)points[i].pt.x, thor);

    // -- Check if it's all zeros.
    bool valid = false;
    for(size_t j=0; j<result_size_; ++j) {
      if(thor[j] != 0) {
	valid = true;
	break;
      }
    }

    if(!valid) {
      delete[] thor;
      results[i] = Vector<float>();
      continue;
    }
      
    // -- Put into results.
    results[i] = Vector<float>(thor, result_size_, false); //Don't copy the data.
    nValid++;
  }

  if(debug_) {
    cout << "Debugging " << name_ << endl;
    cout << "Got " << nValid << " valid features out of " << points.size() << endl;
    cout << "Vector: ";
    float l2 = 0;
    for(size_t i=0; i<results[0].size(); ++i) {
      cout << results[0][i] << " ";
      l2 += results[0][i] * results[0][i];
    }
    cout << endl;

    l2 = sqrt(l2);
    cout << "L2 norm: " << l2 << endl;
    commonDebug(points[0]);
  }
}
