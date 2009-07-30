#include <image_descriptors_gpl/image_descriptors_gpl.h>

using namespace kutility;
using namespace std;
using namespace cv;

/****************************************************************************
*************  ImageDescriptor::Daisy
****************************************************************************/


Daisy::Daisy(double rad, int rad_q_no, int th_q_no, int hist_th_q_no, Daisy* im_provider) :
  ImageDescriptor(),
  rad_(rad),
  rad_q_no_(rad_q_no),
  th_q_no_(th_q_no),
  hist_th_q_no_(hist_th_q_no),
  dai_(),
  im_(NULL),
  im_provider_(im_provider)
{
  char buf[200];
  sprintf(buf, "DAISY_rad%g_radq%d_thq%d_histq%d", rad_, rad_q_no_, th_q_no_, hist_th_q_no_);
  name_.assign(buf);

  dai_.verbose(0);
  dai_.set_parameters(rad_, rad_q_no_, th_q_no_, hist_th_q_no_);

  result_size_ = dai_.descriptor_size();
}

Daisy::~Daisy() {

}

void Daisy::clearImageCache() {
  dai_.reset();

  // -- im_ is deleted by load_gray_image().
  //   if(!im_provider_ && im_) 
  //     delete[] im_;
}

void Daisy::compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results) {
  setImage(img);

  // -- Stupid way to pass img to daisy.
  if(im_provider_) {
    im_ = im_provider_->im_;
  }
  else {
    char filename[] = "temp.pgm";
    IplImage* gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
    cvCvtColor(img, gray, CV_BGR2GRAY);
    cvSaveImage(filename, gray);
    load_gray_image(filename, im_, img->height, img->width);
  }
  dai_.verbose(0);
  dai_.set_image(im_, img->height, img->width);
  dai_.set_parameters(rad_, rad_q_no_, th_q_no_, hist_th_q_no_);
  dai_.initialize_single_descriptor_mode();


  results.clear();
  results.resize(points.size());
  int nValid = 0;
  START_TIMER();
  for(size_t i=0; i<points.size(); ++i) {
    // -- Get the descriptor.
    assert((int)result_size_ == dai_.descriptor_size());
    float* thor = new float[result_size_];
    memset(thor, 0, sizeof(float)*result_size_);
    dai_.get_descriptor((int)points[i].pt.y, (int)points[i].pt.x, 0, thor);

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
  STOP_TIMER();

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
