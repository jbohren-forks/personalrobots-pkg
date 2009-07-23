#include <image_descriptors_gpl/image_descriptors_gpl.h>

using namespace kutility;

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

  result_size_ = dai.descriptor_size();
}

Daisy::~Daisy() {

}

void Daisy::compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results) {
  // -- Stupid way to pass img to daisy.
  cvSaveImage("temp.png", img);
  uchar* im = NULL;
  load_gray_image("temp.png", im, img->height, img->width);
  dai.set_image(im, img->height, img->width);


  float* thor = new float[dai.descriptor_size()];
  memset(thor, 0, sizeof(float)*dai.descriptor_size() );
}
