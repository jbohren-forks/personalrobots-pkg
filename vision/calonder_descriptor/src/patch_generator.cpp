#include "calonder_descriptor/patch_generator.h"
#include <cassert>
#include <cstring>

namespace features {

PatchGenerator::PatchGenerator(IplImage* source, Rng &rng)
  : source_(source),
    theta_min_(PatchGenerator::DEFAULT_THETA_MIN),
    theta_max_(PatchGenerator::DEFAULT_THETA_MAX),
    phi_min_(PatchGenerator::DEFAULT_PHI_MIN),
    phi_max_(PatchGenerator::DEFAULT_PHI_MAX),
    lambda_min_(PatchGenerator::DEFAULT_LAMBDA_MIN),
    lambda_max_(PatchGenerator::DEFAULT_LAMBDA_MAX),
    random_background_(false), white_noise_(false),
    noise_level_(PatchGenerator::DEFAULT_NOISE_LEVEL),
    rng_(rng)
{}

void PatchGenerator::operator() (CvPoint pt, IplImage* patch)
{
  // NOTE: for efficient random noise, we assume patch dimensions are
  //       multiples of sizeof(Rng::int_type)
  typedef Rng::int_type fill_type;
  assert(patch && patch->depth == source_->depth);
  assert(patch->width % sizeof(fill_type) == 0);
  assert(patch->height % sizeof(fill_type) == 0);
  
  double theta = rng_.uniform(theta_min_, theta_max_);
  double phi = rng_.uniform(phi_min_, phi_max_);
  double lambda1 = rng_.uniform(lambda_min_, lambda_max_);
  double lambda2 = rng_.uniform(lambda_min_, lambda_max_);

  // Calculate random parameterized affine transformation A,
  // A = T(patch center) * R(theta) * R(phi)' *
  //     S(lambda1, lambda2) * R(phi) * T(-pt)
  double st = sin(theta);
  double ct = cos(theta);
  double sp = sin(phi);
  double cp = cos(phi);
  double c2p = cp*cp;
  double s2p = sp*sp;

  double A = lambda1*c2p + lambda2*s2p;
  double B = (lambda2 - lambda1)*sp*cp;
  double C = lambda1*s2p + lambda2*c2p;

  double Ax_plus_By = A*pt.x + B*pt.y;
  double Bx_plus_Cy = B*pt.x + C*pt.y;

  // Allocate transform matrix on stack for efficiency.
  float buffer[6];
  buffer[0] = A*ct - B*st;
  buffer[1] = B*ct - C*st;
  buffer[2] = -ct*Ax_plus_By + st*Bx_plus_Cy + patch->width/2;
  buffer[3] = A*st + B*ct;
  buffer[4] = B*st + C*ct;
  buffer[5] = -st*Ax_plus_By - ct*Bx_plus_Cy + patch->height/2;
  CvMat transform = cvMat(2, 3, CV_32F, buffer);

  if (random_background_) {
    // Fill patch with random noise
    fill_type* data = (fill_type*) patch->imageData;
    int data_step = patch->widthStep / sizeof(fill_type);
    int data_width = patch->width / sizeof(fill_type);
    for (int y = 0; y < patch->height; ++y, data += data_step) {
      for (int x = 0; x < data_width; ++x) {
        data[x] = rng_();
      }
    }
  } else {
    cvSet(patch, cvScalarAll(128));
  }

  // Suppress CV_WARP_FILL_OUTLIERS flag which is set by default.
  cvWarpAffine(source_, patch, &transform, CV_INTER_LINEAR);

  if (white_noise_) {
    // Add white noise to patch
    // TODO: Use pregenerated (Gaussian?) white noise?
    uchar *data = (uchar*) patch->imageData;
    for (int y = 0; y < patch->height; ++y, data+= patch->widthStep) {
      for (int x = 0; x < patch->width; x += sizeof(unsigned long)) {
        uchar rand_vals[sizeof(unsigned long)];
        *reinterpret_cast<unsigned long*>(rand_vals) = rng_();
        for (unsigned int i = 0; i < sizeof(unsigned long); ++i) {
          int noise = rand_vals[i] % (2*noise_level_ + 1) - noise_level_;
          uchar &elem = data[x + i];
          int new_val = noise + elem;
          if (new_val < 0)
            elem = 0;
          else if (new_val > 255)
            elem = 255;
          else
            elem = new_val;
        }
      }
    }
  }
}

void PatchGenerator::setSource(IplImage* source)
{
  source_ = source;
}
  
void PatchGenerator::setThetaBounds(double theta_min, double theta_max)
{
  theta_min_ = theta_min;
  theta_max_ = theta_max;
}

void PatchGenerator::setPhiBounds(double phi_min, double phi_max)
{
  phi_min_ = phi_min;
  phi_max_ = phi_max;
}

void PatchGenerator::setLambdaBounds(double lambda_min, double lambda_max)
{
  lambda_min_ = lambda_min;
  lambda_max_ = lambda_max;
}

void PatchGenerator::setRandomBackground(bool on_off)
{
  random_background_ = on_off;
}

void PatchGenerator::addWhiteNoise(bool on_off)
{
  white_noise_ = on_off;
}

void PatchGenerator::setNoiseLevel(int level)
{
  noise_level_ = level;
}

} // namespace features
