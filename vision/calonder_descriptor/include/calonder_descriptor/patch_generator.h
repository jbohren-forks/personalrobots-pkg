#ifndef FEATURES_PATCH_GENERATOR_H
#define FEATURES_PATCH_GENERATOR_H

#include <cv.h>
#include "calonder_descriptor/rng.h"
#include <cmath>

namespace features {

class PatchGenerator
{
public:
  PatchGenerator(IplImage* source, Rng &rng);

  void operator() (CvPoint pt, IplImage* patch);

  void setSource(IplImage* source);

  //! Rotation
  void setThetaBounds(double theta_min, double theta_max);
  //! Skew rotation
  void setPhiBounds(double phi_min, double phi_max);
  //! Scaling
  void setLambdaBounds(double lambda_min, double lambda_max);

  void setRandomBackground(bool on_off);
  void addWhiteNoise(bool on_off);
  void setNoiseLevel(int level);

  static const double DEFAULT_THETA_MIN = -M_PI;
  static const double DEFAULT_THETA_MAX = M_PI;
  static const double DEFAULT_PHI_MIN = -M_PI;
  static const double DEFAULT_PHI_MAX = M_PI;
  static const double DEFAULT_LAMBDA_MIN = 0.6;
  static const double DEFAULT_LAMBDA_MAX = 1.5;
  static const int DEFAULT_NOISE_LEVEL = 20;

private:
  IplImage* source_;
  double theta_min_, theta_max_;
  double phi_min_, phi_max_;
  double lambda_min_, lambda_max_;
  bool random_background_;
  bool white_noise_;
  int noise_level_;
  Rng &rng_;
};

} // namespace features

#endif
