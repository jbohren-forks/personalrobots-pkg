
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdio.h>

class  ParamRange {
public:
  float low_;
  float high_;
  float step_;
};
/// Each of the following parameters is an array of 3. e.g. alphas[0] for low inclusively
/// alphas[1] for high exclusively and alphas[2] for step.
void genTemplates(ParamRange alphas, ParamRange betas, ParamRange gammas, ParamRange scales,
    int *count, CvMat **mat){
  int num_alphas = (alphas.high_ - alphas.low_)/alphas.step_;
  int num_betas  = (betas.high_  - betas.low_ )/betas.step_;
  int num_gammas = (gammas.high_ - gammas.low_)/gammas.step_;
  int num_scales = (scales.high_ - scales.low_)/scales.step_;

  for (int alpha_i=0; alpha_i < num_alphas; alpha_i++) {
    for (int beta_i=0; beta_i < num_betas; beta_i++) {
      for (int gamma_i=0; gamma_i < num_gammas; gamma_i++) {
        for (int scale_i=0; scale_i < num_scales; scale_i++) {
          float alpha = alphas.low_ + alpha_i*alphas.step_;
          float beta  = betas.low_  + beta_i *betas.step_;
          float gamma = gammas.low_ + gamma_i*gammas.step_;
          float scale = scales.low_ + scale_i*scales.step_;

          printf("alpha=%f, beta=%f, gamma=%f, scale=%f\n", alpha, beta, gamma, scale);
        }
      }
    }
  }


}

int main() {
  ParamRange alphas = {0, CV_PI, CV_PI/10.};
  ParamRange betas  = {0, CV_PI, CV_PI/10.};
  ParamRange gammas = {0, CV_PI, CV_PI/10.};
  ParamRange scales = {0.1, 10.0, 0.2};
  int count;
  CvMat *mats;
  genTemplates(alphas, betas, gammas, scales, &count, &mats);
}
