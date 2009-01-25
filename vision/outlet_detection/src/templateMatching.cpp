
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdio.h>

// The hole centers on the  500x500 template
// (208, 239)  (226, 239)  (272, 239) (290, 238)
// (208, 264)  (225, 264)  (271, 264) (288, 263)

const CvPoint Holes[8] = {
    cvPoint(208-150, 239-150), cvPoint(226-150, 239-150), cvPoint(272-150, 239-150), cvPoint(290-150, 238-150),
    cvPoint(208-150, 264-150), cvPoint(225-150, 264-150), cvPoint(271-150, 264-150), cvPoint(288-150, 263-150)
};

#define draw_cross( img, center, color, d )                                 \
  cvLine( img, cvPoint( center.x - d, center.y - d ),                \
  cvPoint( center.x + d, center.y + d ), color, 1, 0 ); \
  cvLine( img, cvPoint( center.x + d, center.y - d ),                \
  cvPoint( center.x - d, center.y + d ), color, 1, 0 )


class  ParamRange {
public:
  float low_;
  float high_;
  float step_;
};

/// Each of the following parameters is an array of 3. e.g. alphas[0] for low inclusively
/// alphas[1] for high inclusively and alphas[2] for step.
void genTemplates3D(ParamRange alphas, ParamRange betas, ParamRange gammas, ParamRange scales,
    int *count, CvMat **mat){
  int num_alphas = (alphas.high_ - alphas.low_)/alphas.step_ + 1;
  int num_betas  = (betas.high_  - betas.low_ )/betas.step_  + 1;
  int num_gammas = (gammas.high_ - gammas.low_)/gammas.step_ + 1;
  int num_scales = (scales.high_ - scales.low_)/scales.step_ + 1;

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
class AffineTf {
public:
  float rot_;
  float scale_x_;
  float scale_y_;
  float shear_;
  float transf_[9];
};
void genTemplatesAffine(CvPoint2D32f center, ParamRange rots,
    ParamRange scale_xs, ParamRange scale_ys,
    ParamRange shears, AffineTf **affine_tf, int *count)
{
  int num_rot      = (rots.high_ - rots.low_)/rots.step_ + 1;
  int num_scale_xs = (scale_xs.high_ - scale_xs.low_)/scale_xs.step_ + 1;
  int num_scale_ys = (scale_ys.high_ - scale_ys.low_)/scale_ys.step_ + 1;
  int num_shears   = (shears.high_   - shears.low_)  /shears.step_   + 1;
  printf("num of rot params=%d\n", num_rot);
  printf("num of scale_x params=%d\n", num_scale_xs);
  printf("num of scale_y params=%d\n", num_scale_ys);
  printf("num of shear params=%d\n", num_shears);
  *count = num_rot*num_scale_xs*num_scale_ys*num_shears;
  int num=0;
  *affine_tf = new AffineTf[*count];
  for (int rot_i=0; rot_i<num_rot; rot_i++ ) {
    for (int scale_x_i=0; scale_x_i < num_scale_xs; scale_x_i++) {
      for (int scale_y_i=0; scale_y_i < num_scale_ys; scale_y_i++) {
        for (int shear_i=0; shear_i < num_shears; shear_i++) {
          float rot = rots.low_ + rots.step_ * rot_i;
          float scale_x = scale_xs.low_ + scale_xs.step_ * scale_x_i;
          float scale_y = scale_ys.low_ + scale_ys.step_ * scale_y_i;
          float shear   = shears.low_   + shears.step_   * shear_i;
          float rot_data[9] = {
               cos(rot),  sin(rot), (1-cos(rot))*center.x - sin(rot)*center.y,
              -sin(rot),  cos(rot), sin(rot)*center.x + (1-cos(rot))*center.y,
                     0,         0, 1
          };
          CvMat rot_mat = cvMat(3, 3, CV_32FC1, rot_data);
//          CvMat rot_mat2x3 = cvMat(3, 3, CV_32FC1, rot_data);
//          cv2DRotationMatrix(center, (double)rot, 1.0, &rot_mat2x3);
          float scale_data[9] = {
              scale_x,       0., -center.x*scale_x + center.x,
                   0.,  scale_y, -center.y*scale_y + center.y,
                   0.,       0., 1.
          };

          CvMat scale_mat = cvMat(3, 3, CV_32FC1, scale_data);
          float shear_data[9] = {
              1.0,    shear,   -center.y*shear,
              0.0,       1.,   0,
              0.0,       0.,   1.
          };
          CvMat shear_mat = cvMat(3, 3, CV_32FC1, shear_data);
          //          AffineTf& aft = (*affine_tf)[((rot_i*num_scale_xs+scale_x_i)*num_scale_ys +scale_y_i)*num_shears + shear_i];
          AffineTf& aft = (*affine_tf)[num];
          num++;
          aft.rot_     = rot;
          aft.scale_x_ = scale_x;
          aft.scale_y_ = scale_y;
          aft.shear_   = shear;
          CvMat mat = cvMat(3, 3, CV_32FC1, aft.transf_);
          cvSetIdentity(&mat);
          cvMatMul(&rot_mat,   &mat, &mat);
          cvMatMul(&scale_mat, &mat, &mat);
          cvMatMul(&shear_mat, &mat, &mat);
        }
      }
    }
  }
}

void affineWarps(IplImage* image, IplImage** warped_images,
    AffineTf* affineTfs, int count) {
  int flags=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS;
  CvScalar fillval=cvScalarAll(0);
  for (int i=0; i<count; i++) {
    CvMat mat = cvMat(2, 3, CV_32FC1, affineTfs[i].transf_);
    cvWarpAffine( image, warped_images[i], &mat, flags, fillval);
  }
}

int main(int argc, char** argv) {
  ParamRange alphas = {0, 1, 2}; //{-CV_PI/4, CV_PI/4, CV_PI/4.};
  ParamRange scale_xs = {1., 1., 0.4}; //{0.8, 1.2, 0.4};
  ParamRange scale_ys = {1., 1., 0.4}; //{0.8, 1.2, 0.4};
  ParamRange shears   = {0., 0., 1.0}; //{-0.4, 0.4, 1.0};
  int count;
  AffineTf *affineTfs;

  IplImage* tmpl_image = cvLoadImage(argv[1]);
  IplImage* test_image = cvLoadImage(argv[2]);
  CvSize sz = cvGetSize(tmpl_image);
  CvPoint2D32f center = cvPoint2D32f(sz.width/2., sz.height/2.);
  CvSize center_sz = cvSize(200.,200.);
  genTemplatesAffine(center, alphas, scale_xs, scale_ys, shears, &affineTfs, &count);
  IplImage* warped_images[count];
  IplImage* warped_images_center[count];

  for (int i=0; i<count; i++) {
    warped_images[i] = cvCreateImage(cvGetSize(tmpl_image), tmpl_image->depth, tmpl_image->nChannels);
    warped_images_center[i] = cvCreateImageHeader(center_sz, tmpl_image->depth, tmpl_image->nChannels);

  }
  affineWarps(tmpl_image, warped_images, affineTfs, count);
  cvNamedWindow("input");
  cvShowImage("input", tmpl_image);
  CvRect center_rect = cvRect(center.x -center_sz.width/2.0, center.y - center_sz.height/2,
        center_sz.width, center_sz.height);
  for (int i=0; i<count; i++) {
    char win_label[255];
    sprintf(win_label, "%5.2f, %5.2f, %5.2f, %5.2f", affineTfs[i].rot_, affineTfs[i].scale_x_,
        affineTfs[i].scale_y_, affineTfs[i].shear_);
    cvNamedWindow(win_label);
    CvMat submat;
    cvGetSubRect( warped_images[i], &submat, center_rect );
    cvGetImage(&submat, warped_images_center[i]);
    cvShowImage(win_label, warped_images_center[i]);
  }

  // testing
  CvSize test_image_sz = cvGetSize(test_image);
  float result_data[(test_image_sz.width-center_sz.width+1)*(test_image_sz.height-center_sz.height+1)];
  CvMat result = cvMat(
      test_image_sz.height - center_sz.height + 1,
      test_image_sz.width  - center_sz.width  + 1,
      CV_32FC1,
      result_data
  );
  int method = CV_TM_CCORR_NORMED;
  double min_val;
  double max_val;
  CvPoint min_loc;
  CvPoint max_loc;
  for (int i=0; i<count; i++) {
    char win_label[255];
    sprintf(win_label, "test %5.2f, %5.2f, %5.2f, %5.2f",
        affineTfs[i].rot_, affineTfs[i].scale_x_,
        affineTfs[i].scale_y_, affineTfs[i].shear_);
    cvNamedWindow(win_label);
    cvMatchTemplate( test_image, warped_images_center[i],
        &result, method);
    cvMinMaxLoc( &result, &min_val, &max_val, &min_loc, &max_loc);
    printf("min_loc %d, %d, max_loc %d, %d\n", min_loc.x, min_loc.y, max_loc.x, max_loc.y);
    cvShowImage(win_label, &result);
  }

  // mark the holes
  CvPoint mark;
  CvPoint best_loc = max_loc;
  mark.x = best_loc.x + 100;
  mark.y = best_loc.y + 100;
  CvScalar color = CV_RGB(0, 255, 0);
  CvScalar red   = CV_RGB(255, 0, 0);
  draw_cross(test_image, mark, color, 3);

  for (int i=0; i<8; i++) {
    mark.x = best_loc.x + Holes[i].x;
    mark.y = best_loc.y + Holes[i].y;
    printf("mark on %d, %d\n", mark.x, mark.y);
    draw_cross(test_image, mark, color, 3);
  }

  cvNamedWindow("test image");
  cvShowImage("test image", test_image);
  cvWaitKey(0);
}
