#include <cv.h>
#include <cstdio>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s input.yml output.ini\n", argv[0]);
    return 0;
  }

  // Read camera matrix and distortion from YML file
  CvFileStorage* fs = cvOpenFileStorage(argv[1], 0, CV_STORAGE_READ);
  assert(fs);
  CvMat* K = (CvMat*)cvReadByName(fs, 0, "camera_matrix");
  CvMat* D = (CvMat*)cvReadByName(fs, 0, "distortion_coefficients");
  int width = cvReadIntByName(fs, 0, "image_width");
  int height = cvReadIntByName(fs, 0, "image_height");
  cvReleaseFileStorage(&fs);

  // Write to ini file
  FILE* out = fopen(argv[2], "w");
  fprintf(out, "# Prosilica camera intrinsics\n\n");
  fprintf(out, "[image]\n\n");
  fprintf(out, "width\n%d\n\n", width);
  fprintf(out, "height\n%d\n\n", height);
  
  fprintf(out, "[prosilica]\n\n");
  fprintf(out, "camera matrix\n");
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      fprintf(out, "%.5f ", K->data.db[3*i+j]);
    }
    fprintf(out, "\n");
  }
  
  fprintf(out, "\ndistortion\n");
  for (int i = 0; i < D->cols; ++i)
    fprintf(out, "%.5f ", D->data.db[i]);
  for (int i = D->cols; i < 5; ++i)
    fprintf(out, "%.5f ", 0.0);
  
  fprintf(out, "\n\nrectification\n");
  fprintf(out, "1.00000 0.00000 0.00000\n");
  fprintf(out, "0.00000 1.00000 0.00000\n");
  fprintf(out, "0.00000 0.00000 1.00000\n");

  fprintf(out, "\nprojection\n");
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      fprintf(out, "%.5f ", K->data.db[3*i+j]);
    }
    fprintf(out, "0.00000\n");
  }

  // Free memory
  cvReleaseMat(&K);
  cvReleaseMat(&D);
  
  return 0;
}
