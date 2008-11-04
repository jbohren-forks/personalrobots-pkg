#include <CvStereoCamModel.h>
#include <gtest/gtest.h>

// this shall go away, replace with rosconsole stuff (?)
#include <iostream>

static const double FX  =   7.290000e+02;
static const double FY  =   7.290000e+02;
static const double TX  =   4.381214e+004/7.290000e+02;

static const double CLX = 3.239809e+002;
static const double CRX = 3.239809e+002;
static const double CY  = 2.478744e+002;
static const double DISPUNITSCALE =.25;

/// testing the following member methods
/// getDeltaX, getDeltaU, getZ, getDisparity
bool testGetDeltaXAndGetDeltaU() {
  CvStereoCamModel camModel;
  camModel.setCameraParams(FX, FY, TX, CLX, CRX, CY, DISPUNITSCALE);

  for (int i=1; i<100/DISPUNITSCALE; i++) {
    double disp = i;
    double du = 25.;
    double dx = camModel.getDeltaX(du, disp);
    double dx0 = du * TX/(disp*DISPUNITSCALE - (CLX - CRX));
    double diff = 0.;
    if ( dx != dx0 ) {
      printf("getDeltaX() test failed, du=%f, raw disp=%f, dx=%f\n",
          du, disp, dx);
      return false;
    }
    double z  = camModel.getZ(disp);
    double z0 = FX*TX/(DISPUNITSCALE*disp - (CLX - CRX));
    diff = fabs(z-z0);
    if (diff > .1e-10) {
      printf("getZ() test failed, disp = %f, z=%e, z0=%e, diff=%e\n",
          disp, z, z0, diff);
      return false;
    }
    // and convert it back
    double du0 = camModel.getDeltaU(dx, z);
    diff = fabs(du-du0);
    if ( diff > .1e-10) {
      printf("getDeltaU() test failed, dx=%f, z=%f, du=%e, du0=%e, diff=%e\n",
          dx, z, du, du0, diff);
      return false;
    }
  }

  return true;
}

/// testing the following member methods
/// getDeltaY, getDeltaV, getZ, getDisparity
bool testGetDeltaYAndGetDeltaV() {
  bool status = true;
  CvStereoCamModel camModel;
  camModel.setCameraParams(FX, FY, TX, CLX, CRX, CY, DISPUNITSCALE);

  for (int i=1; i<100/DISPUNITSCALE; i++) {
    double disp = i;
    double dv = 25.;
    double dy = camModel.getDeltaY(dv, disp);
    double dy0 = dv * TX * FX /(disp*DISPUNITSCALE - (CLX - CRX))/FY;
    double diff = 0.;
    if ( dy != dy0 ) {
      printf("getDeltaX() test failed, dv=%f, raw disp=%f, dy=%f\n",
          dv, disp, dy);
      return false;
    }
    double z = camModel.getZ(disp);
    double z0 = FX*TX/(DISPUNITSCALE*disp - (CLX - CRX));
    diff = fabs(z-z0);
    if (diff > .1e-10) {
      printf("getZ() test failed, disp = %f, z=%e, z0=%e, diff=%e\n",
          disp, z, z0, diff);
      return false;
    }
    // and convert it back
    double dv0 = camModel.getDeltaV(dy, z);
    diff = fabs(dv-dv0);
    if ( diff > .1e-10) {
      printf("getDeltaU() test failed, dx=%f, z=%f, du=%e, du0=%e, diff=%e\n",
          dy, z, dv, dv0, diff);
      return false;
    }
  }

  return true;
}

/// Testing cartToDisp and dispToCart
bool testSparsePointConversion() {
  CvStereoCamModel camModel;
  camModel.setCameraParams(FX, FY, TX, CLX, CRX, CY, DISPUNITSCALE);
  char filename[]= "test/pointsInCartesian.xml";

  CvMat* points_cart  = (CvMat*)cvLoad(filename);
  if (points_cart == NULL ) {
    printf("Cannot load data file: %s\n", filename);
    return false;
  }
  CvMat* points_cart0 = (CvMat*)cvCreateMat(points_cart->rows, points_cart->cols,
      CV_64FC1);
  CvMat* points_disp  = (CvMat*)cvCreateMat(points_cart->rows, points_cart->cols,
      CV_64FC1);
  CvMat* points_diff  = (CvMat*)cvCreateMat(points_cart->rows, points_cart->cols,
      CV_64FC1);

  camModel.cartToDisp(points_cart, points_disp);
  camModel.dispToCart(points_disp, points_cart0);

  cvAbsDiff(points_cart, points_cart0, points_diff);

  double min_val, max_val;

  cvMinMaxLoc(points_diff, &min_val, &max_val);

  if (max_val > .1e-10) {
    printf("dispToCart() or cartToDisp() failed: max diff = %f\n", max_val);
    return false;
  }

  cvReleaseMat(&points_cart);
  cvReleaseMat(&points_disp);
  cvReleaseMat(&points_diff);

  return true;
}


TEST(StereoUtils, DeltaX_DeltaU) {
  EXPECT_TRUE(testGetDeltaXAndGetDeltaU());
}

TEST(StereoUtils, DeltaY_DeltaV) {
  EXPECT_TRUE(testGetDeltaYAndGetDeltaV());
}

TEST(StereoUtils, SparsePointConversion) {
  EXPECT_TRUE(testSparsePointConversion());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

