/*
 * KeypointDescriptors.cpp
 *
 *  Created on: Oct 6, 2008
 *      Author: jdchen
 */
//#include "VisOdom.h"
#include "KeypointDescriptors.h"
using namespace cv::willow;

#include <opencv/cv.h>

#include "ost_stereolib.h"

#include <boost/foreach.hpp>
using namespace boost;

#define DEBUG 0

float KeypointTemplateDescriptor::compare(const KeypointTemplateDescriptor& kpd) const {
  // use cvTemplateMatch
  float _res[1];
  CvMat res = cvMat(1, 1, CV_32FC1, _res);
  CvMat templ0 = cvMat(mSize.width, mSize.height, CV_8UC1, mData);
  CvMat templ1 = cvMat(mSize.width, mSize.height, CV_8UC1, kpd.mData);
  cvMatchTemplate(&templ0, &templ1, &res, mMatchMethod );
  // return a "distance" as smaller the better
#if 1
  switch (mMatchMethod) {
  case CV_TM_CCORR:
  case CV_TM_CCORR_NORMED:
  case CV_TM_CCOEFF:
  case CV_TM_CCOEFF_NORMED:
    // larger is better. inverse it
    if (_res[0]==0)
      return FLT_MAX;
    else {
      return 1.0/_res[0];
    }
  }
#endif
  return _res[0];
}

float KeypointSADDescriptor::compare(const KeypointSADDescriptor& kpd) const {
  uint8_t* gradmap0 = mData;
  uint8_t* gradmap1 = kpd.mData;
  // loop thru each row
  int32_t sad = 0; // sum of absolute difference
  int len = mSize.width*mSize.height;
  for (int r=0; r<len; r++) {
    int32_t diff = *gradmap0 - *gradmap1;
    gradmap0++;
    gradmap1++;

    if (diff>0) {
      sad += diff;
    } else {
      sad -= diff;
    }
  }
#if DEBUG==1
  {
    CvMat m0 = cvMat(mSize.width, mSize.height, CV_8UC1, mData);
    CvMat m1 = cvMat(mSize.width, mSize.height, CV_8UC1, kpd.mData);
    int sad0 = (int)(cvNorm( &m0, &m1, CV_L1)+.5);
    if (sad0 != sad) {
      std::cerr << "Diff with cvNorm(): "<< sad0 <<","<<sad<<std::endl;
    }
  }
#endif
  return (float)sad;
}

float KeypointTemplateDescriptor::compare(const KeypointDescriptor& kpd) const {
  const KeypointTemplateDescriptor* _kpd = (KeypointTemplateDescriptor *)&kpd;
  return compare(*_kpd);
}
float KeypointSADDescriptor::compare(const KeypointDescriptor& kpd) const {
  const KeypointSADDescriptor* _kpd = (KeypointSADDescriptor*)&kpd;
  return compare(*_kpd);
}

void KeypointSADDescriptor::constructDescriptors(
    /// input image
    const uint8_t* img,
    int width,
    int height,
    /// The list of keypoints
    Keypoints& keypoints,
    /// buffer used by this function. Same size as img
    uint8_t* bufImg1,
    /// buffer used by this function. Same size as img
    uint8_t* bufImg2
) {
  const int FTZero = 31;
  const CvSize descriptorSize = cvSize(16,16);
  uint8_t* featureImg = bufImg1;
  // compute the gradient image
  ost_do_prefilter(img, featureImg, width, height, FTZero, bufImg2);

  int dWidth  = descriptorSize.width;
  int dHeight = descriptorSize.height;

  // offset of the (0, 0) corner to the key point. For a 16x16 patch
  // the key point shall be at (7, 7)
  int offset_x = dWidth /2 - 1;
  int offset_y = dHeight/2 - 1;

  // XKERN, YKERN  is full size of the kernel and presumably odd numbers.
  int xmargin = offset_x + XKERN/2;
  int ymargin = offset_y + YKERN/2;
  // extract the descriptor patches.
  // The key point shall be at (7,7) of the patch, for a 16x16 patch
  BOOST_FOREACH(Keypoint& kp, keypoints) {
    int kpx = (int)(kp.x + .5);
    int kpy = (int)(kp.y + .5);
    if (kpx < xmargin || kpy < ymargin || kpx > width-xmargin-2 || kpy > height-ymargin-2){
      // kernel out of bound, skip this keypoint
      delete kp.desc;
      kp.desc=NULL;
    } else {

      // make a copy of the patch around this key point and call it the
      // descriptor of it
      KeypointSADDescriptor* desc = new KeypointSADDescriptor(descriptorSize);
      delete kp.desc;
      kp.desc = desc;

      int x0 = kpx - offset_x;
      int y0 = kpy - offset_y;
      uint8_t* pDescRow = desc->mData;
      const uint8_t* pImgPatchRow = &(featureImg[y0*width+x0]);

      // loop thru each row
      for (int i=0; i<dHeight; i++) {
        // copy a row
        memcpy(pDescRow, pImgPatchRow, sizeof(uint8_t)*dWidth);
        // update the points for next row
        pDescRow     += dWidth;
        pImgPatchRow += width;
      }
    }
  }
}

void KeypointTemplateDescriptor::constructDescriptors(
    /// input image
    const uint8_t* img,
    int width,
    int height,
    /// The list of keypoints
    Keypoints& keypoints
) {
  const CvSize descriptorSize = cvSize(DefWidth,DefHeight);

  int dWidth  = descriptorSize.width;
  int dHeight = descriptorSize.height;

  // offset of the (0, 0) corner to the key point. For a 16x16 patch
  // the key point shall be at (7, 7)
  int offset_x = dWidth /2 - 1;
  int offset_y = dHeight/2 - 1;

  // TODO: check if XKERN is half size or full size of the kernel
  int xmargin = offset_x;
  int ymargin = offset_y;
    // extract the descriptor patches
  BOOST_FOREACH(Keypoint& kp, keypoints) {
    int kpx = (int)(kp.x + .5);
    int kpy = (int)(kp.y + .5);
    if (kpx < xmargin || kpy < ymargin || kpx > width-xmargin-2 || kpy > height-ymargin-2){
      // kernel out of bound, skip this keypoint
      delete kp.desc;
      kp.desc=NULL;
    } else {

      // make a copy of the patch around this key point and call it the
      // descriptor of it
      KeypointTemplateDescriptor* desc = new KeypointTemplateDescriptor(descriptorSize);
      delete kp.desc;
      kp.desc = desc;

      int x0 = kpx - offset_x;
      int y0 = kpy - offset_y;
      uint8_t* pDescRow = desc->mData;
      const uint8_t* pImgPatchRow = &(img[y0*width+x0]);

      // loop thru each row
      for (int i=0; i<dHeight; i++) {
        // copy a row
        memcpy(pDescRow, pImgPatchRow, sizeof(uint8_t)*dWidth);
        // update the pointers for next row
        pDescRow     += dWidth;
        pImgPatchRow += width;
      }
    }
  }
}
