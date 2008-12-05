/*
 * KeypointDescriptors.h
 *
 *  Created on: Oct 6, 2008
 *      Author: jdchen
 */

#ifndef KEYPOINTDESCRIPTORS_H_
#define KEYPOINTDESCRIPTORS_H_

#include <opencv/cv.h>
#include "VisOdom.h"
#include <limits>

namespace cv { namespace willow {
/// a keypoint decriptor that is merely a patch of 8-bit gray image
class KeypointTemplateDescriptor: public KeypointDescriptor {
public:
  KeypointTemplateDescriptor(const CvSize& sz):
    mMatchMethod(CV_TM_CCORR_NORMED),mSize(sz){
    mData = new unsigned char[mSize.width*mSize.height];
  }
  virtual ~KeypointTemplateDescriptor(){delete [] mData;}
  float compare(const KeypointTemplateDescriptor& kpd) const;
  virtual float compare(const KeypointDescriptor& kpd) const;
  /// construct descriptors for a list of key points over an image
  static void constructDescriptors(
      /// input image
      const uint8_t* img,
      int width,
      int height,
      /// The list of keypoints
      Keypoints& keypoints
  );
  static const int DefWidth = 16;
  static const int DefHeight = 16;
  int mMatchMethod;
protected:
  CvSize mSize;
  unsigned char* mData;
};

class KeypointSADDescriptor: public KeypointDescriptor {
public:
  KeypointSADDescriptor(const CvSize& sz):mSize(sz){
    mData = new unsigned char[mSize.width*mSize.height];
    // make sure that unlikely event of overflow does not
    // happen in SAD computation in compare()
    assert(mSize.width*mSize.height*numeric_limits<unsigned char>::max() <
        numeric_limits<int>::max());
  }
  virtual ~KeypointSADDescriptor(){delete [] mData;}
  float compare(const KeypointSADDescriptor& kpd) const;
  virtual float compare(const KeypointDescriptor& kpd) const;
  static void constructDescriptors(
      /// input image.
      /// If compiled with -sse2 (and up) options, it would speed up
      /// computation to have this buffer aligned to 16 byte block.
      const uint8_t* img,
      int width,
      int height,
      /// The list of keypoints
      Keypoints& keypoints,
      /// buffer used by this function. Same size as img.
      /// If compiled with -sse2 (and up) options, it would speed up
      /// computation to have this buffer aligned to 16 byte block.
      uint8_t* bufImg1,
      /// buffer used by this function, of size (width+64)*4 or larger.
      /// No alignment requirement.
      uint8_t* bufImg2
  );
  static void computeDisparity(
      /// input image.
      /// If compiled with -sse2 (and up) options, it would speed up
      /// computation to have this buffer aligned to 16 byte block.
      const uint8_t* rightImg,
      int width,
      int height,
      /// The list of keypoints
      Keypoints& keypoints,
      /// buffer used by this function. Same size as img.
      /// If compiled with -sse2 (and up) options, it would speed up
      /// computation to have this buffer aligned to 16 byte block.
      uint8_t* bufImg1,
      /// buffer used by this function, of size (width+64)*4 or larger.
      /// No alignment requirement.
      uint8_t* bufImg2
  );
  static const int DefWidth  = 16;
  static const int DefHeight = 16;
protected:
  CvSize mSize;
  uint8_t* mData;
};

}
}


#endif /* KEYPOINTDESCRIPTORS_H_ */
