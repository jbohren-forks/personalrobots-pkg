#ifndef _SPARSE_STEREO_H_
#define _SPARSE_STEREO_H_

#include <cvwimage.h>
#include <ost_stereolib.h>
#include <cstdlib>
#include <iostream>

namespace vision {

// TODO: allow memory reuse
class SparseStereoFrame
{
public:
  SparseStereoFrame(const cv::WImage<uchar>& left,
                    const cv::WImage<uchar>& right)
    : W(left.Width()), H(left.Height()),
      lgrad( aligned_malloc(W*H) ),
      rgrad( aligned_malloc(W*H) )
  {
    using std::cerr; // needed by ost_do_prefilter macro...
    using std::endl;
    
    // Compute left/right gradient images
    uchar* buffer = aligned_malloc(W*H);
    ost_do_prefilter(left.ImageData(), lgrad, W, H, FTZERO, buffer);
    ost_do_prefilter(right.ImageData(), rgrad, W, H, FTZERO, buffer);
    free(buffer);
  }

  ~SparseStereoFrame()
  {
    free(lgrad);
    free(rgrad);
  }

  double lookupDisparity(uint x, uint y)
  {
    // Grab 16x16 block around (x,y)
    int x_tl = x - 7, y_tl = y - 7;
    uchar refpat[256];
    for (size_t i = 0; i < 16; ++i)
      memcpy(refpat + 16*i, lgrad + x_tl + (y_tl + i)*W, 16);

    // Find disparity value
    int d = ost_do_stereo_sparse(refpat, rgrad, x, y, W, H, FTZERO,
                                 DLEN, TFILTER_THRESH, UFILTER_THRESH);
    return (double)d / 16.0;
  }

private:
  static const uchar FTZERO = 31;
  static const int DLEN = 64;
  static const int TFILTER_THRESH = 10;
  static const int UFILTER_THRESH = 15;

  uchar* aligned_malloc(size_t size)
  {
    void* ptr;
    if (posix_memalign(&ptr, 16, size) == 0)
      return static_cast<uchar*>(ptr);
    return NULL;
  }
  
  int W, H;
  uchar *lgrad, *rgrad;
};

} // namespace vision

#endif
