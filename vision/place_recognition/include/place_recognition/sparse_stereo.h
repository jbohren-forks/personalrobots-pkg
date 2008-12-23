#ifndef _SPARSE_STEREO_H_
#define _SPARSE_STEREO_H_

#include <boost/scoped_array.hpp>
#include <cvwimage.h>
#include <ost_stereolib.h>
#include <iostream>

namespace vision {

// TODO: aligned allocation of gradients, buffer
class SparseStereoFrame
{
public:
  SparseStereoFrame(const cv::WImage<uchar>& left,
                    const cv::WImage<uchar>& right)
    : W(left.Width()), H(left.Height()),
      lgrad( new uchar[W*H] ),
      rgrad( new uchar[W*H] )
  {
    using std::cerr; // needed by ost_do_prefilter macro...
    using std::endl;
    
    // Compute left/right gradient images
    boost::scoped_array<uchar> buffer( new uchar[W*H] );
    ost_do_prefilter(left.ImageData(), lgrad.get(), W, H, FTZERO, buffer.get());
    ost_do_prefilter(right.ImageData(), rgrad.get(), W, H, FTZERO, buffer.get());
  }

  double lookupDisparity(uint x, uint y)
  {
    // Grab 16x16 block around (x,y)
    int x_tl = x - 7, y_tl = y - 7;
    uchar refpat[256];
    for (size_t i = 0; i < 16; ++i)
      memcpy(refpat + 16*i, lgrad.get() + x_tl + (y_tl + i)*W, 16);

    // Find disparity value
    int d = ost_do_stereo_sparse(refpat, rgrad.get(), x, y, W, H, FTZERO,
                                 DLEN, TFILTER_THRESH, UFILTER_THRESH);
    return (double)d / 16.0;
  }

private:
  static const uchar FTZERO = 31;
  static const int DLEN = 64;
  static const int TFILTER_THRESH = 10;
  static const int UFILTER_THRESH = 15;

  int W, H;
  boost::scoped_array<uchar> lgrad, rgrad;
};

} // namespace vision

#endif
