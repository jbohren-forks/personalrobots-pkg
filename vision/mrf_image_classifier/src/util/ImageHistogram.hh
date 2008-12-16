#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

#include "util/Coordinates.hh"

#include <vector>

/**
   @brief Class for image histogram calculation

   @tparam The image's storage type
*/
template <class Tp>
class ImageHistogram {
public:
  /**
     @brief A histogram bucket range
   */
  typedef pair<double,double> Range;

  struct Params {
    IplImage* image;
    Range histRange;
    int numBins;
  };

  static void histogram(const Params& hparam, 
			const coordList& coords, 
			double *bins) {
    histogram(hparam.image, hparam.histRange, coords,
	      hparam.numBins, bins);
  }

  /**
     @brief Computes an image histogram
     @param image The input image
     @param histRange The total range of the histogram
     @param coords A list of coordinates at which to accumulate values
     @param numBins The number of histogram bins
     @param bins An array of bins
     @attention Image must be single-channel.
     @attention Range excludes last value

     Divides histRange into numBins uniformly-sized bins 
     and computes the histogram of values at the specified 
     coordinates of the image.

     Histogram is normalized.
  */
  static void histogram(const IplImage* image,
			const Range& histRange,
			const coordList& coords,
			int numBins, 
			double* bins) 
  {
    double range = histRange.second - histRange.first;

    for (int ii = 0; ii < numBins; ii++)
      bins[ii] = 0;

    for (std::vector<coord2d>::const_iterator it = coords.coords.begin();
	 it != coords.coords.end();
	 it++) {
      coord2d coord = *it;
      double ival = 
	((Tp*)(image->imageData + coord.y * image->widthStep))[coord.x];
      int bin = ceil(numBins * (ival - histRange.first) / range) - 1;
      bins[bin]++;
    }

    double sum = 0;

    for (int ii = 0; ii < numBins; ii++)
      sum += bins[ii];

    for (int ii = 0; ii < numBins; ii++) 
      bins[ii] /= sum;
  }

private:
  //  histRange histRanges;
  //  double *bins;
};

#endif
