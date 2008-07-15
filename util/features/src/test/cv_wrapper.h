/* Very simple, thin OpenCV image wrapper for now. To be replaced by the
   Google wrapper whenever we get it.

   Image takes ownership of the underlying IplImage.
*/

#ifndef OPENCV_CPP_WRAPPER_H
#define OPENCV_CPP_WRAPPER_H

#include <cv.h>
#include <boost/utility.hpp>
#include <string>
#include <iostream>

template< typename T >
class Image : boost::noncopyable
{
private:
    IplImage* m_img;
    bool m_own;

public:
    Image(IplImage* img, bool own = true)
        : m_img(img), m_own(own) {}
    
    ~Image()
    {
        if (m_own) cvReleaseImage(&m_img);
    }

    inline int rows() const
    {
        return m_img->height;
    }

    inline int cols() const
    {
        return m_img->width;
    }

    inline T& operator() (const int row, const int col)
    {
        return CV_IMAGE_ELEM(m_img, T, row, col);
    }

    inline const T& operator() (const int row, const int col) const
    {
        return CV_IMAGE_ELEM(m_img, T, row, col);
    }

    operator IplImage* ()
    {
        return m_img;
    }
    
    inline IplImage* get()
    {
        return m_img;
    }

    inline bool isNull() const
    {
        return !m_img;
    }
};

template< typename T >
struct ImageOutputCast
{
    typedef int type;
};
/*
template <> struct ImageOutputCast< uchar >
{
    typedef int type;
};
*/
// pjm: only supports single channel of integer type
template< typename T >
std::ostream& operator<< (std::ostream &os, Image<T> const& img)
{
    if ( img.isNull() )
        os << "Null image." << std::endl;
    else {
        os << "Image, " << img.rows() << 'x' << img.cols() << std::endl;
        for (int i = 0; i < img.rows(); ++i) {
            os << '\t';
            for (int j = 0; j < img.cols(); ++j) {
                os << (typename ImageOutputCast<T>::type) img(i,j) << '\t';
            }
            os << std::endl;
        }
    }
    
    return os;
}

#endif
