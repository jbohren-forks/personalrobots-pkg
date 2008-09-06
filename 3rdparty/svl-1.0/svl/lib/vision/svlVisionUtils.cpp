/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2008, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    svlVisionUtils.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <list>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVisionUtils.h"

using namespace std;

// toString() routines.
string toString(const CvPoint& p)
{
    std::stringstream s;
    s << "(" << p.x << ", " << p.y << ")";
    return s.str();
}

string toString(const CvRect& r)
{
    std::stringstream s;
    s << "<" << r.x << ", " << r.y << ", " << r.width << ", " << r.height << ">";
    return s.str();
}

string toString(const CvSize& sz)
{
    std::stringstream s;
    s << "<" << sz.width << ", " << sz.height << ">";
    return s.str();
}

string toString(const CvMat& m)
{
    std::stringstream s;
    s << "[";
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            s << "\t" << cvmGet(&m, i, j);
        }
        if (i != m.rows - 1) {
            s << "\n";
        } else {
            s << "\t]";
        }
    }
    return s.str();
}

string toString(const IplImage& img)
{
    std::stringstream s;
    s << img.width << "x" << img.height
	  << " (" << img.depth << "-bit, "
	  << img.nChannels << "-channel)";
    return s.str();
}

//Function assumes, first entry in the vector is the intensity image and second image is depth map
//images[0] = Intenstiy Image
//images[1] = Depth Map Image
string toString(const vector<IplImage*> imgs)
{
    std::stringstream s;
    s << imgs[0]->width << "x" << imgs[0]->height
	  << " (" << imgs[0]->depth << "-bit, "
	  << imgs[0]->nChannels << "-channel)";

	//Output some details about the depth map image when availible

    return s.str();
}

void dump(const IplImage& image, int maxWidth, int maxHeight)
{
    if (maxWidth <= 0) {
	maxWidth = numeric_limits<int>::max();
    }

    if (maxHeight <= 0) {
	maxHeight = numeric_limits<int>::max();
    }

    for (int y = 0; y < image.height; y++) {
	if (y > maxHeight) {
	    cout << "..." << endl;
	    break;
	}
	for (int x = 0; x < image.width; x++) {
	    if (x > 0) {
		cout << " ";
	    }
	    if (x > maxWidth) {
		cout << "...";
		break;
	    }
	    CvScalar v = cvGet2D(&image, y, x);
	    if (image.nChannels == 1) {
		cout << v.val[0];
	    } else {		
		cout << "(";
		for (int i = 0; i < image.nChannels; i++) {
		    if (i > 0) cout << ", ";
		    cout << v.val[i];
		}
		cout << ")";
	    }
	}
	cout << endl;
    }
}

void dump(const CvMat& matrix, int maxColumns, int maxRows)
{
    if (maxColumns <= 0) {
	maxColumns = numeric_limits<int>::max();
    }

    if (maxRows <= 0) {
	maxRows = numeric_limits<int>::max();
    }

    for (int y = 0; y < matrix.rows; y++) {
	if (y > maxRows) {
	    cout << "..." << endl;
	    break;
	}
	for (int x = 0; x < matrix.cols; x++) {
	    if (x > 0) {
		cout << " ";
	    }
	    if (x > maxColumns) {
		cout << "...";
		break;
	    }
	    cout << CV_MAT_ELEM(matrix, float, y, x);
	}
	cout << endl;
    }
}

bool readMatrix(CvMat *matrix, istream& is)
{
    assert((matrix != NULL) && (!is.fail()));

    if (CV_MAT_TYPE(matrix->type) == CV_32SC1) {
	int v;
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		is >> v;
		CV_MAT_ELEM(*matrix, int, y, x) = v;
	    }
	}
    } else {
	float v;
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		is >> v;
		cvmSet(matrix, y, x, v);
	    }
	}    
    }

    return true;
}

bool readMatrix(CvMat *matrix, const char *filename)
{
    assert(filename != NULL);

    ifstream ifs(filename);
    if (ifs.fail()) {
        SVL_LOG(SVL_LOG_WARNING, "could not open " << filename);
	return false;
    }
    assert(!ifs.fail());
    readMatrix(matrix, ifs);
    ifs.close();

    return true;
}

void writeMatrix(const CvMat *matrix, ostream& os)
{
    assert((matrix != NULL) && (!os.fail()));

    if (CV_MAT_TYPE(matrix->type) == CV_32FC1) {
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		os << " " << CV_MAT_ELEM(*matrix, float, y, x);
	    }
	    os << "\n";
	}
    } else if (CV_MAT_TYPE(matrix->type) == CV_32SC1) {
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		os << " " << CV_MAT_ELEM(*matrix, int, y, x);
	    }
	    os << "\n";
	}
    } else {
	for (int y = 0; y < matrix->rows; y++) {
	    for (int x = 0; x < matrix->cols; x++) {
		os << " " << cvmGet(matrix, y, x);
	    }
	    os << "\n";
	}
    }
}

void writeMatrix(const CvMat *matrix, const char *filename)
{
    ofstream ofs(filename);
    assert(!ofs.fail());
    writeMatrix(matrix, ofs);
    ofs.close();
}

IplImage* readMatrixAsIplImage(const char *filename , int width , int height)
{
    ifstream ifs(filename);
    assert(!ifs.fail());
    
    //Used to read values in the file
    float v;

    //Create IplImage to hold matrix
    IplImage* matrix = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
    
    for (int i = 0; i < matrix->height; i++) 
	{
	for (int j = 0; j < matrix->width; j++) 
	{
	    ifs >> v;
	    CV_IMAGE_ELEM(matrix, float, i, j) = v;
	}	
    }

    return matrix;
}

void writeMatrixAsIplImage(IplImage *matrix, const char *filename , int x , int y , int width , int height)
{
    ofstream ofs(filename);
    assert(!ofs.fail());

    for (int i = y; i < y + height; i++) {
	for (int j = x; j < x + width; j++) {
	    float v = CV_IMAGE_ELEM(matrix,float,i,j);	
	    ofs << v;
	    
	    if (j != matrix->width - 1)
		ofs << " ";
	}
	if (i != matrix->height - 1)
	    ofs << "\n";
    } 
    
    ofs.close();
}

void writeMatrixAsIplImage(IplImage *matrix, const char *filename)
{
    ofstream ofs(filename);
    assert(!ofs.fail());

    for (int i = 0; i < (int)(matrix->height); i++) {
	for (int j = 0; j < (int)(matrix->width); j++) {
	    float v = CV_IMAGE_ELEM(matrix,float,i,j);	
	    ofs << v;
	    
	    if (j != matrix->width - 1)
		ofs << " ";
	    }
	if( i != matrix->height - 1 )
	    ofs<<"\n";
    }
    
    ofs.close();
}

void scaleToRange(CvArr *array, double minValue, double maxValue)
{
    assert((array != NULL) && (minValue <= maxValue));

    double m, M;
    cvMinMaxLoc(array, &m, &M);
    if (m != M) {
	cvScale(array, array, (maxValue - minValue) / (M - m), 
            minValue - m * (maxValue - minValue) / (M - m));
    }
}

IplImage *combineImages(const vector<IplImage *>& images, int rows, int cols)
{
    // check sizes
    assert(!images.empty());
    if ((rows <= 0) && (cols <= 0)) {
	rows = cols = (int)ceil(sqrt((float)images.size()));
    } else if (rows <= 0) {
	rows = (int)ceil((float)images.size() / cols);
    } else if (cols <= 0) {
	cols = (int)ceil((float)images.size() / rows);
    } else {
	assert((int)images.size() <= rows * cols);
    }

    int maxWidth = 0;
    int maxHeight = 0;
    for (unsigned i = 0; i < images.size(); i++) {
	if (images[i] == NULL) continue;
	if (images[i]->width > maxWidth) {
	    maxWidth = images[i]->width;
	}
	if (images[i]->height > maxHeight) {
	    maxHeight = images[i]->height;
	}
    }
    assert((maxWidth > 0) && (maxHeight > 0));

    IplImage *outImg = cvCreateImage(cvSize(maxWidth * cols, maxHeight * rows),
	images.front()->depth, images.front()->nChannels);
    cvZero(outImg);
    
    for (unsigned i = 0; i < images.size(); i++) {
	if (images[i] == NULL) continue;
	int x = (i % cols) * maxWidth;
	int y = ((int)(i / cols)) * maxHeight;
	cvSetImageROI(outImg, cvRect(x, y, maxWidth, maxHeight));
	cvCopyImage(images[i], outImg);
	cvResetImageROI(outImg);
    }

    return outImg;
}

IplImage *greyImage(const IplImage *color)
{
    IplImage *grey = cvCreateImage(cvGetSize(color), IPL_DEPTH_8U, 1);
    assert(grey);
    cvCvtColor(color, grey, CV_BGR2GRAY);
    return grey;
}

IplImage *colorImage(const IplImage *grey)
{
    IplImage *color = cvCreateImage(cvGetSize(grey), IPL_DEPTH_8U, 3);
    assert(color);
    if (grey->depth != IPL_DEPTH_8U) {
	IplImage *tmpImage = cvCreateImage(cvGetSize(grey), IPL_DEPTH_8U, grey->nChannels);
	cvConvert(grey, tmpImage);
	cvCvtColor(tmpImage, color, CV_GRAY2BGR);
	cvReleaseImage(&tmpImage);
    } else {
	cvCvtColor(grey, color, CV_GRAY2BGR);
    }

    return color;
}

// Resize image inplace
void resizeInPlace(IplImage **image, int height, int width)
{
    IplImage *tmpImage = cvCreateImage(cvSize(width, height),
	(*image)->depth, (*image)->nChannels);
    cvResize(*image, tmpImage);
    cvReleaseImage(image);
    *image = tmpImage;
}

void resizeInPlace(CvMat **matrix, int rows, int cols)
{
    CvMat *tmpMatrix = cvCreateMat(rows, cols, (*matrix)->type);
    cvResize(*matrix, tmpMatrix);
    cvReleaseMat(matrix);
    *matrix = tmpMatrix;
}

IplImage *makeIplImage(const unsigned char *data, int width, int height, int channels)
{
    assert(data != NULL);
    IplImage *image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, channels);

    for (int y = 0; y < image->height; y++) {
#if 1
        int yy = y;
#else
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        int yy = image->height - y - 1;
#else
        int yy = y;
#endif
#endif
	for (int x = 0; x < image->width; x++) {
	    image->imageData[y * image->widthStep + 3 * x + 2] = data[3 * (yy * image->width + x) + 0];
	    image->imageData[y * image->widthStep + 3 * x + 1] = data[3 * (yy * image->width + x) + 1];
	    image->imageData[y * image->widthStep + 3 * x + 0] = data[3 * (yy * image->width + x) + 2];
	}
    }

    return image;
}

unsigned char *makeByteStream(IplImage *image)
{
    assert((image != NULL) && (image->nChannels == 3));
    unsigned char *data = new unsigned char[3 * image->width * image->height];

    for (int y = 0; y < image->height; y++) {
#if 1
        int yy = y;
#else
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        int yy = image->height - y - 1;
#else
        int yy = y;
#endif
#endif
	for (int x = 0; x < image->width; x++) {
	    data[3 * (yy * image->width + x) + 0] = image->imageData[y * image->widthStep + 3 * x + 2];
	    data[3 * (yy * image->width + x) + 1] = image->imageData[y * image->widthStep + 3 * x + 1];
	    data[3 * (yy * image->width + x) + 2] = image->imageData[y * image->widthStep + 3 * x + 0];
	}
    }
    
    return data;
}

// estimate normals using SVD around local neighbourhood of point
void estimatePointNormals(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ, int windowSize)
{
    int handle = svlCodeProfiler::getHandle("estimatePointNormals");
    svlCodeProfiler::tic(handle);

    assert((X != NULL) && (Y != NULL) && (Z != NULL));
    assert((nX != NULL) && (nY != NULL) && (nZ != NULL));

    int width = Z->width;
    int height = Z->height;

    assert((X->width == width) && (X->height == height));
    assert((Y->width == width) && (Y->height == height));
    assert((Z->width == width) && (Z->height == height));
    assert((nX->width == width) && (nX->height == height));
    assert((nY->width == width) && (nY->height == height));
    assert((nZ->width == width) && (nZ->height == height));

    // allocate temporary matrices
    CvMat* C = cvCreateMat(3,3,CV_32FC1); // covariance matrix
    CvMat* U = cvCreateMat(3,3,CV_32FC1); // left orthogonal matrix
    CvMat* S = cvCreateMat(3,3,CV_32FC1); // singular values
    assert((C != NULL) && (U != NULL) && (S != NULL));

    // estimate point normals by SVD
    float muX, muY, muZ;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            cvZero(C);
	    muX = 0.0; muY = 0.0; muZ = 0.0;
            int nPoints = 0;
            for (int v = MAX(y - windowSize, 0); v < MIN(y + windowSize, height); v++) {
                for (int u = MAX(x - windowSize, 0); u < MIN(x + windowSize, width); u++) {
		    float ptX = CV_MAT_ELEM(*X, float, v, u);
		    float ptY = CV_MAT_ELEM(*Y, float, v, u);
		    float ptZ = CV_MAT_ELEM(*Z, float, v, u);
                    muX += ptX; muY += ptY; muZ += ptZ;
                    nPoints += 1;
                    CV_MAT_ELEM(*C, float, 0, 0) += ptX * ptX;
                    CV_MAT_ELEM(*C, float, 0, 1) += ptX * ptY;
                    CV_MAT_ELEM(*C, float, 0, 2) += ptX * ptZ;
                    CV_MAT_ELEM(*C, float, 1, 1) += ptY * ptY;
                    CV_MAT_ELEM(*C, float, 1, 2) += ptY * ptZ;
                    CV_MAT_ELEM(*C, float, 2, 2) += ptZ * ptZ;
                }
            }

            CV_MAT_ELEM(*C, float, 0, 0) -= (muX * muX) / (float)nPoints;
            CV_MAT_ELEM(*C, float, 0, 1) -= (muX * muY) / (float)nPoints;
	    CV_MAT_ELEM(*C, float, 0, 2) -= (muX * muZ) / (float)nPoints;
            CV_MAT_ELEM(*C, float, 1, 1) -= (muY * muY) / (float)nPoints;
	    CV_MAT_ELEM(*C, float, 1, 2) -= (muY * muZ) / (float)nPoints;
	    CV_MAT_ELEM(*C, float, 2, 2) -= (muZ * muZ) / (float)nPoints;
            CV_MAT_ELEM(*C, float, 1, 0) = CV_MAT_ELEM(*C, float, 0, 1);
            CV_MAT_ELEM(*C, float, 2, 0) = CV_MAT_ELEM(*C, float, 0, 2);
            CV_MAT_ELEM(*C, float, 2, 1) = CV_MAT_ELEM(*C, float, 2, 1);

            cvSVD(C, S, U, NULL, CV_SVD_MODIFY_A | CV_SVD_U_T);

	    float dp = CV_MAT_ELEM(*U, float, 2, 0) * CV_MAT_ELEM(*X, float, y, x) +
		CV_MAT_ELEM(*U, float, 2, 1) * CV_MAT_ELEM(*Y, float, y, x) +
		CV_MAT_ELEM(*U, float, 2, 2) * CV_MAT_ELEM(*Z, float, y, x);
	    if (dp <= 0.0) {
		CV_MAT_ELEM(*nX, float, y, x) = CV_MAT_ELEM(*U, float, 2, 0);
		CV_MAT_ELEM(*nY, float, y, x) = CV_MAT_ELEM(*U, float, 2, 1);
		CV_MAT_ELEM(*nZ, float, y, x) = CV_MAT_ELEM(*U, float, 2, 2);
	    } else {
		CV_MAT_ELEM(*nX, float, y, x) = -CV_MAT_ELEM(*U, float, 2, 0);
		CV_MAT_ELEM(*nY, float, y, x) = -CV_MAT_ELEM(*U, float, 2, 1);
		CV_MAT_ELEM(*nZ, float, y, x) = -CV_MAT_ELEM(*U, float, 2, 2);
	    }
        }
    }
    
    // release temporary matrices
    cvReleaseMat(&S);
    cvReleaseMat(&U);
    cvReleaseMat(&C);

    svlCodeProfiler::toc(handle);
}

void estimatePointNormalsFast(const CvMat *X, const CvMat *Y, const CvMat *Z,
    CvMat *nX, CvMat *nY, CvMat *nZ)
{
    int handle = svlCodeProfiler::getHandle("estimatePointNormalsFast");
    svlCodeProfiler::tic(handle);

    assert((X != NULL) && (Y != NULL) && (Z != NULL));
    assert((nX != NULL) && (nY != NULL) && (nZ != NULL));

    int width = Z->width;
    int height = Z->height;

    assert((X->width == width) && (X->height == height));
    assert((Y->width == width) && (Y->height == height));
    assert((Z->width == width) && (Z->height == height));
    assert((nX->width == width) && (nX->height == height));
    assert((nY->width == width) && (nY->height == height));
    assert((nZ->width == width) && (nZ->height == height));

    // estimate point normals by orthogonality to neighbours
    float u1, u3, v2, v3;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
	    v2 = CV_MAT_ELEM(*Y, float, MIN(y + 1, height - 1), x) -
		CV_MAT_ELEM(*Y, float, MAX(y - 1, 0), x);
	    v3 = CV_MAT_ELEM(*Z, float, MIN(y + 1, height - 1), x) -
		CV_MAT_ELEM(*Z, float, MAX(y - 1, 0), x);
	    u1 = CV_MAT_ELEM(*Y, float, y, MIN(x + 1, width - 1)) -
		CV_MAT_ELEM(*Y, float, y, MAX(x - 1, 0));
	    u3 = CV_MAT_ELEM(*Z, float, y, MIN(x + 1, width - 1)) -
		CV_MAT_ELEM(*Z, float, y, MAX(x - 1, 0));

	    CV_MAT_ELEM(*nX, float, y, x) = v2 * u3;
	    CV_MAT_ELEM(*nY, float, y, x) = v3 * u1;
	    CV_MAT_ELEM(*nZ, float, y, x) = -v2 * u1;

	    float s = sqrt(CV_MAT_ELEM(*nX, float, y, x) * CV_MAT_ELEM(*nX, float, y, x) +
		CV_MAT_ELEM(*nY, float, y, x) * CV_MAT_ELEM(*nY, float, y, x) +
		CV_MAT_ELEM(*nZ, float, y, x) * CV_MAT_ELEM(*nZ, float, y, x));
	    if (s < 1.0e-12) {
		CV_MAT_ELEM(*nX, float, y, x) = 0.0;
		CV_MAT_ELEM(*nY, float, y, x) = 0.0;
		CV_MAT_ELEM(*nZ, float, y, x) = 1.0;		
	    } else {
		CV_MAT_ELEM(*nX, float, y, x) /= s;
		CV_MAT_ELEM(*nY, float, y, x) /= s;
		CV_MAT_ELEM(*nZ, float, y, x) /= s;
	    }

	    float dp = CV_MAT_ELEM(*nX, float, y, x) * CV_MAT_ELEM(*X, float, y, x) +
		CV_MAT_ELEM(*Y, float, y, x) * CV_MAT_ELEM(*Y, float, y, x) +
		CV_MAT_ELEM(*Z, float, y, x) * CV_MAT_ELEM(*Z, float, y, x);
	    if (dp > 0.0) {
		CV_MAT_ELEM(*nX, float, y, x) = -CV_MAT_ELEM(*nX, float, 2, 0);
		CV_MAT_ELEM(*nY, float, y, x) = -CV_MAT_ELEM(*nY, float, 2, 1);
		CV_MAT_ELEM(*nZ, float, y, x) = -CV_MAT_ELEM(*nZ, float, 2, 2);
	    }
        }
    }
    
    svlCodeProfiler::toc(handle);
}

float estimatePlane(const CvMat *X, const CvMat *Y, const CvMat *Z,
    const vector<CvPoint>& points, svlPoint3d &n, svlPoint3d &c)
{
    int handle = svlCodeProfiler::getHandle("estimatePlane");
    svlCodeProfiler::tic(handle);

    assert((X != NULL) && (Y != NULL) && (Z != NULL));
    
    n = svlPoint3d(0.0, 0.0, -1.0);
    c = svlPoint3d();
    
    if (points.empty()) 
    {
        return 0.0f;
    }

    int width = Z->width;
    int height = Z->height;

    assert((X->width == width) && (X->height == height));
    assert((Y->width == width) && (Y->height == height));
    assert((Z->width == width) && (Z->height == height));

#if 1
    // remove outliers
    vector<CvPoint> trimmedPoints(points);
    vector<double> s(trimmedPoints.size());
#if 0
    // X
    for (unsigned i = 0; i < trimmedPoints.size(); i++) {
        s[i] = CV_MAT_ELEM(*X, float, trimmedPoints[i].y,
            trimmedPoints[i].x);        
    }
    trimmedPoints = removeOutliers(trimmedPoints, s, 
        (int)(0.8 * trimmedPoints.size()));
    // Y
    s.resize(trimmedPoints.size());
    for (unsigned i = 0; i < trimmedPoints.size(); i++) {
        s[i] = CV_MAT_ELEM(*Y, float, trimmedPoints[i].y,
            trimmedPoints[i].x);
    }
    trimmedPoints = removeOutliers(trimmedPoints, s, 
        (int)(0.8 * trimmedPoints.size()));
#endif
    // Z
    s.resize(trimmedPoints.size());
    for (unsigned i = 0; i < trimmedPoints.size(); i++) 
    {
        s[i] = CV_MAT_ELEM(*Z, float, trimmedPoints[i].y,
            trimmedPoints[i].x);
    }
    
    trimmedPoints = removeOutliers(trimmedPoints, s, 
        (int)(0.5 * trimmedPoints.size()));
#else
    const vector<CvPoint> &trimmedPoints(points);
#endif

    // allocate temporary matrices
    CvMat* C = cvCreateMat(3,3,CV_32FC1); // covariance matrix
    CvMat* U = cvCreateMat(3,3,CV_32FC1); // left orthogonal matrix
    CvMat* S = cvCreateMat(3,3,CV_32FC1); // singular values
    assert((C != NULL) && (U != NULL) && (S != NULL));

    float muX, muY, muZ;
    float numPoints = (float)trimmedPoints.size();

    // estimate point normal by SVD
    cvZero(C);
    muX = 0.0; muY = 0.0; muZ = 0.0;
    for (unsigned i = 0; i < trimmedPoints.size(); i++) {
        int u = trimmedPoints[i].x;
        int v = trimmedPoints[i].y;
        float ptX = CV_MAT_ELEM(*X, float, v, u);
        float ptY = CV_MAT_ELEM(*Y, float, v, u);
        float ptZ = CV_MAT_ELEM(*Z, float, v, u);
        muX += ptX; muY += ptY; muZ += ptZ;
        CV_MAT_ELEM(*C, float, 0, 0) += ptX * ptX;
        CV_MAT_ELEM(*C, float, 0, 1) += ptX * ptY;
        CV_MAT_ELEM(*C, float, 0, 2) += ptX * ptZ;
        CV_MAT_ELEM(*C, float, 1, 1) += ptY * ptY;
        CV_MAT_ELEM(*C, float, 1, 2) += ptY * ptZ;
        CV_MAT_ELEM(*C, float, 2, 2) += ptZ * ptZ;
    }
    
    CV_MAT_ELEM(*C, float, 0, 0) -= (muX * muX) / numPoints;
    CV_MAT_ELEM(*C, float, 0, 1) -= (muX * muY) / numPoints;
    CV_MAT_ELEM(*C, float, 0, 2) -= (muX * muZ) / numPoints;
    CV_MAT_ELEM(*C, float, 1, 1) -= (muY * muY) / numPoints;
    CV_MAT_ELEM(*C, float, 1, 2) -= (muY * muZ) / numPoints;
    CV_MAT_ELEM(*C, float, 2, 2) -= (muZ * muZ) / numPoints;
    CV_MAT_ELEM(*C, float, 1, 0) = CV_MAT_ELEM(*C, float, 0, 1);
    CV_MAT_ELEM(*C, float, 2, 0) = CV_MAT_ELEM(*C, float, 0, 2);
    CV_MAT_ELEM(*C, float, 2, 1) = CV_MAT_ELEM(*C, float, 2, 1);

#if 1
    // add bias for fronto-parallel
    float gamma = 10.0f;
    CV_MAT_ELEM(*C, float, 0, 0) += gamma / numPoints;
    CV_MAT_ELEM(*C, float, 1, 1) += gamma / numPoints;
#endif
    
    cvSVD(C, S, U, NULL, CV_SVD_MODIFY_A | CV_SVD_U_T);
    float residual = CV_MAT_ELEM(*S, float, 2, 2);

    // normal
    float dp = CV_MAT_ELEM(*U, float, 2, 0) * muX +
        CV_MAT_ELEM(*U, float, 2, 1) * muY +
        CV_MAT_ELEM(*U, float, 2, 2) * muZ;
    if (dp <= 0.0) {
        n.x = CV_MAT_ELEM(*U, float, 2, 0);
        n.y = CV_MAT_ELEM(*U, float, 2, 1);
        n.z = CV_MAT_ELEM(*U, float, 2, 2);
    } else {
        n.x = -CV_MAT_ELEM(*U, float, 2, 0);
        n.y = -CV_MAT_ELEM(*U, float, 2, 1);
        n.z = -CV_MAT_ELEM(*U, float, 2, 2);
    }

    // centroid
    c = svlPoint3d(muX, muY, muZ) / numPoints;
    
    // release temporary matrices
    cvReleaseMat(&S);
    cvReleaseMat(&U);
    cvReleaseMat(&C);

    svlCodeProfiler::toc(handle);

#if 0
    // discard outliers and repeat
    residual = 0.0f;
    vector<CvPoint> reducedPoints;
    reducedPoints.reserve(points.size());
    double b = n.dot(c);
    for (unsigned i = 0; i < points.size(); i++) {
        int u = points[i].x;
        int v = points[i].y;
        svlPoint3d x(CV_MAT_ELEM(*X, float, v, u), 
            CV_MAT_ELEM(*Y, float, v, u),
            CV_MAT_ELEM(*Z, float, v, u));
        double r = n.dot(x) - b;
        if (r * r < 0.01) {
            reducedPoints.push_back(points[i]);
        }
        residual += r * r;
    }
    residual = sqrt(residual) / (float)points.size();

    if (reducedPoints.size() < points.size()) {
        SVL_LOG(SVL_LOG_VERBOSE, "Repeating estimatePlane() with " 
            << reducedPoints.size() << " points");
        residual = estimatePlane(X, Y, Z, reducedPoints, n, c);
    }
#endif

    return residual;
}

void svlClipRect(CvRect& r, int width, int height)
{
    if (r.x < 0) r.x = 0;
    if (r.y < 0) r.y = 0;
    if (r.x + r.width > width) r.width = width - r.x;
    if (r.y + r.height > height) r.height = height - r.y;
}

void svlClipRect(CvRect& r, const IplImage *img)
{
    assert(img != NULL);
    svlClipRect(r, img->width, img->height);
}

void svlRotatePointCloud(CvMat *X, CvMat *Y, CvMat *Z, const CvMat *R)
{
    assert((X != NULL) && (Y != NULL) && (Z != NULL) && (R != NULL));
    assert((X->cols == Y->cols) && (X->rows == Y->rows));
    assert((X->cols == Z->cols) && (X->rows == Z->rows));
    assert((R->rows == 3) && (R->cols == 3));

    CvMat *b = cvCreateMat(3, 1, CV_32FC1);
    for (int v = 0; v < X->rows; v++) {
        for (int u = 0; u < X->cols; u++) {
            CV_MAT_ELEM(*b, float, 0, 0) = CV_MAT_ELEM(*X, float, v, u);
            CV_MAT_ELEM(*b, float, 1, 0) = CV_MAT_ELEM(*Y, float, v, u);
            CV_MAT_ELEM(*b, float, 2, 0) = CV_MAT_ELEM(*Z, float, v, u);
            cvMatMul(R, b, b);
            CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*b, float, 0, 0);
            CV_MAT_ELEM(*Y, float, v, u) = CV_MAT_ELEM(*b, float, 1, 0);
            CV_MAT_ELEM(*Z, float, v, u) = CV_MAT_ELEM(*b, float, 2, 0);
        }
    }
}

void svlTranslatePointCloud(CvMat *X, CvMat *Y, CvMat *Z,
    double dx, double dy, double dz)
{
    cvAddS(X, cvScalar(dx), X);
    cvAddS(Y, cvScalar(dy), Y);
    cvAddS(Z, cvScalar(dz), Z);
}

void svlAddSquared(CvMat *src1, CvMat *src2, CvMat *dst, bool bNormalize)
{
    assert((src1 != NULL) && (src2 != NULL) && (dst != NULL));
    assert((src1->rows == src2->rows) && (src1->cols == src2->cols));
    assert((src1->rows == dst->rows) && (src1->cols == dst->cols));

    float maxValue = 0.0;
    for (int y = 0; y < dst->rows; y++) {
        for (int x = 0; x < dst->cols; x++) {
            CV_MAT_ELEM(*dst, float, y, x) =
                CV_MAT_ELEM(*src1, float, y, x) * CV_MAT_ELEM(*src1, float, y, x) +
                CV_MAT_ELEM(*src2, float, y, x) * CV_MAT_ELEM(*src2, float, y, x);
            if (CV_MAT_ELEM(*dst, float, y, x) > maxValue)
                maxValue = CV_MAT_ELEM(*dst, float, y, x);
        }
    }

    if (bNormalize && (maxValue != 0.0)) {
        for (int y = 0; y < dst->rows; y++) {
            for (int x = 0; x < dst->cols; x++) {
                CV_MAT_ELEM(*dst, float, y, x) /= maxValue;
            }
        }
    }
}

// Fill zero points with value from nearest-neighbour
void svlNearestNeighbourFill(CvMat *X, float emptyToken)
{
    assert(X != NULL);
    const int width = X->cols;
    const int height = X->rows;

    list<pair<int, int> > toBeFilled;
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            if (CV_MAT_ELEM(*X, float, v, u) == emptyToken)
                toBeFilled.push_back(make_pair(v, u));
        }
    }

    CvMat *lastX = cvCreateMat(height, width, CV_32FC1);
    while (!toBeFilled.empty()) {
        cvCopy(X, lastX);
        for (list<pair<int, int> >::iterator it = toBeFilled.begin(); it != toBeFilled.end(); ) {
            int v = it->first;
            int u = it->second;
            if (((v - 1) >= 0) && (CV_MAT_ELEM(*lastX, float, v - 1, u) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v - 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u - 1) >= 0) && (CV_MAT_ELEM(*lastX, float, v, u - 1) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v, u - 1);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((v + 1) < height) && (CV_MAT_ELEM(*lastX, float, v + 1, u) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v + 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u + 1) < width) && (CV_MAT_ELEM(*lastX, float, v, u + 1) != emptyToken)) {
                CV_MAT_ELEM(*X, float, v, u) = CV_MAT_ELEM(*lastX, float, v, u + 1);
                it = toBeFilled.erase(it);
                continue;
            }
            ++it;
        }
    }
    cvReleaseMat(&lastX);
}

void svlNearestNeighbourFill(IplImage *X, unsigned char emptyToken)
{
    assert((X != NULL) && (X->depth == IPL_DEPTH_8U));
    const int width = X->width;
    const int height = X->height;
    
    if (X->nChannels > 1) {
        IplImage *imgChannel = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
        for (int c = 0; c < X->nChannels; c++) {
            cvSetImageCOI(X, c + 1);
            cvCopyImage(X, imgChannel);
            svlNearestNeighbourFill(imgChannel, emptyToken);
            cvCopyImage(imgChannel, X);
        }
        cvSetImageCOI(X, 0);
        return;
    }


    list<pair<int, int> > toBeFilled;
	
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            if (CV_IMAGE_ELEM(X, unsigned char, v, u) == emptyToken)
                toBeFilled.push_back(make_pair(v, u));
        }
    }

    IplImage *lastX = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    while (!toBeFilled.empty()) {
        cvCopy(X, lastX);
        for (list<pair<int, int> >::iterator it = toBeFilled.begin(); it != toBeFilled.end(); ) {
            int v = it->first;
            int u = it->second;
            if (((v - 1) >= 0) && (CV_IMAGE_ELEM(lastX, unsigned char, v - 1, u) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v - 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u - 1) >= 0) && (CV_IMAGE_ELEM(lastX, unsigned char, v, u - 1) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v, u - 1);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((v + 1) < height) && (CV_IMAGE_ELEM(lastX, unsigned char, v + 1, u) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v + 1, u);
                it = toBeFilled.erase(it);
                continue;
            }
            if (((u + 1) < width) && (CV_IMAGE_ELEM(lastX, unsigned char, v, u + 1) != emptyToken)) {
                CV_IMAGE_ELEM(X, unsigned char, v, u) = CV_IMAGE_ELEM(lastX, unsigned char, v, u + 1);
                it = toBeFilled.erase(it);
                continue;
            }
            ++it;
        }
    }
    cvReleaseImage(&lastX);
}

// Returns true if p is inside polygon poly. The polygon is assumed to
// be closed,
bool svlInsidePolygon(const vector<CvPoint>& poly, const CvPoint& p)
{
    assert(poly.size() > 2);

    int i, j, counter = 0;
    for (i = 0, j = (int)poly.size() - 1; i < (int)poly.size(); j = i++) {
        if (((poly[i].y <= p.y) && (poly[j].y <= p.y)) ||
            ((poly[i].y > p.y) && (poly[j].y > p.y)) ||
            ((poly[i].x < p.x) && (poly[j].x < p.x))) {
            if (((p.y == poly[j].y) && (p.x == poly[j].x)) || 
                ((p.y == poly[i].y) && (poly[i].x <= p.x) && (p.x <= poly[j].x)) ||
                ((poly[j].x <= p.x) && (p.x <= poly[i].x)))
                return true;
            continue;
        }

        int dist = (p.y - poly[i].y)*(poly[j].x - poly[i].x) - 
            (p.x - poly[i].x)*(poly[j].y - poly[i].y);
        if (dist == 0)
            return true;
        if (poly[j].y < poly[i].y)
            dist = -dist;
        counter += (dist > 0);
    }

    return ((counter % 2) == 1);
}


