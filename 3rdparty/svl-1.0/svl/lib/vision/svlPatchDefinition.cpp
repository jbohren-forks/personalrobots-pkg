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
** FILENAME:    svlPatchDefinition.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Sid Batra<sidbatra@cs.stanford.edu>
**
*****************************************************************************/

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>

#include "opencv/highgui.h"

#include "svlBase.h"
#include "svlVision.h"

#include "svlPatchDefinition.h"

using namespace std;

// svlPatchDefinition class --------------------------------------------------

svlPatchDefinition:: svlPatchDefinition() :
    _template(NULL), _validChannel(0)
{
    _validRect = cvRect(0, 0, 0, 0);
}

svlPatchDefinition::svlPatchDefinition(XMLNode& node) :
    _template(NULL), _validChannel(0)
{
	
    _validRect = cvRect(0, 0, 0, 0);
    //read(node);
}

svlPatchDefinition::svlPatchDefinition(const IplImage *t, const CvRect& rect, int channel)
{
    if (t != NULL) {
        _template = cvCloneImage(t);
    } else {
        _template = NULL;
    }
    _validRect.x = rect.x;
    _validRect.y = rect.y;
    _validRect.width = rect.width;
    _validRect.height = rect.height;
    _validChannel = channel;
}

svlPatchDefinition::~svlPatchDefinition()
{
    if (_template != NULL) {
        cvReleaseImage(&_template);
    }
}

// constuction helper functions
svlSmartPointer<svlPatchDefinition> svlPatchDefinition::createPatchDefinition(svlPatchDefinitionType t)
{
    svlSmartPointer<svlPatchDefinition> p;
    switch (t) {
    case SVL_INTENSITY_PATCH:
        p = svlSmartPointer<svlPatchDefinition>(new svlIntensityPatchDefinition());
        break;
    case SVL_DEPTH_PATCH:
        p = svlSmartPointer<svlPatchDefinition>(new svlDepthPatchDefinition());
        break;
    }

    return p;
}

svlSmartPointer<svlPatchDefinition> svlPatchDefinition::createPatchDefinition(XMLNode &node)
{
    assert(!node.isEmpty());
    svlSmartPointer<svlPatchDefinition> p;

    const char *t = node.getAttribute("type");
    assert(t != NULL);

    if (!strcasecmp(t, "intensity")) {
        p = svlSmartPointer<svlPatchDefinition>(new svlIntensityPatchDefinition(node));
    } else if (!strcasecmp(t, "depth")) 
	{ 
		
        p = svlSmartPointer<svlPatchDefinition>(new svlDepthPatchDefinition(node));
    } else {
        SVL_LOG(SVL_LOG_FATAL, "unknown patch type " << t);
    }

    return p;
}


bool svlPatchDefinition::read(XMLNode &node)
{		
	
    assert(!node.isEmpty());
    if (_template != NULL) {
        cvReleaseImage(&_template);
    }

    // read valid region
    if (node.getAttribute("validRect")) {
        vector<int> v;
        parseString<int>(node.getAttribute("validRect"), v);
        assert(v.size() == 4);
        _validRect = cvRect(v[0], v[1], v[2], v[3]);
    } else {
        _validRect = cvRect(0, 0, 0, 0);
    }

    // read channel
    if (node.getAttribute("channel")) {
        _validChannel = atoi(node.getAttribute("channel"));
    } else {
        _validChannel = 0;
    }

    // read patch size
    int w = 0;
    int h = 0;
    if (node.getAttribute("size")) {
        vector<int> v;
        parseString<int>(node.getAttribute("size"), v);
        assert(v.size() == 2);
        w = v[0]; h = v[1];
    }

    // read patch data
    if ((w > 0) && (h > 0)) {
        _template = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
        vector<int> v;
        parseString(node.getText(), v);
        assert(v.size() == (unsigned)(w * h));
        unsigned n = 0;
        for (unsigned y = 0; y < (unsigned)_template->height; y++) {
            char *p = &_template->imageData[y * _template->widthStep];
            for (unsigned x = 0; x < (unsigned)_template->width; x++) {
                p[x] = (char)v[n++];
            }
        }
    }
	
    return true;
}


bool svlPatchDefinition::write(ostream& os)
{
    assert(_template != NULL);

    os << "  <PatchDefinition type=\"";
    switch (this->patchType()) 
	{
    case SVL_INTENSITY_PATCH:
        os << "intensity";
        break;
    case SVL_DEPTH_PATCH:
        os << "depth";
        break;
    default:
        SVL_LOG(SVL_LOG_FATAL, "unknown patch type");
    }
    os << "\"\n"
       << "    validRect=\"" << _validRect.x << " " << _validRect.y << " "
       << _validRect.width << " " << _validRect.height << "\"\n"
       << "    channel=\"" << _validChannel << "\"\n"
       << "    size=\"" << _template->width << " " << _template->height << "\">\n";
    
    for (unsigned y = 0; y < (unsigned)_template->height; y++) 
	{
        os << "   ";
        for (unsigned x = 0; x < (unsigned)_template->width; x++) 
		{
            if (_template->depth == IPL_DEPTH_8U) 
			{
                os << " " << (int)CV_IMAGE_ELEM(_template, unsigned char, y, x);
            } 
			else if (_template->depth == IPL_DEPTH_32F) 
			{
                os << " " << CV_IMAGE_ELEM(_template, float, y, x);
            }
			else
			{
                SVL_LOG(SVL_LOG_FATAL, "unrecognized template type");
            }
        }        
        os << "\n";
    }
    
    os << "  </PatchDefinition>\n";

    return true;
}

IplImage *svlPatchDefinition::responseImage(const std::vector<IplImage *>& images) const
{
    assert(images.size() > (unsigned)_validChannel);
    return responseImage(images[_validChannel]);
}

double svlPatchDefinition::patchValue(const IplImage *responseImage, const CvPoint& location)
{
#if 0
    assert((location.x + _validRect.x + _validRect.width <= responseImage->width) &&
        (location.y + _validRect.y + _validRect.height <= responseImage->height));
#endif

    float maxScore = -numeric_limits<float>::max();

    float *p = &CV_IMAGE_ELEM(responseImage, float, location.y + _validRect.y,
        location.x + _validRect.x);

    size_t inc = responseImage->widthStep / sizeof(float);
    for (int dy = 0; dy < _validRect.height; dy++) {
        for (int dx = 0; dx < _validRect.width; dx++) {
            if (p[dx] > maxScore) {
                maxScore = p[dx];
            }
        }        
        p += inc;
    }

    return (double)maxScore;
}

vector<double> svlPatchDefinition::patchValues(IplImage *image,
    const vector<CvPoint>& locations)
{
    assert(false);
    return vector<double>();
}

vector<double> svlPatchDefinition::patchValues(const vector<IplImage *>& images,
    const vector<CvPoint>& locations)
{
    assert(images.size() > (unsigned)_validChannel);
    return patchValues(images[_validChannel], locations);
}


// svlIntensityPatchDefinition class --------------------------------------------------

svlIntensityPatchDefinition::svlIntensityPatchDefinition() :
    svlPatchDefinition()
{
    // do nothing
}

svlIntensityPatchDefinition::svlIntensityPatchDefinition(XMLNode& node) :
    svlPatchDefinition(node)
{
    // do nothing
	read(node);
}

svlIntensityPatchDefinition::svlIntensityPatchDefinition(const IplImage *t, const CvRect& rect, int channel) :
    svlPatchDefinition(t, rect, channel)
{
    // do nothing

}

svlIntensityPatchDefinition::~svlIntensityPatchDefinition()
{
    // do nothing
}


bool svlIntensityPatchDefinition::read(XMLNode &node)
{		
	
    assert(!node.isEmpty());
    if (_template != NULL) {
        cvReleaseImage(&_template);
    }

    // read valid region
    if (node.getAttribute("validRect")) {
        vector<int> v;
        parseString<int>(node.getAttribute("validRect"), v);
        assert(v.size() == 4);
        _validRect = cvRect(v[0], v[1], v[2], v[3]);
    } else {
        _validRect = cvRect(0, 0, 0, 0);
    }

    // read channel
    if (node.getAttribute("channel")) {
        _validChannel = atoi(node.getAttribute("channel"));
    } else {
        _validChannel = 0;
    }

    // read patch size
    int w = 0;
    int h = 0;
    if (node.getAttribute("size")) {
        vector<int> v;
        parseString<int>(node.getAttribute("size"), v);
        assert(v.size() == 2);
        w = v[0]; h = v[1];
    }

    // read patch data
    if ((w > 0) && (h > 0)) {
        _template = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
        vector<int> v;
        parseString(node.getText(), v);
        assert(v.size() == (unsigned)(w * h));
        unsigned n = 0;
        for (unsigned y = 0; y < (unsigned)_template->height; y++) {
            char *p = &_template->imageData[y * _template->widthStep];
            for (unsigned x = 0; x < (unsigned)_template->width; x++) {
                p[x] = (char)v[n++];
            }
        }
    }
	
    return true;
}


IplImage *svlIntensityPatchDefinition::responseImage(const IplImage *image) const
{
    static int handle = svlCodeProfiler::getHandle("svlIntensityPatchDefinition::responseImage");
    svlCodeProfiler::tic(handle);

    assert((_template != NULL) && (image != NULL));

    IplImage *response = cvCreateImage(cvSize(image->width - _template->width + 1,
	    image->height - _template->height + 1), IPL_DEPTH_32F, 1);
    cvMatchTemplate(image, _template, response, CV_TM_CCOEFF_NORMED);
    
    svlCodeProfiler::toc(handle);
    return response;
}

IplImage *svlIntensityPatchDefinition::responseImage(const IplImage *image, 
    const IplImage *imageSum, const IplImage *imageSumSq) const
{
    static int handle = svlCodeProfiler::getHandle("svlIntensityPatchDefinition::responseImage");
    svlCodeProfiler::tic(handle);

    assert((_template != NULL) && (image != NULL) &&
        (imageSum != NULL) && (imageSumSq != NULL));

    IplImage *response = cvCreateImage(cvSize(image->width - _template->width + 1,
	    image->height - _template->height + 1), IPL_DEPTH_32F, 1);

    //assert(image->depth == IPL_DEPTH_8U);
    //assert(imageSum->depth == IPL_DEPTH_32S);
    //assert(imageSumSq->depth == IPL_DEPTH_64F);

    float patchSum = 0.0;
    float patchNorm = 0.0;
    unsigned char *pp = (unsigned char *)_template->imageData;
    for (int y = 0; y < _template->height; y++) {
        for (int x = 0; x < _template->width; x++) {
            patchSum += (float)pp[x];
            patchNorm += (float)(pp[x] * pp[x]);
        }
        pp += _template->widthStep;
    }
    float N = (float)(_template->width * _template->height);
    float patchMean = patchSum / N;
    patchNorm -= patchSum * patchMean;

    //int numMultiplications = 0;
    for (int y = 0; y < image->height - _template->height + 1; y++) {
        const unsigned char *p = &CV_IMAGE_ELEM(image, unsigned char, y, 0);
        const int *ps = &CV_IMAGE_ELEM(imageSum, int, y, 0);
        const int *ps2 = &CV_IMAGE_ELEM(imageSum, int, y + _template->height, 0);
        const double *psSq = &CV_IMAGE_ELEM(imageSumSq, double, y, 0);
        const double *psSq2 = &CV_IMAGE_ELEM(imageSumSq, double, y + _template->height, 0);
        float *q = &CV_IMAGE_ELEM(response, float, y, 0);
        for (int x = 0; x < image->width - _template->width + 1; x++, p++) {
            float windowSum = (float)(ps[x] + ps2[x + _template->width] - 
                ps[x + _template->width] - ps2[x]);
            float windowNorm = (float)(psSq[x] + psSq2[x + _template->width] - 
                psSq[x + _template->width] - psSq2[x]);
            const unsigned char *pp = (const unsigned char *)_template->imageData;
            const unsigned char *pw = p;
	    unsigned int convolutionSum = 0;
            for (int v = 0; v < _template->height; v++) {
                for (int u = 0; u < _template->width; u++) {
                    convolutionSum += pp[u] * pw[u];
                    //numMultiplications += 1;
                }
                pp += _template->widthStep;
                pw += image->widthStep;
            }

            windowNorm -= windowSum * windowSum / N;
            *q = (float)convolutionSum - patchMean * windowSum;
            *q /= sqrt(patchNorm * windowNorm);
            q++;
        }
    }

    //cerr << numMultiplications << " multiplications" << endl;
    svlCodeProfiler::toc(handle);
    return response;
}

IplImage *svlIntensityPatchDefinition::responseImage(vector<const IplImage *>& images, 
    vector<const IplImage *>& imageSums, vector<const IplImage *>& imageSumSqs) const
{
    assert((images.size() > (unsigned)_validChannel) &&
	(imageSums.size() > (unsigned)_validChannel) &&
	(imageSumSqs.size() > (unsigned)_validChannel));

    return responseImage(images[_validChannel], imageSums[_validChannel],
	imageSumSqs[_validChannel]);
}

vector<double> svlIntensityPatchDefinition::patchValues(IplImage *image,
    const vector<CvPoint>& locations)
{		
    assert(image->depth == IPL_DEPTH_8U);


    // compute patch statistics
    float patchSum = 0.0;
    float patchNorm = 0.0;
    unsigned char *pp = (unsigned char *)_template->imageData;
    for (int y = 0; y < _template->height; y++) {
        for (int x = 0; x < _template->width; x++) {
            patchSum += (float)pp[x];
            patchNorm += (float)pp[x] * (float)pp[x];
        }
        pp += _template->widthStep;
    }
    float N = (float)(_template->width * _template->height);
    float patchMean = patchSum / N;
    patchNorm -= patchSum * patchMean;

    // precompute these in patch dictionary
    vector<double> values;
    values.reserve(locations.size());

    for (vector<CvPoint>::const_iterator it = locations.begin(); it != locations.end(); ++it) 
	{
        float maxVal = -numeric_limits<float>::max();
        for (int dy = _validRect.y; dy < _validRect.y + _validRect.height; dy++) {
            for (int dx = _validRect.x; dx < _validRect.x + _validRect.width; dx++) {
                const unsigned char *pw = &CV_IMAGE_ELEM(image, unsigned char, it->y + dy, it->x + dx);
                const unsigned char *pp = (const unsigned char *)_template->imageData;
                unsigned int windowSum = 0;
                unsigned int windowNorm = 0;
                unsigned int val = 0;
                for (int v = 0; v < _template->height; v++) {
                    for (int u = 0; u < _template->width; u++) {
                        windowSum += pw[u];
                        windowNorm += pw[u] * pw[u];
                        val += pp[u] * pw[u];
                    }
                    pp += _template->widthStep;
                    pw += image->widthStep;
                }

                float fWindowNorm = (float)windowNorm - (float)(windowSum * windowSum) / N;
                float fVal = (float)val - patchMean * (float)windowSum;
                fVal /= sqrt(patchNorm * fWindowNorm);
                if (fVal > maxVal) {
                    maxVal = fVal;
                }
            }
        }
        values.push_back((double)maxVal);
    }

    return values;
}

// svlDepthPatchDefinition class --------------------------------------------------

// initialize static data members
int svlDepthPatchDefinition::_maxDistance = 3;
int svlDepthPatchDefinition::_totalBins = 100;
CvSize svlDepthPatchDefinition::_windowSize;

svlDepthPatchDefinition::svlDepthPatchDefinition() :
    svlPatchDefinition()
{
    // do nothing
}

svlDepthPatchDefinition::svlDepthPatchDefinition(XMLNode& node) :
    svlPatchDefinition(node)
{
    // do nothing
	read(node);
}

svlDepthPatchDefinition::svlDepthPatchDefinition(const IplImage *t, const CvRect& rect, int channel) :
    svlPatchDefinition(t, rect, channel)
{
    // do nothing
}

svlDepthPatchDefinition::~svlDepthPatchDefinition()
{
    // do nothing
}

bool svlDepthPatchDefinition::read(XMLNode& node)
{	
	
    assert(!node.isEmpty());
	
    if (_template != NULL) {
        cvReleaseImage(&_template);
    }

    // read valid region
    if (node.getAttribute("validRect")) {
        vector<int> v;
        parseString<int>(node.getAttribute("validRect"), v);
        assert(v.size() == 4);
        _validRect = cvRect(v[0], v[1], v[2], v[3]);
    } else {
        _validRect = cvRect(0, 0, 0, 0);
    }

    // read channel
    if (node.getAttribute("channel")) {
        _validChannel = atoi(node.getAttribute("channel"));
    } else {
        _validChannel = 0;
    }

    // read patch size
    int w = 0;
    int h = 0;
    if (node.getAttribute("size")) {
        vector<int> v;
        parseString<int>(node.getAttribute("size"), v);
        assert(v.size() == 2);
        w = v[0]; h = v[1];
    }

    // read patch data
    if ((w > 0) && (h > 0)) {
        _template = cvCreateImage(cvSize(w, h), IPL_DEPTH_32F, 1);
        vector<float> v;
        parseString(node.getText(), v);
        assert(v.size() == (unsigned)(w * h));
        unsigned n = 0;
        for (unsigned y = 0; y < (unsigned)_template->height; y++) {
            float *p = &CV_IMAGE_ELEM(_template, float, y, 0);
            for (unsigned x = 0; x < (unsigned)_template->width; x++) {
                p[x] = v[n++];
            }
        }
    }
	
    return true;
}


vector<double> svlDepthPatchDefinition::patchValues(IplImage *image, const vector<CvPoint>& windows)
{	
	vector<double> f(windows.size());
    for (unsigned j = 0; j < windows.size(); j++) 
	{
        //Compute response image for the patch
        IplImage *response = responseImage(svlPatchDictionary::_normalizedDepthPatches[j]);


        //int handle = svlCodeProfiler::getHandle("svlDepthPatchDefinition::patchValue");
        
        //svlCodeProfiler::tic(handle);
   	

        f[j] = patchValue(response, cvPoint(0,0));
		
        
        //delete temp;
        cvReleaseImage(&response);
        //svlCodeProfiler::toc(handle);
	}
		
		return f;
            
	}
	
    
	//Iterate over each patch in the dictionary
	//for( int i=0 ; i<_entries.size() ; i++)
	//{
	//	if( (int)_entries[i]->_validChannel == DEPTH_MAP_IMAGE)
	//	{
	//		svlDepthPatchDefinition *temp = static_cast<svlDepthPatchDefinition*>(_entries[i]);

	//		//Compute response image for the patch
	//		IplImage *responseImage = temp->responseImage(image);

	//		int handle = svlCodeProfiler::getHandle("svlDepthPatchDefinition::patchValue");
 //           
	//		svlCodeProfiler::tic(handle);
 //           
	//		for (unsigned j = 0; j < windows.size(); j++) 
	//		{
 //               v[j][i] = temp->patchValue(responseImage, windows[j]);
 //           }
 //           
	//		svlCodeProfiler::toc(handle);
 //           cvReleaseImage(&responseImage);
	//	}
	//}



//Takes the depth values of the template patch and computes a depth histogram
int** svlDepthPatchDefinition::computeDepthHistogram(const IplImage* patchTemplate , CvRect region)
{
	float averageDepth = 0.0;

	//Compute average depth of the template patch
	for( int i=region.y ; i< region.y + region.height ; i++ )
	{
		for( int j=region.x ; j< region.x + region.width ; j++)
		{			
			averageDepth += CV_IMAGE_ELEM(patchTemplate, float, i, j);
		}
	}	

		
	averageDepth /= region.height * region.width;

	//averageDepth = CV_IMAGE_ELEM(patchTemplate,float,region.y + region.height /2 , region.x + region.width /2);

	
	//printf("\n AD = %f", averageDepth );

	//Init the histogram
	int **histogram = new int*[32];

	for( int i=0 ; i<32 ; i++)
	{
		histogram[i] = new int[32];
	}

	//Compute average depth of the template patch
	for( int i=region.y ; i< region.y + region.height ; i++ )
	{
		for( int j=region.x ; j< region.x + region.width ; j++)
		{
			float patchValue = CV_IMAGE_ELEM(patchTemplate, float, i, j);
			//float normalizedDepth =  (patchValue - averageDepth) + svlDepthPatchDefinition::maxDistance;
			float normalizedDepth =  (patchValue - averageDepth) * 1000;

			histogram[i-region.y][j-region.x] = (int)normalizedDepth;
			
			//int index = ( (normalizedDepth * svlDepthPatchDefinition::totalBins) / ( 2.0 * svlDepthPatchDefinition::maxDistance )) ;

			//printf("\n %f %f %d",patchValue,normalizedDepth,index);
	
			//Check out of bound conditions
			//if( index < 0 )
			//	index = 0;
			//else if( index >= svlDepthPatchDefinition::totalBins )
			//	index = svlDepthPatchDefinition::totalBins -1;
		
			//histogram[index]++;

			//char gg;
			//scanf("%c",&gg);
		
		}
	}

	
	//printf("\nHistorgam ");
	//for( int i=0 ; i<svlDepthPatchDefinition::totalBins ; i++)
	//{
	//	printf(" %d " , histogram[i]);
	//}

	
	//char gg;
	//scanf("%c",&gg);
	
	//for( int i=0 ; i<3 ; i++)
	//{
	//	smoothHistogram(histogram,svlDepthPatchDefinition::totalBins);
		//}
	

	return histogram;

}

//Smooth histogram with a kernel of [1/3 1/3 1/3]
void svlDepthPatchDefinition::smoothHistogram(int *histogram, int totalLength)
{
   
	//Apply smoothing kernel
   for (int i = 1; i < totalLength - 1 ; i++)
   {
		histogram[i] = (histogram[i-1] + histogram[i] + histogram[i+1] ) / 3;  
   }

   //Handle first and last entry seperately
   histogram[0] = ( histogram[0] + histogram[1] ) / 2;
   histogram[totalLength-1] = ( histogram[totalLength-2] + histogram[totalLength-1] ) / 2;
}





//Computes the earth movers distances between the two given integer arrays (histograms)
int svlDepthPatchDefinition::computeEMD(int *a , int *b , int totalLength)
{
	float *af = new float[totalLength];
	float *bf = new float[totalLength];

	//Convert histograms to float types to use opencv emd function
	for( int i=0 ; i<totalLength ; i++)
	{
		af[i] = (float)a[i];
		bf[i] = (float)b[i];
	}

	float dist = cvCalcEMD(af,totalLength,bf,totalLength,0,CV_DIST_L1);

	delete []af;
	delete []bf;

	return (int)dist;
}


//Computes the euclidean distances between the two given integer arrays (histograms)
int svlDepthPatchDefinition::computeEuclideanDistance(int *a , int *b , int totalLength)
{

	float totalDistance = 0;

	for( int i=0 ; i<totalLength ; i++)
	{
		totalDistance += (a[i] - b[i]) * (a[i] - b[i]);
	}

	return (int)sqrt(totalDistance);
}

IplImage *svlDepthPatchDefinition::responseImage(const IplImage *image) const
{
	
//	int handle = svlCodeProfiler::getHandle("svlDepthPatchDefinition::responseImage");
 //   svlCodeProfiler::tic(handle);

    assert((_template != NULL) && (image != NULL));
    

    IplImage *response = cvCreateImage(cvSize(image->width - _template->width + 1,image->height - _template->height + 1)
		, IPL_DEPTH_32F, 1);
	
	CvMat* tplate = cvCreateMat(_template->height,_template->width,CV_32F);

	for( int y=0 ; y<_template->height ; y++)
	{
		for( int x=0 ; x<_template->width ; x++)
		{
			cvmSet(tplate,y,x,CV_IMAGE_ELEM(_template,float,y,x));
		}
	}
	//cvGetSubRect(_template,tplate,cvRect(0,0,_template->width,_template->height));

	cvMatchTemplate(image,tplate,response,CV_TM_CCORR_NORMED);	
	////cvMatchTemplate(image,tplate,response,CV_TM_CCORR);	

	cvReleaseMat(&tplate);
	
	////Compute histogram for the patch template
	//_histogram = computeDepthHistogram(_template , cvRect(0,0,_template->width,_template->height) );


	//for( int i= _template->height / 2; i< response->height - _template->height / 2 ; i++)
	//{
	//	for( int j= _template->width / 2 ; j< response->width - _template->width / 2 ; j++)
	//	{
	//		int *patchHistogram = computeDepthHistogram(image, cvRect(j -_template->width / 2
	//			,i - _template->height / 2,_template->width,_template->height));

	//		int distance = computeEuclideanDistance(_histogram,patchHistogram,svlDepthPatchDefinition::totalBins);
	//		
	//		//printf("\n%d %d %d %d",i,j,response->height,response->width);
	//		//cvSet2D(response,i,j,cvScalar(distance));
	//		
	//		CV_IMAGE_ELEM(response,float,i,j) = distance;	

	//		delete []patchHistogram;
	//
	//	}

	//}	

	//delete []_histogram;

	
//	svlCodeProfiler::toc(handle);
  
    return response;
}


//Change to finding the minimum entry in response image because the lesser the distance the better
//double svlDepthPatchDefinition::patchValue(const IplImage *responseImage,  const CvPoint& location)
//{
//#if 0
//    assert((location.x + _validRect.x + _validRect.width <= responseImage->width) &&
//	   (location.y + _validRect.y + _validRect.height <= responseImage->height));
//#endif
//
//    float minScore = numeric_limits<float>::max();
//
//    float *p = &CV_IMAGE_ELEM(responseImage, float, location.y + _validRect.y,
//        location.x + _validRect.x);
//
//    size_t inc = responseImage->widthStep / sizeof(float);
//
//    for (int dy = 0; dy < _validRect.height; dy++) 
//	{
//		for (int dx = 0; dx < _validRect.width; dx++) 
//		{
//		    if (p[dx] < minScore) 
//			{
//				minScore = p[dx];
//			}
//		}
//
//		p += inc;
//    }
//
//    return (double)minScore;
//}



//*******************************************************************************************************************

