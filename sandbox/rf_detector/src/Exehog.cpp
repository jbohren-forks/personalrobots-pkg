/*********************************************************************
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Extracted from Opencv and Modified by Min Sun

#include <iostream>
#include "Exehog.h"
#include "cv.h"
#include "write.h"

using namespace cv;


size_t HogFast::getDescriptorSize() const
{
    CV_Assert(blockSize.width % cellSize.width == 0 &&
        blockSize.height % cellSize.height == 0);
    CV_Assert((winSize.width - blockSize.width) % blockStride.width == 0 &&
        (winSize.height - blockSize.height) % blockStride.height == 0 );
    return (size_t)nbins*
        (blockSize.width/cellSize.width)*
        (blockSize.height/cellSize.height)*
        ((winSize.width - blockSize.width)/blockStride.width + 1)*
        ((winSize.height - blockSize.height)/blockStride.height + 1);
}

double HogFast::getWinSigma() const
{
    return winSigma >= 0 ? winSigma : (blockSize.width + blockSize.height)/8.;
}

bool HogFast::checkDetectorSize() const
{
    size_t detectorSize = svmDetector.size(), descriptorSize = getDescriptorSize();
    return detectorSize == 0 ||
        detectorSize == descriptorSize ||
        detectorSize == descriptorSize + 1;
}

void HogFast::setSVMDetector(const Vector<float>& _svmDetector)
{
    svmDetector = _svmDetector;
    CV_Assert( checkDetectorSize() );
}

void HogFast::setRFDetector(std::string& RFmodel_filename, std::string& FgBgProb_filename)
{
    cout << "loading" << RFmodel_filename << endl;
    rf.RFLoad(RFmodel_filename);
    rf.SetLoadStatus(true);
    read2DFloatText( FgBgProb_filename, FgBgProb);
}

FileNode getFirstTopLevelNode(FileStorage& fs)
{
    FileNode root = fs.root();
    FileNodeIterator it = root.begin();
    return it != root.end() ? *it : FileNode();
}

bool HogFast::load(const String& filename, const String& objname)
{
    FileStorage fs(filename, FileStorage::READ);
    FileNode obj = !objname.empty() ? fs[objname] :
        getFirstTopLevelNode(fs);
    if( !obj.isMap() )
        return false;
    FileNodeIterator it = obj["winSize"].begin();
    it >> winSize.width >> winSize.height;
    it = obj["blockSize"].begin();
    it >> blockSize.width >> blockSize.height;
    it = obj["blockStride"].begin();
    it >> blockStride.width >> blockStride.height;
    it = obj["cellSize"].begin();
    it >> cellSize.width >> cellSize.height;
    obj["nbins"] >> nbins;
    obj["derivAperture"] >> derivAperture;
    obj["winSigma"] >> winSigma;
    obj["histogramNormType"] >> histogramNormType;
    obj["L2HysThreshold"] >> L2HysThreshold;
    obj["gammaCorrection"] >> gammaCorrection;

    FileNode vecNode = obj["SVMDetector"];
    if( vecNode.isSeq() )
    {
        vecNode >> svmDetector;
        CV_Assert(checkDetectorSize());
    }
    return true;
}

void HogFast::save(const String& filename, const String& objName) const
{
    FileStorage fs(filename, FileStorage::WRITE);
    fs << (!objName.empty() ? objName : FileStorage::getDefaultObjectName(filename)) << "{";

    fs  << "winSize" << winSize
        << "blockSize" << blockSize
        << "blockStride" << blockStride
        << "cellSize" << cellSize
        << "nbins" << nbins
        << "derivAperture" << derivAperture
        << "winSigma" << getWinSigma()
        << "histogramNormType" << histogramNormType
        << "L2HysThreshold" << L2HysThreshold
        << "gammaCorrection" << gammaCorrection;
    if( !svmDetector.empty() )
        fs << "SVMDetector" << "[:" << svmDetector << "]";
    fs << "}";
}

void HogFast::computeGradient_fbr(const vector<Mat>& fbr, Mat& grad, Mat& qangle,
                                    Size paddingTL, Size paddingBR) const
{
    Size gradsize(fbr[0].cols+ paddingTL.width + paddingBR.width,
        fbr[0].rows + paddingTL.height + paddingBR.height);
    cout << "fbe_channel" << fbr.size() << " "<< fbr[0].cols << " " << fbr[0].rows<<endl;
    cout << "gradsize" << gradsize.width << gradsize.height<<endl;

    Size wholeSize;
    Point roiofs;
    fbr[0].locateROI(wholeSize, roiofs);
    cout << "wholeSize" << wholeSize.width << " " <<wholeSize.height << endl;
    cout << "roiofs" << roiofs.x << " " <<roiofs.y << endl;

    vector<int> xmap( gradsize.width);
    vector<int> ymap( gradsize.height);

    const int borderType = (int)BORDER_REFLECT_101;

    for( int x = 0; x < gradsize.width ; x++ ){
        xmap[x] = borderInterpolate(x - paddingTL.width + roiofs.x,
                        wholeSize.width, borderType);
    }
    cout << endl;
    for( int y = 0; y < gradsize.height ; y++ ){
        ymap[y] = borderInterpolate(y - paddingTL.height + roiofs.y,
                        wholeSize.height, borderType);
    }
    cout << endl;

    grad.create(gradsize, CV_32FC2);  // <magnitude*(1-alpha), magnitude*alpha>
    qangle.create(gradsize, CV_8UC2); // [0..nbins-1] - quantized gradient orientation
    int i, x, y;
    cout << "nbins"<< nbins << endl;
    for( y = 0; y < gradsize.height; y++ )
    {
        float* gradPtr = (float*)grad.ptr(y);
        uchar* qanglePtr = (uchar*)qangle.ptr(y);
        vector<float*> fbrptr(nbins);
        for (i = 0; i< nbins;i++){
            fbrptr[i] = (float*)fbr[i].ptr(ymap[y]);
        }

        for( x = 0; x < gradsize.width; x++ )
        {
            qanglePtr[x*2] = (uchar)0;
            qanglePtr[x*2+1] = (uchar)0;
            gradPtr[x*2] = 0;
            gradPtr[x*2+1] = 0;
            for (i = 0; i< nbins;i++){
                if (gradPtr[x*2]< fbrptr[i][xmap[x]]){
                    gradPtr[x*2+1] = gradPtr[x*2];
                    qanglePtr[x*2+1] = qanglePtr[x*2];
                    gradPtr[x*2] = fbrptr[i][xmap[x]];
                    qanglePtr[x*2] = i;
                }else if(gradPtr[x*2+1]< fbrptr[i][xmap[x]]){
                    gradPtr[x*2+1] = fbrptr[i][xmap[x]];
                    qanglePtr[x*2+1] = i;
                }
            }
        }
    }
    cout << "out of computeGradient_fbr" << endl;
}

void HogFast::computeGradient(const Mat& img, Mat& grad, Mat& qangle,
                                    Size paddingTL, Size paddingBR) const
{
    CV_Assert( img.type() == CV_8U || img.type() == CV_8UC3 );

    Size gradsize(img.cols + paddingTL.width + paddingBR.width,
                  img.rows + paddingTL.height + paddingBR.height);
    grad.create(gradsize, CV_32FC2);  // <magnitude*(1-alpha), magnitude*alpha>
    qangle.create(gradsize, CV_8UC2); // [0..nbins-1] - quantized gradient orientation
    Size wholeSize;
    Point roiofs;
    img.locateROI(wholeSize, roiofs);

    int i, x, y;
    int cn = img.channels();

    Mat_<float> _lut(1, 256);
    const float* lut = &_lut(0,0);

    if( gammaCorrection )
        for( i = 0; i < 256; i++ )
            _lut(0,i) = std::sqrt((float)i);
    else
        for( i = 0; i < 256; i++ )
            _lut(0,i) = (float)i;

    AutoBuffer<int> mapbuf(gradsize.width + gradsize.height + 4);
    int* xmap = (int*)mapbuf + 1;
    int* ymap = xmap + gradsize.width + 2;

    const int borderType = (int)BORDER_REFLECT_101;

    for( x = -1; x < gradsize.width + 1; x++ )
        xmap[x] = borderInterpolate(x - paddingTL.width + roiofs.x,
                        wholeSize.width, borderType);
    for( y = -1; y < gradsize.height + 1; y++ )
        ymap[y] = borderInterpolate(y - paddingTL.height + roiofs.y,
                        wholeSize.height, borderType);

    // x- & y- derivatives for the whole row
    int width = gradsize.width;
    AutoBuffer<float> _dbuf(width*4);
    float* dbuf = _dbuf;
    Mat Dx(1, width, CV_32F, dbuf);
    Mat Dy(1, width, CV_32F, dbuf + width);
    Mat Mag(1, width, CV_32F, dbuf + width*2);
    Mat Angle(1, width, CV_32F, dbuf + width*3);

    int _nbins = nbins;
    float angleScale = (float)(_nbins/CV_PI);

    for( y = 0; y < gradsize.height; y++ )
    {
        const uchar* imgPtr = img.data + img.step*ymap[y];
        const uchar* prevPtr = img.data + img.step*ymap[y-1];
        const uchar* nextPtr = img.data + img.step*ymap[y+1];
        float* gradPtr = (float*)grad.ptr(y);
        uchar* qanglePtr = (uchar*)qangle.ptr(y);

        if( cn == 1 )
        {
            for( x = 0; x < width; x++ )
            {
                int x1 = xmap[x];
                dbuf[x] = (float)(lut[imgPtr[xmap[x+1]]] - lut[imgPtr[xmap[x-1]]]);
                dbuf[width + x] = (float)(lut[nextPtr[x1]] - lut[prevPtr[x1]]);
            }
        }
        else
        {
            for( x = 0; x < width; x++ )
            {
                int x1 = xmap[x]*3;
                const uchar* p2 = imgPtr + xmap[x+1]*3;
                const uchar* p0 = imgPtr + xmap[x-1]*3;
                float dx0, dy0, dx, dy, mag0, mag;

                dx0 = lut[p2[2]] - lut[p0[2]];
                dy0 = lut[nextPtr[x1+2]] - lut[prevPtr[x1+2]];
                mag0 = dx0*dx0 + dy0*dy0;

                dx = lut[p2[1]] - lut[p0[1]];
                dy = lut[nextPtr[x1+1]] - lut[prevPtr[x1+1]];
                mag = dx*dx + dy*dy;

                if( mag0 < mag )
                {
                    dx0 = dx;
                    dy0 = dy;
                    mag0 = mag;
                }

                dx = lut[p2[0]] - lut[p0[0]];
                dy = lut[nextPtr[x1]] - lut[prevPtr[x1]];
                mag = dx*dx + dy*dy;

                if( mag0 < mag )
                {
                    dx0 = dx;
                    dy0 = dy;
                    mag0 = mag;
                }

                dbuf[x] = dx0;
                dbuf[x+width] = dy0;
            }
        }

        cartToPolar( Dx, Dy, Mag, Angle, false );

        for( x = 0; x < width; x++ )
        {
            float mag = dbuf[x+width*2], angle = dbuf[x+width*3]*angleScale - 0.5f;
            int hidx = cvFloor(angle);
            angle -= hidx;
            if( hidx < 0 )
                hidx += _nbins;
            else if( hidx >= _nbins )
                hidx -= _nbins;
            assert( (unsigned)hidx < (unsigned)_nbins );

            qanglePtr[x*2] = (uchar)hidx;
            hidx++;
            hidx &= hidx < _nbins ? -1 : 0;
            qanglePtr[x*2+1] = (uchar)hidx;
            gradPtr[x*2] = mag*(1.f - angle);
            gradPtr[x*2+1] = mag*angle;
        }
    }
}

void HogFast::normalizeBlockHistogram(Vector<float>& _hist) const
{
    float* hist = &_hist[0];
    size_t i, sz = _hist.size();

    float sum = 0;
    for( i = 0; i < sz; i++ )
        sum += hist[i]*hist[i];
    float scale = 1.f/(std::sqrt(sum)+sz*0.1f), thresh = (float)L2HysThreshold;
    for( i = 0, sum = 0; i < sz; i++ )
    {
        hist[i] = std::min(hist[i]*scale, thresh);
        sum += hist[i]*hist[i];
    }
    scale = 1.f/(std::sqrt(sum)+1e-3f);
    for( i = 0; i < sz; i++ )
        hist[i] *= scale;
}


struct HOGCache
{
    struct BlockData
    {
        int histOfs;
        Point imgOffset;
    };

    struct PixData
    {
        size_t gradOfs, qangleOfs;
        int histOfs[4];
        float histWeights[4];
        float gradWeight;
    };

    HOGCache();
    HOGCache(const HogFast* descriptor,
        const Mat& img, Size paddingTL, Size paddingBR,
        bool useCache, Size cacheStride);
    HOGCache(const HogFast* descriptor,
        const Mat& img, Size paddingTL, Size paddingBR,
        bool useCache, Size cacheStride,const vector<Mat>& fbr, bool PbFlag);
    virtual ~HOGCache() {};
    virtual void init(const HogFast* descriptor,
        const Mat& img, Size paddingTL, Size paddingBR,
        bool useCache, Size cacheStride, const vector<Mat>& fbr=vector<Mat>(), bool PbFlag=false);

    Size windowsInImage(Size imageSize, Size winStride) const;
    Rect getWindow(Size imageSize, Size winStride, int idx) const;

    const float* getBlock(Point pt, float* buf);

    Vector<PixData> pixData;
    Vector<BlockData> blockData;

    bool useCache;
    Vector<int> ymaxCached;
    Size winSize, cacheStride;
    Size nblocks, ncells;
    int blockHistogramSize;
    int count1, count2, count4;
    Point imgoffset;
    Mat_<float> blockCache;
    Mat_<uchar> blockCacheFlags;

    Mat grad, qangle;
    const HogFast* descriptor;
};


HOGCache::HOGCache()
{
    useCache = false;
    blockHistogramSize = count1 = count2 = count4 = 0;
    descriptor = 0;
}

HOGCache::HOGCache(const HogFast* _descriptor,
        const Mat& _img, Size _paddingTL, Size _paddingBR,
        bool _useCache, Size _cacheStride)
{
    init(_descriptor, _img, _paddingTL, _paddingBR, _useCache, _cacheStride);
}

HOGCache::HOGCache(const HogFast* _descriptor,
        const Mat& _img, Size _paddingTL, Size _paddingBR,
        bool _useCache, Size _cacheStride, const vector<Mat>& fbr, bool PbFlag)
{
    if (PbFlag){
        init(_descriptor, _img, _paddingTL, _paddingBR, _useCache, _cacheStride, fbr, PbFlag);
    }else{
        init(_descriptor, _img, _paddingTL, _paddingBR, _useCache, _cacheStride);
    }
}

void HOGCache::init(const HogFast* _descriptor,
        const Mat& _img, Size _paddingTL, Size _paddingBR,
        bool _useCache, Size _cacheStride, const vector<Mat>& fbr, bool PbFlag)
{
    descriptor = _descriptor;
    cacheStride = _cacheStride;
    useCache = _useCache;
    if (PbFlag){
        // build grad, qangle from fbr
        cout << "computeGradient_fbr" << endl;
        descriptor->computeGradient_fbr(fbr, grad, qangle, _paddingTL, _paddingBR);
    }else{
        descriptor->computeGradient(_img, grad, qangle, _paddingTL, _paddingBR);
    }
    imgoffset = _paddingTL;

    winSize = descriptor->winSize;
    Size blockSize = descriptor->blockSize;
    Size blockStride = descriptor->blockStride;
    Size cellSize = descriptor->cellSize;
    Size winSize = descriptor->winSize;
    int i, j, nbins = descriptor->nbins;
    int rawBlockSize = blockSize.width*blockSize.height;

    nblocks = Size((winSize.width - blockSize.width)/blockStride.width + 1,
                   (winSize.height - blockSize.height)/blockStride.height + 1);
    ncells = Size(blockSize.width/cellSize.width, blockSize.height/cellSize.height);
    blockHistogramSize = ncells.width*ncells.height*nbins;

    if( useCache )
    {
        Size cacheSize((grad.cols - blockSize.width)/cacheStride.width+1,
                       (winSize.height/cacheStride.height)+1);
        blockCache.create(cacheSize.height, cacheSize.width*blockHistogramSize);
        blockCacheFlags.create(cacheSize);
        size_t i, cacheRows = blockCache.rows;
        ymaxCached.resize(cacheRows);
        for( i = 0; i < cacheRows; i++ )
            ymaxCached[i] = -1;
    }

    Mat_<float> weights(blockSize);
    float sigma = (float)descriptor->getWinSigma();
    float scale = 1.f/(sigma*sigma*2);

    for(i = 0; i < blockSize.height; i++)
        for(j = 0; j < blockSize.width; j++)
        {
            float di = i - blockSize.height*0.5f;
            float dj = j - blockSize.width*0.5f;
            weights(i,j) = std::exp(-(di*di + dj*dj)*scale);
        }

    blockData.resize(nblocks.width*nblocks.height);
    pixData.resize(rawBlockSize*3);

    // Initialize 2 lookup tables, pixData & blockData.
    // Here is why:
    //
    // The detection algorithm runs in 4 nested loops (at each pyramid layer):
    //  loop over the windows within the input image
    //    loop over the blocks within each window
    //      loop over the cells within each block
    //        loop over the pixels in each cell
    //
    // As each of the loops runs over a 2-dimensional array,
    // we could get 8(!) nested loops in total, which is very-very slow.
    //
    // To speed the things up, we do the following:
    //   1. loop over windows is unrolled in the HogFast::{computecompute|detect} methods;
    //         inside we compute the current search window using getWindow() method.
    //         Yes, it involves some overhead (function call + couple of divisions),
    //         but it's tiny in fact.
    //   2. loop over the blocks is also unrolled. Inside we use pre-computed blockData[j]
    //         to set up gradient and histogram pointers.
    //   3. loops over cells and pixels in each cell are merged
    //       (since there is no overlap between cells, each pixel in the block is processed once)
    //      and also unrolled. Inside we use PixData[k] to access the gradient values and
    //      update the histogram
    //
    count1 = count2 = count4 = 0;
    for( j = 0; j < blockSize.width; j++ )
        for( i = 0; i < blockSize.height; i++ )
        {
            PixData* data = 0;
            float cellX = (j+0.5f)/cellSize.width - 0.5f;
            float cellY = (i+0.5f)/cellSize.height - 0.5f;
            int icellX0 = cvFloor(cellX);
            int icellY0 = cvFloor(cellY);
            int icellX1 = icellX0 + 1, icellY1 = icellY0 + 1;
            cellX -= icellX0;
            cellY -= icellY0;

            if( (unsigned)icellX0 < (unsigned)ncells.width &&
                (unsigned)icellX1 < (unsigned)ncells.width )
            {
                if( (unsigned)icellY0 < (unsigned)ncells.height &&
                    (unsigned)icellY1 < (unsigned)ncells.height )
                {
                    data = &pixData[rawBlockSize*2 + (count4++)];
                    data->histOfs[0] = (icellX0*ncells.height + icellY0)*nbins;
                    data->histWeights[0] = (1.f - cellX)*(1.f - cellY);
                    data->histOfs[1] = (icellX1*ncells.height + icellY0)*nbins;
                    data->histWeights[1] = cellX*(1.f - cellY);
                    data->histOfs[2] = (icellX0*ncells.height + icellY1)*nbins;
                    data->histWeights[2] = (1.f - cellX)*cellY;
                    data->histOfs[3] = (icellX1*ncells.height + icellY1)*nbins;
                    data->histWeights[3] = cellX*cellY;
                }
                else
                {
                    data = &pixData[rawBlockSize + (count2++)];
                    if( (unsigned)icellY0 < (unsigned)ncells.height )
                    {
                        icellY1 = icellY0;
                        cellY = 1.f - cellY;
                    }
                    data->histOfs[0] = (icellX0*ncells.height + icellY1)*nbins;
                    data->histWeights[0] = (1.f - cellX)*cellY;
                    data->histOfs[1] = (icellX1*ncells.height + icellY1)*nbins;
                    data->histWeights[1] = cellX*cellY;
                    data->histOfs[2] = data->histOfs[3] = 0;
                    data->histWeights[2] = data->histWeights[3] = 0;
                }
            }
            else
            {
                if( (unsigned)icellX0 < (unsigned)ncells.width )
                {
                    icellX1 = icellX0;
                    cellX = 1.f - cellX;
                }

                if( (unsigned)icellY0 < (unsigned)ncells.height &&
                    (unsigned)icellY1 < (unsigned)ncells.height )
                {
                    data = &pixData[rawBlockSize + (count2++)];
                    data->histOfs[0] = (icellX1*ncells.height + icellY0)*nbins;
                    data->histWeights[0] = cellX*(1.f - cellY);
                    data->histOfs[1] = (icellX1*ncells.height + icellY1)*nbins;
                    data->histWeights[1] = cellX*cellY;
                    data->histOfs[2] = data->histOfs[3] = 0;
                    data->histWeights[2] = data->histWeights[3] = 0;
                }
                else
                {
                    data = &pixData[count1++];
                    if( (unsigned)icellY0 < (unsigned)ncells.height )
                    {
                        icellY1 = icellY0;
                        cellY = 1.f - cellY;
                    }
                    data->histOfs[0] = (icellX1*ncells.height + icellY1)*nbins;
                    data->histWeights[0] = cellX*cellY;
                    data->histOfs[1] = data->histOfs[2] = data->histOfs[3] = 0;
                    data->histWeights[1] = data->histWeights[2] = data->histWeights[3] = 0;
                }
            }
            data->gradOfs = (grad.cols*i + j)*2;
            data->qangleOfs = (qangle.cols*i + j)*2;
            data->gradWeight = weights(i,j);
        }

    assert( count1 + count2 + count4 == rawBlockSize );
    // defragment pixData
    for( j = 0; j < count2; j++ )
        pixData[j + count1] = pixData[j + rawBlockSize];
    for( j = 0; j < count4; j++ )
        pixData[j + count1 + count2] = pixData[j + rawBlockSize*2];
    count2 += count1;
    count4 += count2;

    // initialize blockData
    for( j = 0; j < nblocks.width; j++ )
        for( i = 0; i < nblocks.height; i++ )
        {
            BlockData& data = blockData[j*nblocks.height + i];
            data.histOfs = (j*nblocks.height + i)*blockHistogramSize;
            data.imgOffset = Point(j*blockStride.width,i*blockStride.height);
        }
}


const float* HOGCache::getBlock(Point pt, float* buf)
{
    float* blockHist = buf;
    assert(descriptor != 0);

    Size blockSize = descriptor->blockSize;
    pt += imgoffset;

    CV_Assert( (unsigned)pt.x <= (unsigned)(grad.cols - blockSize.width) &&
               (unsigned)pt.y <= (unsigned)(grad.rows - blockSize.height) );

    if( useCache )
    {
        std::cout << "used Cache" << std::endl;
        CV_Assert( pt.x % cacheStride.width == 0 &&
                   pt.y % cacheStride.height == 0 );
        Point cacheIdx(pt.x/cacheStride.width,
                      (pt.y/cacheStride.height) % blockCache.rows);
        if( pt.y != ymaxCached[cacheIdx.y] )
        {
            Mat_<uchar> cacheRow = blockCacheFlags.row(cacheIdx.y);
            cacheRow = (uchar)0;
            ymaxCached[cacheIdx.y] = pt.y;
        }

        blockHist = &blockCache[cacheIdx.y][cacheIdx.x*blockHistogramSize];
        uchar& computedFlag = blockCacheFlags(cacheIdx.y, cacheIdx.x);
        if( computedFlag != 0 )
        {
            return blockHist;}
        computedFlag = (uchar)1; // set it at once, before actual computing
    }

    int k, C1 = count1, C2 = count2, C4 = count4;
    const float* gradPtr = (const float*)(grad.data + grad.step*pt.y) + pt.x*2;
    const uchar* qanglePtr = qangle.data + qangle.step*pt.y + pt.x*2;

    CV_Assert( blockHist != 0 );

    for( k = 0; k < blockHistogramSize; k++ )
        blockHist[k] = 0.f;

    const PixData* _pixData = &pixData[0];

    for( k = 0; k < C1; k++ )
    {
        const PixData& pk = _pixData[k];
        const float* a = gradPtr + pk.gradOfs;
        float w = pk.gradWeight*pk.histWeights[0];
        const uchar* h = qanglePtr + pk.qangleOfs;
        int h0 = h[0], h1 = h[1];
        float* hist = blockHist + pk.histOfs[0];
        float t0 = hist[h0] + a[0]*w;
        float t1 = hist[h1] + a[1]*w;
        hist[h0] = t0; hist[h1] = t1;
    }

    for( ; k < C2; k++ )
    {
        const PixData& pk = _pixData[k];
        const float* a = gradPtr + pk.gradOfs;
        float w, t0, t1, a0 = a[0], a1 = a[1];
        const uchar* h = qanglePtr + pk.qangleOfs;
        int h0 = h[0], h1 = h[1];

        float* hist = blockHist + pk.histOfs[0];
        w = pk.gradWeight*pk.histWeights[0];
        t0 = hist[h0] + a0*w;
        t1 = hist[h1] + a1*w;
        hist[h0] = t0; hist[h1] = t1;

        hist = blockHist + pk.histOfs[1];
        w = pk.gradWeight*pk.histWeights[1];
        t0 = hist[h0] + a0*w;
        t1 = hist[h1] + a1*w;
        hist[h0] = t0; hist[h1] = t1;
    }

    for( ; k < C4; k++ )
    {
        const PixData& pk = _pixData[k];
        const float* a = gradPtr + pk.gradOfs;
        float w, t0, t1, a0 = a[0], a1 = a[1];
        const uchar* h = qanglePtr + pk.qangleOfs;
        int h0 = h[0], h1 = h[1];

        float* hist = blockHist + pk.histOfs[0];
        w = pk.gradWeight*pk.histWeights[0];
        t0 = hist[h0] + a0*w;
        t1 = hist[h1] + a1*w;
        hist[h0] = t0; hist[h1] = t1;

        hist = blockHist + pk.histOfs[1];
        w = pk.gradWeight*pk.histWeights[1];
        t0 = hist[h0] + a0*w;
        t1 = hist[h1] + a1*w;
        hist[h0] = t0; hist[h1] = t1;

        hist = blockHist + pk.histOfs[2];
        w = pk.gradWeight*pk.histWeights[2];
        t0 = hist[h0] + a0*w;
        t1 = hist[h1] + a1*w;
        hist[h0] = t0; hist[h1] = t1;

        hist = blockHist + pk.histOfs[3];
        w = pk.gradWeight*pk.histWeights[3];
        t0 = hist[h0] + a0*w;
        t1 = hist[h1] + a1*w;
        hist[h0] = t0; hist[h1] = t1;
    }

    // normalize the block histogram
    Vector<float> d(blockHist, (size_t)blockHistogramSize);
    descriptor->normalizeBlockHistogram(d);

    return blockHist;
}


Size HOGCache::windowsInImage(Size imageSize, Size winStride) const
{
    return Size((imageSize.width - winSize.width)/winStride.width + 1,
                (imageSize.height - winSize.height)/winStride.height + 1);
}

Rect HOGCache::getWindow(Size imageSize, Size winStride, int idx) const
{
    int nwindowsX = (imageSize.width - winSize.width)/winStride.width + 1;
    int y = idx / nwindowsX;
    int x = idx - nwindowsX*y;
    return Rect( x*winStride.width, y*winStride.height, winSize.width, winSize.height );
}


size_t HogFast::compute(const Mat& img, Vector<Point>& locations,
                            Vector<float>& descriptors,
                            Size winStride, Size paddingTL, Size paddingBR,
                            const string& EdgeMapFilename_filename, bool PbFlag
                            )
{
    vector<Mat> fbr;
    if (PbFlag){
        string tmp_string = EdgeMapFilename_filename+".jpg";
        cout<< tmp_string << endl;
        bool fbr_done = load_orient_edges( tmp_string, fbr );
        if (!fbr_done)
            cout << "can not load " << EdgeMapFilename_filename << endl;
    }

    if( winStride == Size() ){
        winStride = cellSize;
        cout << " winStride=" << winStride.width <<" " <<winStride.height << endl;
    }
    Size cacheStride(gcd(winStride.width, blockStride.width),
                     gcd(winStride.height, blockStride.height));
    size_t nwindows = locations.size();
    bool locationsOriEmptyFlag = false;
    paddingTL.width = (int)alignSize(std::max(paddingTL.width, 0), cacheStride.width);
    paddingTL.height = (int)alignSize(std::max(paddingTL.height, 0), cacheStride.height);
    paddingBR.width = (int)alignSize(std::max(paddingBR.width, 0), cacheStride.width);
    paddingBR.height = (int)alignSize(std::max(paddingBR.height, 0), cacheStride.height);

    Size paddedImgSize(img.cols + paddingTL.width + paddingBR.width, img.rows + paddingTL.height + paddingBR.height);

    HOGCache cache(this, img, paddingTL, paddingBR, false, cacheStride, fbr, PbFlag);

    std::cout << "nwindows = " << nwindows << std::endl;
    if( !nwindows ){
        nwindows = cache.windowsInImage(paddedImgSize, winStride).area();
        locationsOriEmptyFlag = true;
    }
    std::cout << "nwindows = " << nwindows << std::endl;

    const HOGCache::BlockData* blockData = &cache.blockData[0];
    cout << "here1"<<endl;
    int nblocks = cache.nblocks.area();
    cout << "here2"<<endl;
    int blockHistogramSize = cache.blockHistogramSize;
    cout << "here3"<<endl;
    size_t dsize = getDescriptorSize();
    cout << "here4"<<endl;
    descriptors.resize(dsize*nwindows);
    cout << "here5"<<endl;
    for( size_t i = 0; i < nwindows; i++ )
    {
        float* descriptor = &descriptors[i*dsize];

        Point pt0;
        if( !locationsOriEmptyFlag)
        {
            pt0 = locations[i];
            if( pt0.x < -paddingTL.width || pt0.x > img.cols + paddingBR.width - winSize.width ||
                pt0.y < -paddingTL.height || pt0.y > img.rows + paddingBR.height - winSize.height ){
                continue;
            }else{
                pt0 -= Point(paddingTL);
            }
        }
        else
        {
            pt0 = cache.getWindow(paddedImgSize, winStride, i).tl() - Point(paddingTL);
            CV_Assert(pt0.x % cacheStride.width == 0 && pt0.y % cacheStride.height == 0);
            // keep the pt0s
            locations.push_back(pt0);
        }

        for( int j = 0; j < nblocks; j++ )
        {
            const HOGCache::BlockData& bj = blockData[j];
            Point pt = pt0 + bj.imgOffset;

            float* dst = descriptor + bj.histOfs;
            const float* src = cache.getBlock(pt, dst);
            if( src != dst )
                for( int k = 0; k < blockHistogramSize; k++ )
                    dst[k] = src[k];
        }

    }

    return dsize;
}


void HogFast::detect(const Mat& img,
    Vector<Point>& hits, Vector<Point>& hitsIds,
    vector<float>& hitsConfs, double hitThreshold,
    Size winStride, Size padding, const vector<Mat>& fbr, bool PbFlag, const Vector<Point>& locations)
{
    hits.clear();

    if( winStride == Size() )
        winStride = cellSize;
    Size cacheStride(gcd(winStride.width, blockStride.width),
                     gcd(winStride.height, blockStride.height));
    size_t nwindows = locations.size();

    padding.width = (int)alignSize(std::max(padding.width, 0), cacheStride.width);
    padding.height = (int)alignSize(std::max(padding.height, 0), cacheStride.height);

    HOGCache cache(this, img, padding, padding, false, cacheStride, fbr, PbFlag);

    Size paddedImgSize(img.cols + padding.width*2, img.rows + padding.height*2);

    std::cout << "nwindows = " << nwindows << std::endl;
    if( !nwindows )
        nwindows = cache.windowsInImage(paddedImgSize, winStride).area();
    std::cout << "nwindows = " << nwindows << std::endl;

    const HOGCache::BlockData* blockData = &cache.blockData[0];

    int nblocks = cache.nblocks.area();
    cout << "nblocks " << nblocks << endl;
    int blockHistogramSize = cache.blockHistogramSize;
    cout << "blockHistogramSize " << blockHistogramSize << endl;

    Vector<float> blockHist(blockHistogramSize);

    vector< vector<float> > vecTestFeature;
    vecTestFeature.resize(nblocks*blockHistogramSize);
    for (unsigned int i=0; i < vecTestFeature.size();i++){
        vecTestFeature.at(i).resize(1);
    }

    vector< vector<int> > vecTestLeafNo;
    vecTestLeafNo.resize(1);
    float FgBgProb_inst;

    for( size_t i = 0; i < nwindows; i++ )
    {
        vecTestLeafNo.at(0).clear();

        Point pt0;
        if( !locations.empty() )
        {
            pt0 = locations[i];
            if( pt0.x < -padding.width || pt0.x > img.cols + padding.width - winSize.width ||
                pt0.y < -padding.height || pt0.y > img.rows + padding.height - winSize.height ){
                continue;
            }else{
                pt0 -= Point(padding);
            }
        }
        else
        {
            pt0 = cache.getWindow(paddedImgSize, winStride, i).tl() - Point(padding);
            CV_Assert(pt0.x % cacheStride.width == 0 && pt0.y % cacheStride.height == 0);
        }


        for( int j = 0; j < nblocks; j++)
        {

            const HOGCache::BlockData& bj = blockData[j];
            Point pt = pt0 + bj.imgOffset;

            const float* src = cache.getBlock(pt, &blockHist[0]);
            for( int k = 0; k < blockHistogramSize; k++ ){
                    vecTestFeature.at(j*blockHistogramSize+k).at(0) = src[k];
            }
        }


        rf.RFPredict(vecTestFeature, vecTestLeafNo);

        for (unsigned int j=0; j < vecTestLeafNo.at(0).size(); j++){
            int NodeId = vecTestLeafNo.at(0).at(j);
            FgBgProb_inst = FgBgProb.at(j).at(NodeId);
            if( FgBgProb_inst >= hitThreshold ){
                hits.push_back(pt0);
                hitsIds.push_back(Point(j,vecTestLeafNo.at(0).at(j)));
                hitsConfs.push_back( FgBgProb_inst);
            }
        }
    }
}


struct SimilarRects
{
    SimilarRects(double _eps) : eps(_eps) {}
    inline bool operator()(const Rect& r1, const Rect& r2) const
    {
        double delta = eps*(std::min(r1.width, r2.width) + std::min(r1.height, r2.height))*0.5;
        return std::abs(r1.x - r2.x) <= delta &&
            std::abs(r1.y - r2.y) <= delta &&
            std::abs(r1.x + r1.width - r2.x - r2.width) <= delta &&
            std::abs(r1.y + r1.height - r2.y - r2.height) <= delta;
    }
    double eps;
};

struct HOGThreadData
{
    Vector<Rect> rectangles;
    Vector<Point> locations;
    Mat smallerImgBuf;
};

bool HogFast::load_orient_edges( const string& orient_edges_filename, vector<Mat>& fbr ){
    ifstream fin(orient_edges_filename.c_str(), ios::in | ios::binary);;
    if (!fin){
        cout << fin;
        return false;
    }

    unsigned int dim;
    fin.read((char *)&dim,sizeof(unsigned int));

    cout << dim <<endl;
    vector<unsigned int> d(dim);
    for ( unsigned int i=0; i< dim; i++){
        fin.read((char *)&d[i],sizeof(unsigned int));
        cout << d[i] <<endl;
    }
    fbr.resize(d[0]);
    for (unsigned int chid =0; chid < d[0]; chid++){
        fbr[chid].create(d[2], d[1], CV_32FC1);
         for (unsigned int x=0; x<d[1];x++){
            for (unsigned int y=0; y<d[2];y++){
            float * fbrptr = (float*) fbr[chid].ptr(y);
                fin.read((char *)&fbrptr[x],sizeof(float));
            }
        }
    }

    nbins = d[0];
    cout << "nbins"<< nbins << endl;

    return true;
}

void HogFast::detectMultiScale(
    const Mat& img, Vector<Rect>& foundLocations, Vector<Point>& foundIds,
    vector< float>& foundConfs, vector<int>& ScaleChangeBoundary,
    double hitThreshold, Size winStride, Size padding,
    double scale0, int groupThreshold, float ObjHeight2winHeightRatio, const string& EdgeMapFilename_filename, bool PbFlag)
{
    // load fbr
    vector<Mat> fbr;
    if (PbFlag){
        string tmp_string = EdgeMapFilename_filename;
        bool fbr_done = load_orient_edges( tmp_string, fbr );
        if (!fbr_done)
            cout << "can not load " << EdgeMapFilename_filename << endl;
    }

    double scale = 1.;
    foundLocations.clear();
    int i, levels = 0;
    const int maxLevels = 64;

    int t, nthreads = getNumThreads();
    Vector<HOGThreadData> threadData(nthreads);
    vector< Vector<Point> > threadFoundIds(nthreads);
    vector< vector<float> > threadFoundConfs(nthreads);

    for( t = 0; t < nthreads; t++ )
        threadData[t].smallerImgBuf.create(img.size(), img.type());

    Vector<double> levelScale(maxLevels);
    for( levels = 0; levels < maxLevels; levels++ )
    {
        levelScale[levels] = scale;
        if( cvRound(img.cols/scale) < (float)winSize.width*ObjHeight2winHeightRatio ||
            cvRound(img.rows/scale) < (float)winSize.height*ObjHeight2winHeightRatio ||
            scale0 <= 1 )
            break;
        scale *= scale0;
    }
    levels = std::max(levels, 1);
    levelScale.resize(levels);

    ScaleChangeBoundary.resize(levels);
    {
#ifdef _OPENMP
    #pragma omp parallel for num_threads(nthreads) schedule(dynamic)
#endif // _OPENMP
    for( i = 0; i < levels; i++ )
    {
        HOGThreadData& tdata = threadData[getThreadNum()];
        Vector<Point>& Iddata = threadFoundIds[getThreadNum()];
        vector<float>& Confsdata = threadFoundConfs[getThreadNum()];

        double scale = levelScale[i];
        Size sz(cvRound(img.cols/scale), cvRound(img.rows/scale));
        Mat smallerImg(sz, img.type(), tdata.smallerImgBuf.data);

        if( sz == img.size() ){
            smallerImg = Mat(sz, img.type(), img.data, img.step);
        }else{
            resize(img, smallerImg, sz);
        }
        if (PbFlag){
            vector< Mat> smallerfbr( fbr.size());
            if( sz == img.size() ){
                for (unsigned int bid=0; bid<smallerfbr.size();bid++)
                    smallerfbr[bid] = Mat(sz, fbr[bid].type(), fbr[bid].data, fbr[bid].step);
            }else{
                for (unsigned int bid=0; bid<smallerfbr.size();bid++){
                    smallerfbr[bid] = Mat(sz, fbr[bid].type(), fbr[bid].data, fbr[bid].step);
                    resize(fbr[bid], smallerfbr[bid], sz);
                }
            }
            detect(smallerImg, tdata.locations, Iddata, Confsdata, hitThreshold, winStride, padding, smallerfbr, PbFlag);
        }else{
            detect(smallerImg, tdata.locations, Iddata, Confsdata, hitThreshold, winStride, padding);
        }
        Size scaledWinSize = Size(cvRound(winSize.width*scale), cvRound(winSize.height*scale));
        for( size_t j = 0; j < tdata.locations.size(); j++ ){
            tdata.rectangles.push_back(Rect(
                cvRound(tdata.locations[j].x*scale),
                cvRound(tdata.locations[j].y*scale),
                scaledWinSize.width, scaledWinSize.height));
        }
        ScaleChangeBoundary.at(i) = tdata.locations.size();
    }
    }

    for( t = 0; t < nthreads; t++ )
    {
        HOGThreadData& tdata = threadData[t];
        Vector<Point>& Iddata = threadFoundIds[t];
        vector<float>& Confsdata = threadFoundConfs[t];
        std::copy(tdata.rectangles.begin(), tdata.rectangles.end(),
            std::back_inserter(foundLocations));
        std::copy(Iddata.begin(), Iddata.end(),
            std::back_inserter(foundIds));
        std::copy(Confsdata.begin(), Confsdata.end(),
            std::back_inserter(foundConfs));

    }

    return;
}

void HogFast::detectSetOfScale(
    const Mat& img, Vector<Rect>& foundLocations, Vector<Point>& foundIds,
    vector< float>& foundConfs, vector<int>& ScaleChangeBoundary,
    double hitThreshold, Size winStride, Size padding,
    const vector<Vector<Point> >& locations, const vector<Size>& scaledWinSize,
    int groupThreshold, float ObjHeight2winHeightRatio, const string& EdgeMapFilename_filename, bool PbFlag)
{
    // load fbr
    vector<Mat> fbr;
    if (PbFlag){
        string tmp_string = EdgeMapFilename_filename;
        bool fbr_done = load_orient_edges( tmp_string, fbr );
        if (!fbr_done)
            cout << "can not load " << EdgeMapFilename_filename << endl;
    }

    double scale = 1.;
    foundLocations.clear();
    int i, levels = 0;

    int t, nthreads = getNumThreads();
    Vector<HOGThreadData> threadData(nthreads);
    vector< Vector<Point> > threadFoundIds(nthreads);
    vector< vector<float> > threadFoundConfs(nthreads);

    scale = (double)(scaledWinSize[0].width)/(double) (winSize.width);
    Size imgScaleSize = Size(img.size().width/scale,img.size().height/scale);
    cout << "imgScaleSize" << imgScaleSize.width << " " <<imgScaleSize.height <<endl;
    for( t = 0; t < nthreads; t++ )
        threadData[t].smallerImgBuf.create(imgScaleSize, img.type());

    levels = scaledWinSize.size();
    ScaleChangeBoundary.resize( levels);
    {
#ifdef _OPENMP
        #pragma omp parallel for num_threads(nthreads) schedule(dynamic)
#endif // _OPENMP
        for( i = 0; i < levels; i++ )
        {
            HOGThreadData& tdata = threadData[getThreadNum()];
            Vector<Point>& Iddata = threadFoundIds[getThreadNum()];
            vector<float>& Confsdata = threadFoundConfs[getThreadNum()];

            scale = (double)(scaledWinSize[i].width)/(double) (winSize.width);
            Size sz(cvRound(img.cols/scale), cvRound(img.rows/scale));
            Mat smallerImg(sz, img.type(), tdata.smallerImgBuf.data);

            if( sz == img.size() ){
                smallerImg = Mat(sz, img.type(), img.data, img.step);
            }else{
                resize(img, smallerImg, sz);
            }
            // modify locations
            Vector<Point> scaled_locations;
            for( size_t j = 0; j < locations[i].size(); j++ ){
                scaled_locations.push_back( Point(
                    cvRound( locations[i][j].x/scale),
                    cvRound( locations[i][j].y/scale) ));
            }


            if (PbFlag){
                vector< Mat> smallerfbr( fbr.size());
                if( sz == img.size() ){
                    for (unsigned int bid=0; bid<smallerfbr.size();bid++)
                        smallerfbr[bid] = Mat(sz, fbr[bid].type(), fbr[bid].data, fbr[bid].step);
                }else{
                    for (unsigned int bid=0; bid<smallerfbr.size();bid++){
                        smallerfbr[bid] = Mat(sz, fbr[bid].type(), fbr[bid].data, fbr[bid].step);
                        resize(fbr[bid], smallerfbr[bid], sz);
                    }
                }
                detect(smallerImg, tdata.locations, Iddata, Confsdata, hitThreshold, winStride, padding, smallerfbr, PbFlag, scaled_locations);
            }else{
                detect(smallerImg, tdata.locations, Iddata, Confsdata, hitThreshold, winStride, padding, vector<Mat>(), false, scaled_locations);
            }
            for( size_t j = 0; j < tdata.locations.size(); j++ ){
                tdata.rectangles.push_back(Rect(
                    cvRound(tdata.locations[j].x*scale),
                    cvRound(tdata.locations[j].y*scale),
                    scaledWinSize[i].width, scaledWinSize[i].height));
            }
            ScaleChangeBoundary.at(i) = tdata.locations.size();
        }
    }

    for( t = 0; t < nthreads; t++ )
    {
        HOGThreadData& tdata = threadData[t];
        Vector<Point>& Iddata = threadFoundIds[t];
        vector<float>& Confsdata = threadFoundConfs[t];
        std::copy(tdata.rectangles.begin(), tdata.rectangles.end(),
            std::back_inserter(foundLocations));
        std::copy(Iddata.begin(), Iddata.end(),
            std::back_inserter(foundIds));
        std::copy(Confsdata.begin(), Confsdata.end(),
            std::back_inserter(foundConfs));
    }

    return;
}

void HogFast::detectAppendFea(const Mat& img,
    Vector<Point>& hits, Vector<Point>& hitsIds,
    vector<float>& hitsConfs, double hitThreshold,
    Size winStride, Size padding, const vector<Mat>& fbr, bool PbFlag, const Vector<Point>& locations, vector< cv::Vector<float>* >& des_ptr_3d)
{
    hits.clear();

    if( winStride == Size() )
        winStride = cellSize;
    Size cacheStride(gcd(winStride.width, blockStride.width),
                     gcd(winStride.height, blockStride.height));
    size_t nwindows = locations.size();

    padding.width = (int)alignSize(std::max(padding.width, 0), cacheStride.width);
    padding.height = (int)alignSize(std::max(padding.height, 0), cacheStride.height);

    HOGCache cache(this, img, padding, padding, false, cacheStride, fbr, PbFlag);

    Size paddedImgSize(img.cols + padding.width*2, img.rows + padding.height*2);

    if( !nwindows ){
        cout << "erro no locations predefined" << endl;
        return;
    }

    const HOGCache::BlockData* blockData = &cache.blockData[0];

    int nblocks = cache.nblocks.area();
    int blockHistogramSize = cache.blockHistogramSize;

    Vector<float> blockHist(blockHistogramSize);

    vector< vector<float> > vecTestFeature;
    vecTestFeature.resize(nblocks*blockHistogramSize);
    for (unsigned int i=0; i < vecTestFeature.size();i++){
        vecTestFeature.at(i).resize(1);
    }

    vector< vector<int> > vecTestLeafNo;
    vecTestLeafNo.resize(1);
    float FgBgProb_inst;

    for( size_t i = 0; i < nwindows; i++ )
    {
        vecTestLeafNo.at(0).clear();

        Point pt0;
        if( !locations.empty() )
        {
            pt0 = locations[i];
            if( pt0.x < -padding.width || pt0.x > img.cols + padding.width - winSize.width ||
                pt0.y < -padding.height || pt0.y > img.rows + padding.height - winSize.height ){
                continue;
            }else{
                pt0 -= Point(padding);
            }
        }
        else
        {
            pt0 = cache.getWindow(paddedImgSize, winStride, i).tl() - Point(padding);
            CV_Assert(pt0.x % cacheStride.width == 0 && pt0.y % cacheStride.height == 0);
        }


        for( int j = 0; j < nblocks; j++)
        {

            const HOGCache::BlockData& bj = blockData[j];
            Point pt = pt0 + bj.imgOffset;

            const float* src = cache.getBlock(pt, &blockHist[0]);
            for( int k = 0; k < blockHistogramSize; k++ ){
                    vecTestFeature.at(j*blockHistogramSize+k).at(0) = src[k];
            }
        }


            vector< vector<float> > vecTestFeatureApp;
            vecTestFeatureApp.resize( des_ptr_3d[i]->size()+vecTestFeature.size());
            for (unsigned int j =0; j < (vecTestFeature.size()+des_ptr_3d[i]->size()); j++){
                if (j<vecTestFeature.size()){
                    vecTestFeatureApp[j].push_back( vecTestFeature[j][0]);
                }else{
                    vecTestFeatureApp[j].push_back( (*(des_ptr_3d[i]))[j-vecTestFeature.size()]);
                }
            }
            rf.RFPredict(vecTestFeatureApp, vecTestLeafNo);

        for (unsigned int j=0; j < vecTestLeafNo.at(0).size(); j++){
            int NodeId = vecTestLeafNo.at(0).at(j);
            FgBgProb_inst = FgBgProb.at(j).at(NodeId);
            if( FgBgProb_inst >= hitThreshold ){
                hits.push_back(pt0);
                hitsIds.push_back(Point(j,vecTestLeafNo.at(0).at(j)));
                hitsConfs.push_back( FgBgProb_inst);
            }
        }
    }
}

void HogFast::detectSetOfScaleAppendFea(
    const Mat& img, Vector<Rect>& foundLocations, Vector<Point>& foundIds,
    vector< float>& foundConfs, 
    cv::Vector<cv::Vector<float> >& descriptors_3d,
    vector<int>& ScaleChangeBoundary,
    double hitThreshold, Size winStride, Size padding,
    const vector<Vector<Point> >& locations, const vector<Size>& scaledWinSize,
    int groupThreshold, float ObjHeight2winHeightRatio, const string& EdgeMapFilename_filename, bool PbFlag)
{
    // load fbr
    vector<Mat> fbr;
    if (PbFlag){
        string tmp_string = EdgeMapFilename_filename;
        bool fbr_done = load_orient_edges( tmp_string, fbr );
        if (!fbr_done)
            cout << "can not load " << EdgeMapFilename_filename << endl;
    }

    double scale = 1.;
    foundLocations.clear();
    int i, levels = 0;

    int t, nthreads = getNumThreads();
    Vector<HOGThreadData> threadData(nthreads);
    vector< Vector<Point> > threadFoundIds(nthreads);
    vector< vector<float> > threadFoundConfs(nthreads);

    scale = (double)(scaledWinSize[0].width)/(double) (winSize.width);
    Size imgScaleSize = Size(img.size().width/scale,img.size().height/scale);
    cout << "imgScaleSize" << imgScaleSize.width << " " <<imgScaleSize.height <<endl;
    for( t = 0; t < nthreads; t++ )
        threadData[t].smallerImgBuf.create(imgScaleSize, img.type());

    levels = scaledWinSize.size();
    ScaleChangeBoundary.resize( levels);

    // set pointer to the level of descriptors_3d
    vector<unsigned int> level_cum_count(levels+1);
    level_cum_count[0] = 0;
    for ( int i=0; i<levels; i++){
        level_cum_count[i+1] = locations[i].size() + level_cum_count[i];
    }

    {
#ifdef _OPENMP
        #pragma omp parallel for num_threads(nthreads) schedule(dynamic)
#endif // _OPENMP
        for( i = 0; i < levels; i++ )
        {
            HOGThreadData& tdata = threadData[getThreadNum()];
            Vector<Point>& Iddata = threadFoundIds[getThreadNum()];
            vector<float>& Confsdata = threadFoundConfs[getThreadNum()];

            scale = (double)(scaledWinSize[i].width)/(double) (winSize.width);
            Size sz(cvRound(img.cols/scale), cvRound(img.rows/scale));
            Mat smallerImg(sz, img.type(), tdata.smallerImgBuf.data);

            if( sz == img.size() ){
                smallerImg = Mat(sz, img.type(), img.data, img.step);
            }else{
                resize(img, smallerImg, sz);
            }
            // modify locations
            Vector<Point> scaled_locations;
            for( size_t j = 0; j < locations[i].size(); j++ ){
                scaled_locations.push_back( Point(
                    cvRound( locations[i][j].x/scale),
                    cvRound( locations[i][j].y/scale) ));
            }

            vector< cv::Vector<float>* > des_ptr_3d;
            for ( unsigned int q = level_cum_count[i]; q < level_cum_count[i+1]; q++){    
                des_ptr_3d.push_back(&descriptors_3d[ q]);
            }
            if (PbFlag){
                vector< Mat> smallerfbr( fbr.size());
                if( sz == img.size() ){
                    for (unsigned int bid=0; bid<smallerfbr.size();bid++)
                        smallerfbr[bid] = Mat(sz, fbr[bid].type(), fbr[bid].data, fbr[bid].step);
                }else{
                    for (unsigned int bid=0; bid<smallerfbr.size();bid++){
                        smallerfbr[bid] = Mat(sz, fbr[bid].type(), fbr[bid].data, fbr[bid].step);
                        resize(fbr[bid], smallerfbr[bid], sz);
                    }
                }                
                detectAppendFea(smallerImg, tdata.locations, Iddata, Confsdata, hitThreshold, winStride, padding, smallerfbr, PbFlag, scaled_locations, des_ptr_3d);
            }else{
                detectAppendFea(smallerImg, tdata.locations, Iddata, Confsdata, hitThreshold, winStride, padding, vector<Mat>(), false, scaled_locations, des_ptr_3d);
            }
            for( size_t j = 0; j < tdata.locations.size(); j++ ){
                tdata.rectangles.push_back(Rect(
                    cvRound(tdata.locations[j].x*scale),
                    cvRound(tdata.locations[j].y*scale),
                    scaledWinSize[i].width, scaledWinSize[i].height));
            }
            ScaleChangeBoundary.at(i) = tdata.locations.size();
        }
    }

    for( t = 0; t < nthreads; t++ )
    {
        HOGThreadData& tdata = threadData[t];
        Vector<Point>& Iddata = threadFoundIds[t];
        vector<float>& Confsdata = threadFoundConfs[t];
        std::copy(tdata.rectangles.begin(), tdata.rectangles.end(),
            std::back_inserter(foundLocations));
        std::copy(Iddata.begin(), Iddata.end(),
            std::back_inserter(foundIds));
        std::copy(Confsdata.begin(), Confsdata.end(),
            std::back_inserter(foundConfs));

    }

    return;
}
