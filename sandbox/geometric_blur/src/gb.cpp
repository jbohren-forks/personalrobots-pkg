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

// Author Min Sun

#include "geometric_blur/gb.h"

using namespace cv;
using namespace std;

void gb::setDefaultRsNthetas(){
    //default rs
    rs.resize(6);
    rs[0]=0.00000;
    rs[1]=0.08000;
    rs[2]=0.16000;
    rs[3]=0.32000;
    rs[4]=0.64000;
    rs[5]=1.00000;

    //default nthetas
    nthetas.resize(6);
    nthetas[0]=1;
    nthetas[1]=8;
    nthetas[2]=8;
    nthetas[3]=10;
    nthetas[4]=12;
    nthetas[5]=12;

    for (unsigned int i=0; i< rs.size(); i++){
        rs[i] *= winSize.width/2;
    }
}

void gb::compute_orient_edges(Mat& img, vector<Mat>& fbr){

    imgSize = img.size();

    Size gradsize(img.cols + paddingTL.width + paddingBR.width,
                  img.rows + paddingTL.height + paddingBR.height);
    fbr.resize(nbins);
    for ( int chid = 0; chid < nbins; chid++){
        fbr[chid].create(gradsize, CV_32FC1);  // magnitude of each channel
    }

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
        vector<float*> fbrPtr;
        for ( int chid=0; chid < nbins; chid++){
             fbrPtr.push_back( (float*)fbr[chid].ptr(y) );
        }

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
                // pick the max mag across all channels
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

            fbrPtr[hidx][x] = mag;
        }
    }

}

bool gb::load_orient_edges( string& orient_edges_filename, vector<Mat>& fbr ){
    ifstream fin(orient_edges_filename.c_str(), ios::in | ios::binary);;
    if (!fin)
        return false;

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
    imgSize.width = d[1];
    imgSize.height = d[2];

    return true;
}

void gb::compute_sample_rois(Size CurrImgSize, vector<Rect>& rois, vector<Point>& roi_center){
    if (DenseSample){
//        cout << CurrImgSize.width << " " << winSize.width << " " << CurrImgSize.height << " " << winSize.height;
        for ( int x=0; x<std::max(CurrImgSize.width-winSize.width,1); x+=winStride.width){
            for ( int y=0; y<std::max(CurrImgSize.height-winSize.height,1); y+=winStride.height){
                rois.push_back(Rect(x,y,winSize.width,winSize.height));
            }
        }
    }else{ // sample roi along the strong edges -To Do
        
    }

    for (unsigned int k = 0; k< rois.size(); k++){
            Point tmpPoint;
            tmpPoint.x = rois[k].x+rois[k].width/2;
            tmpPoint.y = rois[k].y+rois[k].height/2;
            roi_center.push_back(tmpPoint);
        }
}

void gb::compute_gb(vector<Mat>& fbr, vector< vector<float> > & features){
    float acc_scale = 1;
    int levels = 0;
    if (ScaleRatio == 1)
        NumScale = 1; // handle exception case

    Vector<double> levelScale(NumScale);
    for( levels = 0; levels < NumScale; levels++ )
    {
        levelScale[levels] = acc_scale;
        if( cvRound(imgSize.width/acc_scale) < (float)winSize.width/2 ||
            cvRound(imgSize.height/acc_scale) < (float)winSize.height/2 ||
            ScaleRatio <= 1 )
            break;
        acc_scale *= ScaleRatio;
    }
    levels = std::max(levels, 1);
    levelScale.resize(levels);

    for ( int sid = 0; sid< (int)levelScale.size(); sid++){
    //        if (winSize.width>=imgSize.width*2 || winSize.height >= imgSize.height*2)// hacky 2, since some training images do have winSize.width >= imgSize.width
    //            return; // the winSize is relatively too big
        Size CurrImgSize;
        CurrImgSize.width = imgSize.width/levelScale[sid];
        CurrImgSize.height = imgSize.height/levelScale[sid];

        vector<Rect> rois;
        vector<Point> roi_center;
        compute_sample_rois(CurrImgSize, rois,  roi_center);

        cout << "start compute_gb_single_scale" << endl;
        gb::compute_gb_single_scale(fbr, features, CurrImgSize, rois,  roi_center);

        // resizing rois
        for (unsigned int k=0; k< rois.size(); k++){
            rois[k].x = rois[k].x*acc_scale;
            rois[k].y = rois[k].y*acc_scale;
            rois[k].width = rois[k].width*acc_scale;
            rois[k].height = rois[k].height*acc_scale;
        }
        // save roi
        ROI.insert(ROI.end(),rois.begin(), rois.end());

//        imgSize.width /= ScaleRatio;
//        imgSize.height /= ScaleRatio;
        acc_scale *= ScaleRatio;
//        roi_center.clear();
//        rois.clear();
    }

    // L2 normalized the feature
    for (unsigned int fid = 0; fid < features.size(); fid++)
        gb::normalizeBlockHistogram( features[fid]);
}

void gb::normalizeBlockHistogram(vector<float>& _hist)
{
    float* hist = &_hist[0];
    size_t i, sz = _hist.size();

    float sum = 0;
    for( i = 0; i < sz; i++ )
        sum += hist[i]*hist[i];
    float scale = 1.f/(std::sqrt(sum)+sz*1e-8f);
    for( i = 0, sum = 0; i < sz; i++ )
    {
        hist[i] = hist[i]*scale;
    }
}

void gb::compute_gb_single_scale(vector<Mat>& fbr, vector< vector<float> > & features, Size CurrImgSize, vector<Rect>& rois, vector<Point>& roi_center){

    // parameter setting
    unsigned int numSigma = rs.size();

    // sample points relative to roi centers
    vector<Point> sample_points;
    vector< vector<int> > sample_points_sigma_level;
    gb::compute_sample_points( sample_points, sample_points_sigma_level);
    unsigned int nFeaOffset = features.size();
    features.resize(nFeaOffset+roi_center.size());
//    cout << "nFeaOffset "<<nFeaOffset;//<<  " sample_points.size() "<< sample_points.size() <<endl;
//    cout << "feaDim " << features[0].size() << " nSamples " << features.size() <<endl;

    // extracting gb features
    float sigma, hs;
    Size Orifbrsize = fbr[0].size();
    Mat des( CurrImgSize, CV_32FC1);
    Mat smallerfbr(CurrImgSize, fbr[0].type());
    Point anchor;
    vector<Mat> kernelHor(numSigma);
    vector<Mat> kernelVert(numSigma);
    for (unsigned int sigma_id=0; sigma_id< numSigma; sigma_id++){
            sigma = alpha*rs[sigma_id]+beta;
            hs = cvCeil(sigma)*3;

            kernelHor[sigma_id].create(1,hs*2+1,CV_32FC1);
            kernelVert[sigma_id].create(hs*2+1,1,CV_32FC1);
            float* kernelHorptr = (float*)kernelHor[sigma_id].ptr(0);
            int count = 0;
            float sum = 0;
            for (int k=-hs; k<= hs; k++){
                kernelHorptr[count] = exp(-(k*k)/(2*sigma*sigma));
                sum += kernelHorptr[count];
                count++;
            }
            for (int k=0; k< count; k++){
                kernelHorptr[k] = kernelHorptr[k]/sum;
                float* kernelVertptr = (float*)kernelVert[sigma_id].ptr(k);
                kernelVertptr[0] = kernelHorptr[k];
            }
    }


//        //show patches
//        Mat smallerfbr_sum(CurrImgSize, fbr[0].type());
//        float Max_fbr_sum = 0;



    for ( int bin_id=0; bin_id< nbins; bin_id++){
        // rescale fbr
        if (CurrImgSize == Orifbrsize)
            smallerfbr = Mat( CurrImgSize, fbr[bin_id].type(), fbr[bin_id].data, fbr[bin_id].step);
        else
            resize( fbr[bin_id], smallerfbr, CurrImgSize);

//        for (unsigned des_idy=0; des_idy< smallerfbr_sum.rows; des_idy++){
//            float* smallerfbr_ptr = (float*)smallerfbr.ptr(des_idy);
//            float* smallerfbr_sum_ptr = (float*)smallerfbr_sum.ptr(des_idy);
//            for (unsigned des_idx=0; des_idx< smallerfbr_sum.rows; des_idx++){
//                smallerfbr_sum_ptr[des_idx] += smallerfbr_ptr[des_idx];
//                if (smallerfbr_sum_ptr[des_idx]>Max_fbr_sum)
//                    Max_fbr_sum = smallerfbr_sum_ptr[des_idx];
//            }
//        }


//        cout << "smallerfbr size=" << smallerfbr.rows <<" " << smallerfbr.cols << endl;
        for (unsigned int sigma_id=0; sigma_id< numSigma; sigma_id++){

            anchor.x = kernelHor[sigma_id].cols/2+1;
            anchor.y = 0;
            filter2D( smallerfbr, des, -1,
               kernelHor[sigma_id], anchor,
               0, BORDER_REPLICATE);

            anchor.y = kernelVert[sigma_id].rows/2+1;
            anchor.x = 0;
            filter2D( des, des, -1,
            kernelVert[sigma_id], anchor,
            0, BORDER_REPLICATE);
//            for (unsigned des_idy=0; des_idy< des.rows; des_idy++){
//                float* des_ptr_new = (float*)des.ptr(des_idy);
//                for (unsigned des_idx=0; des_idx< des.rows; des_idx++){
//                    cout << " " << des_ptr_new[des_idx];
//                }
//            }
//            cout << endl;

//            cout << "des.cols" << des.cols << "des.rows" << des.rows << endl;
//            namedWindow("blur image", 1);
//            imshow("blur image", des);
////            namedWindow("edge image", 1);
////            imshow("edge image", smallerfbr);
//            cv::waitKey(0);
//            namedWindow("blur image", 1);
//            imshow("blur image", des);
//            cv::waitKey(0);

            // get sampled value
            for (unsigned int cid = 0; cid< roi_center.size(); cid++){
                for (unsigned int k=0; k<sample_points_sigma_level[sigma_id].size(); k++){
                    unsigned int sample_id = sample_points_sigma_level[sigma_id][k];
                    Point sample;
                    sample.x = cvRound(roi_center[cid].x+sample_points[sample_id].x);
                    sample.y = cvRound(roi_center[cid].y+sample_points[sample_id].y);
                    if (sample.x<0 || sample.y <0 || sample.x>= CurrImgSize.width || sample.y >= CurrImgSize.height){
                        sample.x = 1;
                        sample.y = 1;
                    }
                    float* desptr = (float*) des.ptr(sample.y);
                    features[nFeaOffset+cid].push_back(desptr[sample.x]);
                }
            }
        }
    }
}

void gb::compute_sample_points( vector<Point>& sample_points, vector< vector<int> >& sample_points_sigma_level){
    float theta;
    Point tmp_point;
    unsigned int sample_count = 0;

    sample_points_sigma_level.resize(rs.size());
    for ( unsigned int r_id=0; r_id< rs.size(); r_id++){
        for ( int theta_id=0; theta_id< nthetas[r_id]; theta_id++){
            theta = (theta_id+1)*2*CV_PI/nthetas[r_id];
            tmp_point.x = rs[r_id]*cos(theta);
            tmp_point.y = rs[r_id]*sin(theta);
            sample_points.push_back(tmp_point);
            sample_points_sigma_level[r_id].push_back( sample_count);
            sample_count++;
        }
    }
}
