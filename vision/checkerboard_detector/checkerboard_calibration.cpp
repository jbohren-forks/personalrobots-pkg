// Software License Agreement (BSD License)
// Copyright (c) 2008, Rosen Diankov
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// author: Rosen Diankov
#include <cstdio>
#include <vector>
#include <ros/node.h>

#include <boost/thread/mutex.hpp>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "image_msgs/CvBridge.h"
#include "image_msgs/Image.h"
#include "math.h"

#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>

using namespace std;
using namespace ros;

template <typename T>
T SQR(T t) { return t*t; }

boost::shared_ptr<ros::Node> s_pmasternode;

class CheckerboardCalibration
{
public:
    struct CHECKERBOARD
    {
        CvSize griddims; ///< number of squares
        vector<RaveVector<float> > grid3d;
        vector<CvPoint2D32f> corners;
    };

    image_msgs::Image _imagemsg;
    image_msgs::CvBridge _cvbridge;
    
    int display;
    CHECKERBOARD _checkerboard; // grid points for every checkerboard
    IplImage* frame;
    vector<CvPoint2D32f> _vAllPoints; ///< all checkerboards found
    vector<int> _vNumPointsPerImage;

    CvMat *_intrinsic_matrix, *_distortion_coeffs; // intrinsic matrices
    TransformMatrix _tProjection; // project matrix
    bool _bHasCalibration, _bClearData, _bTakeObservation;

    CheckerboardCalibration() : frame(NULL), _intrinsic_matrix(NULL), _distortion_coeffs(NULL), _bHasCalibration(false), _bClearData(false), _bTakeObservation(false)
    {
        s_pmasternode->param("display",display,1);

        int dimx, dimy;
        double fRectSize[2];
        string type;
        
        if( !s_pmasternode->getParam("grid_size_x",dimx) ) {
            ROS_ERROR("grid_size_x not defined");
            throw;
        }
        if( !s_pmasternode->getParam("grid_size_y",dimy) ) {
            ROS_ERROR("grid_size_y not defined");
            throw;
        }
        if( !s_pmasternode->getParam("rect_size_x",fRectSize[0]) ) {
            ROS_ERROR("rect_size_x not defined");
            throw;
        }
        if( !s_pmasternode->getParam("rect_size_y",fRectSize[1]) ) {
            ROS_ERROR("rect_size_y not defined");
            throw;
        }

        _checkerboard.griddims = cvSize(dimx,dimy);
        _checkerboard.grid3d.resize(dimx*dimy);
        int j=0;
        for(int y=0; y<dimy; ++y)
            for(int x=0; x<dimx; ++x)
                _checkerboard.grid3d[j++] = Vector(x*fRectSize[0], y*fRectSize[1], 0);

        if( _intrinsic_matrix == NULL )
            _intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
        if( _distortion_coeffs == NULL )
            _distortion_coeffs = cvCreateMat(4,1,CV_32FC1);

        if( display ) {
            cvNamedWindow("Checkerboard Detector", CV_WINDOW_AUTOSIZE);
            cvSetMouseCallback("Checkerboard Detector", MouseCallback, this);
            cvNamedWindow("Calibration Result", CV_WINDOW_AUTOSIZE);
            cvSetMouseCallback("Calibration Result", MouseCallback, this);
            cvStartWindowThread();
        }

        s_pmasternode->subscribe("Image", _imagemsg, &CheckerboardCalibration::image_cb,this,1);
    }
    ~CheckerboardCalibration()
    {
        if( frame )
            cvReleaseImage(&frame);
        if( _intrinsic_matrix )
            cvReleaseMat(&_intrinsic_matrix);
        if( _distortion_coeffs )
            cvReleaseMat(&_distortion_coeffs);

        s_pmasternode->unsubscribe("Image");
    }

private:
    void image_cb()
    {
        if( !_cvbridge.fromImage(_imagemsg, "mono") ) {
            ROS_ERROR("failed to get image");
            return;
        }

        IplImage *pimggray = _cvbridge.toIpl();
        if( display ) {
            // copy the raw image
            if( frame != NULL && (frame->width != (int)pimggray->width || frame->height != (int)pimggray->height) ) {
                cvReleaseImage(&frame);
                frame = NULL;
            }
            if( frame == NULL ) 
                frame = cvCreateImage(cvSize(pimggray->width,pimggray->height),IPL_DEPTH_8U, 3);

            cvCvtColor(pimggray,frame,CV_GRAY2RGB);
        }

        int ncorners;
        
        // do until no more checkerboards detected
        _checkerboard.corners.resize(_checkerboard.grid3d.size()+64);
        int allfound = cvFindChessboardCorners( pimggray, _checkerboard.griddims, &_checkerboard.corners[0],
                                                &ncorners, CV_CALIB_CB_ADAPTIVE_THRESH );
        _checkerboard.corners.resize(ncorners);        
        
        if(allfound && ncorners == (int)_checkerboard.grid3d.size()) {
        
            cvFindCornerSubPix(pimggray, &_checkerboard.corners[0], _checkerboard.corners.size(), cvSize(5,5),cvSize(-1,-1),
                               cvTermCriteria(CV_TERMCRIT_ITER,20,1e-2));
        }
        else
            _checkerboard.corners.resize(0);

        if( _bClearData ) {
            ROS_INFO("clearing current calibration data");
            _vAllPoints.resize(0);
            _vNumPointsPerImage.resize(0);
            _bHasCalibration = false;
            _bClearData = false;
        }

        if( _bTakeObservation ) {
            _bTakeObservation = false;

            if(allfound && ncorners == (int)_checkerboard.grid3d.size()) {
                
                for(vector<CvPoint2D32f>::iterator it = _checkerboard.corners.begin(); it != _checkerboard.corners.end(); ++it)
                    _vAllPoints.push_back(*it);
                _vNumPointsPerImage.push_back(_checkerboard.corners.size());
                
                ROS_INFO("calibrating");
            
                CvMat object_points, image_points, point_counts;
            
                vector<float> vobject_points;
                vobject_points.reserve(_vNumPointsPerImage.size()*3*_checkerboard.grid3d.size());
                for(size_t j = 0; j < _vNumPointsPerImage.size(); ++j) {
                    for(size_t i = 0; i < _checkerboard.grid3d.size(); ++i) {
                        vobject_points.push_back(_checkerboard.grid3d[i].x);
                        vobject_points.push_back(_checkerboard.grid3d[i].y);
                        vobject_points.push_back(_checkerboard.grid3d[i].z);
                    }
                }

                cvInitMatHeader(&object_points, vobject_points.size()/3,3,CV_32FC1, &vobject_points[0]);
                cvInitMatHeader(&image_points, _vAllPoints.size(), 2, CV_32FC1, &_vAllPoints[0]);
                cvInitMatHeader(&point_counts, _vNumPointsPerImage.size(), 1, CV_32SC1, &_vNumPointsPerImage[0]);
                
                CvMat* rotation_vectors = cvCreateMat(_vNumPointsPerImage.size(),3,CV_32FC1);
                CvMat* translation_vectors = cvCreateMat(_vNumPointsPerImage.size(),3,CV_32FC1);
                
                cvCalibrateCamera2(&object_points, &image_points,
                                   &point_counts, cvSize(frame->width,frame->height),
                                   _intrinsic_matrix, _distortion_coeffs,
                                   rotation_vectors, translation_vectors, 0);

                double err = 0;
                int off = 0;
                for(size_t i = 0; i < _vNumPointsPerImage.size(); ++i) {
                    CvMat cur_object_points, rotation_vector, translation_vector;

                    cvInitMatHeader(&cur_object_points, _vNumPointsPerImage[i],3,CV_32FC1, &vobject_points[0]);
                    cvInitMatHeader(&rotation_vector, 3, 1, CV_32FC1, &rotation_vectors->data.fl[3*i]);
                    cvInitMatHeader(&translation_vector, 3, 1, CV_32FC1, &translation_vectors->data.fl[3*i]);
                    CvMat* new_image_points = cvCreateMat(_vNumPointsPerImage[i], 2, CV_32FC1);
                    cvProjectPoints2(&cur_object_points, &rotation_vector, &translation_vector, _intrinsic_matrix,
                                     _distortion_coeffs, new_image_points);

                    for(int j = 0; j < _vNumPointsPerImage[i]; ++j)
                        err += SQR(new_image_points->data.fl[2*j]-_vAllPoints[off+j].x) + SQR(new_image_points->data.fl[2*j+1]-_vAllPoints[off+j].y);

                    cvReleaseMat(&new_image_points);
                    off += _vNumPointsPerImage[i];
                }
                err = sqrt(err)/_vAllPoints.size();

                ROS_INFO("calibration done, reprojection error = %f", (float)err);
                ROS_INFO("Intrinsic Matrix:");
                for(int i = 0; i < 3; ++i) {
                    ROS_INFO("%15f %15f %15f", _intrinsic_matrix->data.fl[3*i], _intrinsic_matrix->data.fl[3*i+1], _intrinsic_matrix->data.fl[3*i+2]);
                    _tProjection.m[4*i+0] = _intrinsic_matrix->data.fl[3*i+0];
                    _tProjection.m[4*i+1] = _intrinsic_matrix->data.fl[3*i+1];
                    _tProjection.m[4*i+2] = _intrinsic_matrix->data.fl[3*i+2];
                }
                ROS_INFO("distortion: %f %f %f %f", _distortion_coeffs->data.fl[0], _distortion_coeffs->data.fl[1], _distortion_coeffs->data.fl[2], _distortion_coeffs->data.fl[3]);

                cvReleaseMat(&rotation_vectors);
                cvReleaseMat(&translation_vectors);
                _bHasCalibration = true;
            }
            else
                ROS_WARN("no checkerboard found");
        }

        if( display ) {            
            if( _bHasCalibration ) {
                IplImage* frameres = cvCloneImage(frame);
                RaveTransform<float> tlocal = FindTransformation(_checkerboard.corners, _checkerboard.grid3d);

                CvSize& s = _checkerboard.griddims;
                CvPoint X[4];
                int inds[4] = {0, s.width-1, s.width*(s.height-1), s.width*s.height-1 };
            
                for(int i = 0; i < 4; ++i) {
                    Vector p = _tProjection * tlocal * _checkerboard.grid3d[inds[i]];
                    X[i].x = (int)(p.x/p.z);
                    X[i].y = (int)(p.y/p.z);
                }
                
                // draw two lines
                CvScalar col0 = CV_RGB(255,0,0);
                CvScalar col1 = CV_RGB(0,255,0);
                cvLine(frameres, X[0], X[1], col0, 1);
                cvLine(frameres, X[0], X[2], col1, 1);
                
                // draw all the points
                for(size_t i = 0; i < _checkerboard.grid3d.size(); ++i) {
                    Vector p = _tProjection * tlocal * _checkerboard.grid3d[i];
                    int x = (int)(p.x/p.z);
                    int y = (int)(p.y/p.z);
                    cvCircle(frameres, cvPoint(x,y), 6, CV_RGB(0,0,0), 2);
                    cvCircle(frameres, cvPoint(x,y), 2, CV_RGB(0,0,0), 2);
                    cvCircle(frameres, cvPoint(x,y), 4, CV_RGB(128,128,0), 3);
                }
                
                cvCircle(frameres, X[0], 3, CV_RGB(255,255,128), 3);

                cvShowImage("Calibration Result",frameres);
                cvReleaseImage(&frameres);
            }

            if(allfound && ncorners == (int)_checkerboard.grid3d.size())
                cvDrawChessboardCorners(frame, _checkerboard.griddims, &_checkerboard.corners[0], _checkerboard.corners.size(), 1);
            
            cvShowImage("Checkerboard Detector",frame);
        }
    }

    Transform FindTransformation(const vector<CvPoint2D32f> &imgpts, const vector<Vector> &objpts)
    {
        CvMat *objpoints = cvCreateMat(3,objpts.size(),CV_32FC1);
        for(size_t i=0; i<objpts.size(); ++i) {
            cvSetReal2D(objpoints, 0,i, objpts[i].x);
            cvSetReal2D(objpoints, 1,i, objpts[i].y);
            cvSetReal2D(objpoints, 2,i, objpts[i].z);
        }
        
        RaveTransform<float> pose;
        float fR3[3];
        CvMat R3, T3;
        cvInitMatHeader(&R3, 3, 1, CV_32FC1, fR3);
        cvInitMatHeader(&T3, 3, 1, CV_32FC1, &pose.trans.x);
        
        // for some reason distortion coeffs are needed
        CvMat img_points;
        cvInitMatHeader(&img_points, 1,imgpts.size(), CV_32FC2, const_cast<CvPoint2D32f*>(&imgpts[0]));
    
        cvFindExtrinsicCameraParams2(objpoints, &img_points, _intrinsic_matrix, _distortion_coeffs, &R3, &T3);
        cvReleaseMat(&objpoints);
        
        double fang = sqrt(fR3[0]*fR3[0] + fR3[1]*fR3[1] + fR3[2]*fR3[2]);
        if( fang < 1e-6 ) {
            pose.rot.x = 1;
            pose.rot.y = 0;
            pose.rot.z = 0;
            pose.rot.w = 0;
        }
        else {
            double fmult = sin(fang/2)/fang;
            pose.rot.x = cos(fang/2);
            pose.rot.y = fR3[0]*fmult;
            pose.rot.z = fR3[1]*fmult;
            pose.rot.w = fR3[2]*fmult;
        }

        return pose;
    }

    static void MouseCallback(int event, int x, int y, int flags, void* param)
    {
        ((CheckerboardCalibration*)param)->_MouseCallback(event, x, y, flags);
    }

    void _MouseCallback(int event, int x, int y, int flags)
    {
        switch(event) {
        case CV_EVENT_RBUTTONDOWN:
            _bClearData = true;
            break;
        case CV_EVENT_MBUTTONDOWN:
            _bTakeObservation = true;
            break;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv);
    s_pmasternode.reset(new ros::Node("checkerboard_calibration"));

    if( !s_pmasternode->checkMaster() )
        return -1;
    
    boost::shared_ptr<CheckerboardCalibration> checker(new CheckerboardCalibration());
    
    s_pmasternode->spin();
    checker.reset();
    s_pmasternode.reset();
    return 0;
}
