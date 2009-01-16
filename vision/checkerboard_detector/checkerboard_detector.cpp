// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
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
#include "image_msgs/StereoInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "checkerboard_detector/ObjectDetection.h"
#include "math.h"

#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>

using namespace std;
using namespace ros;

class CheckerboardDetectorNode : public Node
{
public:
    struct CHECKERBOARD
    {
        CvSize griddims; ///< number of squares
        vector<Vector> grid3d;
        vector<CvPoint2D32f> corners;
    };

    image_msgs::CamInfo _caminfomsg;
    image_msgs::Image _imagemsg;
    checkerboard_detector::ObjectDetection _objdetmsg;
    image_msgs::CvBridge _cvbridge;
    string frame_id; // tf frame id
    
    int display, uidnext;
    vector<CHECKERBOARD> vcheckers; // grid points for every checkerboard
    vector< string > vstrtypes; // type names for every grid point
    map<string,int> maptypes;
    ros::Time lasttime;
    CvMat *intrinsic_matrix; // intrinsic matrices
    boost::mutex _mutexcalib;
    IplImage* frame;

    CheckerboardDetectorNode() : Node("checkerboard_detector"), uidnext(1), intrinsic_matrix(NULL), frame(NULL)
    {
        param("display",display,1);

        char str[32];
        int index = 0;
        while(1) {
            int dimx, dimy;
            double fRectSize[2];
            string type;

            sprintf(str,"grid%d_size_x",index);
            if( !get_param(str,dimx) )
                break;
            
            sprintf(str,"grid%d_size_y",index);
            if( !get_param(str,dimy) )
                break;

            sprintf(str,"rect%d_size_x",index);
            if( !get_param(str,fRectSize[0]) )
                break;

            sprintf(str,"rect%d_size_y",index);
            if( !get_param(str,fRectSize[1]) )
                break;

            sprintf(str,"type%d",index);
            if( !get_param(str,type) ) {
                sprintf(str,"checker%dx%d", dimx, dimy);
                type = str;
            }

            CHECKERBOARD cb;
            cb.griddims = cvSize(dimx,dimy);

            cb.grid3d.resize(dimx*dimy);
            int j=0;
            for(int y=0; y<dimy; ++y)
                for(int x=0; x<dimx; ++x)
                    cb.grid3d[j++] = Vector(x*fRectSize[0], y*fRectSize[1], 0);

            vcheckers.push_back(cb);

            vstrtypes.push_back(type);
            maptypes[vstrtypes.back()] = index;
            index++;
        }

        param("frame_id",frame_id,string(""));

        if( maptypes.size() == 0 ) {
            ROS_ERROR("no checkerboards to detect");
            return;
        }

        if( display ) {
            cvNamedWindow("Checkerboard Detector", CV_WINDOW_AUTOSIZE);
            cvStartWindowThread();
        }

        lasttime = ros::Time::now();
        advertise<checkerboard_detector::ObjectDetection>("ObjectDetection",1);
        subscribe("CamInfo", _caminfomsg, &CheckerboardDetectorNode::caminfo_cb,1);
        subscribe("Image", _imagemsg, &CheckerboardDetectorNode::image_cb,1);
    }
    ~CheckerboardDetectorNode()
    {
        if( frame )
            cvReleaseImage(&frame);
        if( intrinsic_matrix )
            cvReleaseMat(&intrinsic_matrix);
    }

    void caminfo_cb()
    {
        boost::mutex::scoped_lock lock(_mutexcalib);
        if( intrinsic_matrix == NULL )
            intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);

        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                intrinsic_matrix->data.fl[3*i+j] = _caminfomsg.P[4*i+j];
    }

    void image_cb()
    {
        boost::mutex::scoped_lock lock(_mutexcalib);
        if( intrinsic_matrix == NULL ) {
            ROS_ERROR("need CamInfo message for calibration info");
            return;
        }
        
        if( !_cvbridge.fromImage(_imagemsg, "mono") ) {
            ROS_ERROR("failed to get image");
            return;
        }

        IplImage *pimggray = _cvbridge.toIpl();
        if( display ) {
            // copy the raw image
            if( frame != NULL && (frame->width != _caminfomsg.width || frame->height != _caminfomsg.height) ) {
                cvReleaseImage(&frame);
                frame = NULL;
            }
            if( frame == NULL ) 
                frame = cvCreateImage(cvSize(_caminfomsg.width,_caminfomsg.height),IPL_DEPTH_8U, 3);

            cvCvtColor(pimggray,frame,CV_GRAY2RGB);
        }

        vector<checkerboard_detector::Object6DPose> vobjects;

        #pragma omp parallel for schedule(dynamic,1)
        for(size_t i = 0; i < vcheckers.size(); ++i) {
            CHECKERBOARD& cb = vcheckers[i];
            int ncorners;
            checkerboard_detector::Object6DPose objpose;

            // do until no more checkerboards detected
            while(1) {
                cb.corners.resize(200);
                int allfound = cvFindChessboardCorners( pimggray, cb.griddims, &cb.corners[0], &ncorners,
                                                        CV_CALIB_CB_ADAPTIVE_THRESH );
                cb.corners.resize(ncorners);
            
                //cvDrawChessboardCorners(pimgGray, itbox->second.griddims, &corners[0], ncorners, allfound);
                //cvSaveImage("temp.jpg", pimgGray);

                if(!allfound || ncorners != (int)cb.grid3d.size())
                    break;

                // remove any corners that are close to the border
                const int borderthresh = 30;
                for(int j = 0; j < ncorners; ++j) {
                    int x = cb.corners[j].x;
                    int y = cb.corners[j].y;
                    if( x < borderthresh || x > pimggray->width-borderthresh ||
                        y < borderthresh || y > pimggray->height-borderthresh ) {
                        allfound = 0;
                        break;
                    }
                }

                // mark out the image
                CvPoint upperleft, lowerright;
                upperleft.x = lowerright.x = cb.corners[0].x; 
                upperleft.y = lowerright.y = cb.corners[0].y; 
                for(int j = 1; j < (int)cb.corners.size(); ++j) {
                    if( upperleft.x > cb.corners[j].x ) upperleft.x = cb.corners[j].x;
                    if( upperleft.y > cb.corners[j].y ) upperleft.y = cb.corners[j].y;
                    if( lowerright.x < cb.corners[j].x ) lowerright.x = cb.corners[j].x;
                    if( lowerright.y < cb.corners[j].y ) lowerright.y = cb.corners[j].y;
                }

                if( allfound ) {
                    
                    cvFindCornerSubPix(pimggray, &cb.corners[0], cb.corners.size(), cvSize(5,5),cvSize(-1,-1),
                                       cvTermCriteria(CV_TERMCRIT_ITER,20,1e-2));
                    
                    objpose.pose = FindTransformation(cb.corners, cb.grid3d);
                }

                #pragma omp critical
                {
                    if( allfound ) {
                        objpose.uid = uidnext++;
                        vobjects.push_back(objpose);
                        vobjects.back().type = vstrtypes[i];
                    }
                    
                    cvRectangle(pimggray, upperleft, lowerright, CV_RGB(0,0,0),CV_FILLED);
                }
            }
            
            //cvSaveImage("temp.jpg", pimggray);
        }
        
        _objdetmsg.set_objects_vec(vobjects);
        _objdetmsg.header.frame_id = frame_id;
        publish("ObjectDetection", _objdetmsg);

        ROS_INFO("checkerboard: image: %dx%d (size=%d), num: %d, total: %.3fs",_caminfomsg.width,_caminfomsg.height,
                _imagemsg.uint8_data.data.size(), _objdetmsg.get_objects_size(),
                (float)(ros::Time::now()-lasttime).toSec());
        lasttime = ros::Time::now();
        
        if( display ) {
            
            // draw each found checkerboard
            for(size_t i = 0; i < vobjects.size(); ++i) {
                int itype = maptypes[vobjects[i].type];
                CHECKERBOARD& cb = vcheckers[itype];
                CvSize& s = cb.griddims;
                Transform tlocal;
                tlocal.trans = Vector(vobjects[i].pose.position.x,vobjects[i].pose.position.y,vobjects[i].pose.position.z);
                tlocal.rot = Vector(vobjects[i].pose.orientation.w,vobjects[i].pose.orientation.x,vobjects[i].pose.orientation.y, vobjects[i].pose.orientation.z);

                CvPoint X[4];
                
                int inds[4] = {0, s.width-1, s.width*(s.height-1), s.width*s.height-1 };

                for(int i = 0; i < 4; ++i) {
                    Vector p = tlocal * cb.grid3d[inds[i]];
                    dReal fx = p.x*_caminfomsg.P[0] + p.y*_caminfomsg.P[1] + p.z*_caminfomsg.P[2] + _caminfomsg.P[3];
                    dReal fy = p.x*_caminfomsg.P[4] + p.y*_caminfomsg.P[5] + p.z*_caminfomsg.P[6] + _caminfomsg.P[7];
                    dReal fz = p.x*_caminfomsg.P[8] + p.y*_caminfomsg.P[9] + p.z*_caminfomsg.P[10] + _caminfomsg.P[11];
                    X[i].x = (int)(fx/fz);
                    X[i].y = (int)(fy/fz);
                }

                // draw two lines
                CvScalar col0 = CV_RGB(255,0,(64*itype)%256);
                CvScalar col1 = CV_RGB(0,255,(64*itype)%256);
                cvLine(frame, X[0], X[1], col0, 1);
                cvLine(frame, X[0], X[2], col1, 1);

                // draw all the points
                for(size_t i = 0; i < cb.grid3d.size(); ++i) {
                    Vector p = tlocal * cb.grid3d[i];
                    dReal fx = p.x*_caminfomsg.P[0] + p.y*_caminfomsg.P[1] + p.z*_caminfomsg.P[2] + _caminfomsg.P[3];
                    dReal fy = p.x*_caminfomsg.P[4] + p.y*_caminfomsg.P[5] + p.z*_caminfomsg.P[6] + _caminfomsg.P[7];
                    dReal fz = p.x*_caminfomsg.P[8] + p.y*_caminfomsg.P[9] + p.z*_caminfomsg.P[10] + _caminfomsg.P[11];
                    int x = (int)(fx/fz);
                    int y = (int)(fy/fz);
                    cvCircle(frame, cvPoint(x,y), 6, CV_RGB(0,0,0), 2);
                    cvCircle(frame, cvPoint(x,y), 2, CV_RGB(0,0,0), 2);
                    cvCircle(frame, cvPoint(x,y), 4, CV_RGB(128,128,64*itype), 3);
                }

                cvCircle(frame, X[0], 3, CV_RGB(255,255,128), 3);
            }

            cvShowImage("Checkerboard Detector",frame);
        }
    }

    std_msgs::Pose FindTransformation(const vector<CvPoint2D32f> &imgpts, const vector<Vector> &objpts)
    {
        CvMat *objpoints = cvCreateMat(3,objpts.size(),CV_32FC1);
        for(size_t i=0; i<objpts.size(); ++i) {
            cvSetReal2D(objpoints, 0,i, objpts[i].x);
            cvSetReal2D(objpoints, 1,i, objpts[i].y);
            cvSetReal2D(objpoints, 2,i, objpts[i].z);
        }
        
        std_msgs::Pose pose;
        float fR3[3];
        CvMat R3, T3;
        assert(sizeof(pose.position.x) == sizeof(double));
        cvInitMatHeader(&R3, 3, 1, CV_32FC1, fR3);
        cvInitMatHeader(&T3, 3, 1, CV_64FC1, &pose.position.x);
        
        // for some reason distortion coeffs are needed
        float kc[4] = {0};
        CvMat kcmat;
        cvInitMatHeader(&kcmat,1,4,CV_32FC1,kc);

        CvMat img_points;
        cvInitMatHeader(&img_points, 1,imgpts.size(), CV_32FC2, const_cast<CvPoint2D32f*>(imgpts.data()));
        
        cvFindExtrinsicCameraParams2(objpoints, &img_points, intrinsic_matrix, &kcmat, &R3, &T3);
        cvReleaseMat(&objpoints);
        
        double fang = sqrt(fR3[0]*fR3[0] + fR3[1]*fR3[1] + fR3[2]*fR3[2]);
        if( fang < 1e-6 ) {
            pose.orientation.w = 1;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
        }
        else {
            double fmult = sin(fang/2)/fang;
            pose.orientation.w = cos(fang/2);
            pose.orientation.x = fR3[0]*fmult;
            pose.orientation.y = fR3[1]*fmult;
            pose.orientation.z = fR3[2]*fmult;
        }

        return pose;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv);
    CheckerboardDetectorNode checker;
    checker.spin();
    ros::fini();
    return 0;
}
