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

// Author: Min Sun

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "topic_synchronizer2/topic_synchronizer.h"
#include <sstream>
#include <string>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "write.h"
#include "hog.h"
#include "ObjectInPerspective.h"

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/projections.h>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace cv;
using namespace std;
int NView = 3;
float fgbgSampleRatio = 0.5;
int MAX_CharBuffer = 100;
float obj_physical_height = 0.20;//in meter

class rf_detector
{

public:
    ros::NodeHandle n;

    Mat left;
    Mat left_objbbx;
    string RFmodel_filename;
    string FgBgProb_filename;
    string ViewIdBinary_filename;
    string NormVotesBinary_filename;
    int MaxViewId;
    double Scale; //was float
    double hitThreshold;
    bool VisualizeFlag;
    double ObjHeight2winHeightRatio; // was float
    int MeanShiftMaxIter;
    Size winSize;
    Size winStride;
    Size numCellPerWin;
    Size numCellPerBlock;
    Size padding;

    bool ObjectInPerspectiveFlag;
    int nCandPerView;

    tf::TransformListener tf_;
	tf::TransformBroadcaster broadcaster_;
    TopicSynchronizer sync_;
    sensor_msgs::ImageConstPtr limage;
    sensor_msgs::CvBridge lbridge;
    sensor_msgs::CameraInfoConstPtr lcinfo_;

    ros::Subscriber left_image_sub_;
    ros::Subscriber left_caminfo_image_sub_;
    ros::Subscriber cloud_sub_;

    sensor_msgs::PointCloudConstPtr cloud;
    float camera_height;
    float horizontal_line_row;

    // rf related variables
    HogFast hog;
    float ObjHeight2sinStrideRatio;
    Size cellSize;
    Size blockSize;
    Size blockStride;
    int nbins;
    int derivAperture;
    double winSigma;
    int histogramNormType;
    double L2HysThreshold;
    bool gammaCorrection;
    int groupThreshold;
    vector<double> AvergeKernel;
    int nCandidatePerDim;

    // readin binary files
    vector< vector< float> > vecNotmVotes;
    vector< int> vecViewIds;
    float BiggestWidth2HeightRatio;

    rf_detector(): sync_(&rf_detector::syncCallback, this)
    {
    // get parameters
    n.getParam( (string) "/rf_detector/RFmodel_filename", RFmodel_filename, true);
    n.getParam( (string) "/rf_detector/FgBgProb_filename", FgBgProb_filename, true);
    n.getParam( (string) "/rf_detector/ViewIdBinary_filename", ViewIdBinary_filename, true);
    n.getParam( (string) "/rf_detector/NormVotesBinary_filename", NormVotesBinary_filename, true);
    n.getParam( (string) "/rf_detector/MaxViewId", MaxViewId, true);
    n.getParam( (string) "/rf_detector/Scale", Scale, true);
    n.getParam( (string) "/rf_detector/hitThreshold", hitThreshold, true);
    n.getParam( (string) "/rf_detector/VisualizeFlag", VisualizeFlag, true);
    n.getParam( (string) "/rf_detector/ObjHeight2winHeightRatio", ObjHeight2winHeightRatio, true);
    n.getParam( (string) "/rf_detector/MeanShiftMaxIter", MeanShiftMaxIter, true);
    n.getParam( (string) "/rf_detector/winSize_width", winSize.width, true);
    n.getParam( (string) "/rf_detector/winSize_height", winSize.height, true);
    n.getParam( (string) "/rf_detector/winStride_width", winStride.width, true);
    n.getParam( (string) "/rf_detector/winStride_height", winStride.height, true);
    n.getParam( (string) "/rf_detector/numCellPerWin_width", numCellPerWin.width, true);
    n.getParam( (string) "/rf_detector/numCellPerWin_height", numCellPerWin.height, true);
    n.getParam( (string) "/rf_detector/numCellPerBlock_width", numCellPerBlock.width, true);
    n.getParam( (string) "/rf_detector/numCellPerBlock_height", numCellPerBlock.height, true);
    n.getParam( (string) "/rf_detector/padding_width", padding.width, true);
    n.getParam( (string) "/rf_detector/padding_height", padding.height, true);
    n.getParam( (string) "/rf_detector/ObjectInPerspectiveFlag", ObjectInPerspectiveFlag, true);
    n.getParam( (string) "/rf_detector/nCandPerView", nCandPerView, true);

    cout << "RFmodel_filename "<< RFmodel_filename << endl;
    cout << "VisualizeFlag  "<< VisualizeFlag << endl;
    cout << "padding_height "<< padding.height << endl;

    cout << "Finish handle input" << endl;


    ObjHeight2sinStrideRatio = (float)winSize.height*ObjHeight2winHeightRatio/winStride.height;

        // setup hog parameters
    cellSize.width = cvFloor(winSize.width/numCellPerWin.width);
    cellSize.height = cvFloor(winSize.height/numCellPerWin.height);
    blockSize.width = cvFloor(cellSize.width*numCellPerBlock.width);
    blockSize.height = cvFloor(cellSize.height*numCellPerBlock.height);
    blockStride = cellSize;
    nbins =  9;
    derivAperture = 1;
    winSigma=-1;
    histogramNormType=0;
    L2HysThreshold=0.2;
    gammaCorrection=true;
    groupThreshold = 0;
    AvergeKernel.push_back(0.25);
    AvergeKernel.push_back(0.5);
    nCandidatePerDim = 5;

    // readin binary files
    readFloattext( NormVotesBinary_filename, vecNotmVotes);
    readIntegertext( ViewIdBinary_filename, vecViewIds);
    BiggestWidth2HeightRatio = 0;
    for (unsigned int i=0; i< vecNotmVotes.at(2).size();i++){
        if ( vecNotmVotes.at(2).at(i) > BiggestWidth2HeightRatio)
            BiggestWidth2HeightRatio = vecNotmVotes.at(2).at(i);
    }
    cout << "BiggestWidth2HeightRatio "<< BiggestWidth2HeightRatio << endl;
//    cout << vecNotmVotes.size() << " " << vecNotmVotes.at(0).size() << endl;
//    cout << vecViewIds.size() << endl;

    // initialize the hog descriptor
    hog.winSize = winSize;
    hog.blockSize = blockSize;
    hog.blockStride = blockStride;
    hog.cellSize = cellSize;
    hog.nbins = nbins;
    hog.derivAperture = derivAperture;
    hog.winSigma = winSigma;
    hog.histogramNormType = histogramNormType;
    hog.L2HysThreshold = L2HysThreshold;
    hog.gammaCorrection = gammaCorrection;
    cout << "Finish initialize hog"<< endl;

    // load the RF model
    hog.setRFDetector( RFmodel_filename, FgBgProb_filename);

    namedWindow("left_objbbx", 1);
    // subscribe to topics
    left_image_sub_ = n.subscribe("stereo/left/image_rect", 1, sync_.synchronize(&rf_detector::leftImageCallback, this));
    left_caminfo_image_sub_ = n.subscribe("stereo/left/cam_info", 1, sync_.synchronize(&rf_detector::leftCameraInfoCallback, this));
    cloud_sub_ = n.subscribe("stereo/cloud", 1, sync_.synchronize(&rf_detector::cloudCallback, this));

    }

    bool spin()
    {
            while (n.ok())
            {
                    ros::spinOnce();
            }

            return true;
    }

private:

    void leftImageCallback(const sensor_msgs::Image::ConstPtr& image)
    {
            limage = image;
            if(lbridge.fromImage(*limage, "bgr")) {
                cout << " left image call back"<<endl;
                    left = Mat(lbridge.toIpl(), false);
                    left_objbbx = left.clone();
            }
    }

    void leftCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
	{
		lcinfo_ = info;
	}

    void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
	{
		cloud = point_cloud;
	}

    void syncCallback()
    {
        if (ObjectInPerspectiveFlag){
            runObjectInPerspectiveDetection();
        }else{
            vector<Rect> objBbxes;
            vector<float> objBbxesConf;
            vector<int> viewIds;
            Rect roi;
            roi.x = 0;
            roi.y = 0;
            double startScale=1.;
            double endScale=1000.;
            run_rf_detector(left, nCandPerView, startScale, endScale, objBbxes, objBbxesConf, viewIds);
//            cout << "going to plot det" << endl;
            PlotDetection(left_objbbx, roi, objBbxes, objBbxesConf, viewIds);
        }
        cout << "syncCallback" << endl;
    }

    void PlotDetection(Mat& left_objbbx, Rect roi, vector<Rect>& objBbxes, vector<float>& objBbxesConf, vector<int> viewIds){
        char buffer [50];
        cout << "number bbxes " << objBbxes.size() << endl;
        for (unsigned int j=0; j< objBbxes.size(); j++){
//                cout << "draw rect "<< endl;
                sprintf(buffer,"C:%.5g,V:%d",objBbxesConf[j],viewIds[j]);
                objBbxes[j].x+=roi.x;
                objBbxes[j].y+=roi.y;
                rectangle(left_objbbx, objBbxes[j].tl(), objBbxes[j].br(), Scalar(0,cvFloor(255*objBbxesConf[j]/objBbxesConf[0]),0), 1);
                putText( left_objbbx, buffer, objBbxes[j].tl(),  CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255));
		    }
    }

    void CheckRange(int& Value, int MinValue, int MaxValue){
        if ( Value < MinValue){
            Value = MinValue;
        }else if( Value > MaxValue){
            Value = MaxValue;
        }
    }

    void runObjectInPerspectiveDetection(){

        //test rf tree
//        cout << "num of trees " << hog.rf.GetNumTrees() << endl;

        vector<CvPoint> positions;
		vector<CvPoint> obj_bottom;
		vector<float> scales;
		vector<float> scales_msun;
		float camera_height = findObjectPositionsFromStereo(*cloud, positions, obj_bottom, scales, scales_msun, tf_, broadcaster_, *lcinfo_);
        cout << "camera_height " << camera_height << endl;

        vector< vector<Rect> > objBbxes;
        vector< vector<float> > objBbxesConf;
        vector< vector<int> > viewIds;
        objBbxes.resize(positions.size());
        objBbxesConf.resize(positions.size());
        viewIds.resize(positions.size());
		for (unsigned int i=0; i<positions.size(); i++){
		    // getting roi
            Point pt_tl;
            Point pt_br;
            Rect roi;
            int ObjCanonicalHeight = (int)scales_msun[i]*obj_physical_height;
            roi.height = ObjCanonicalHeight;//*pow(Scale,3);//hack 2
            cout << "ObjCanHeight " << ObjCanonicalHeight << "ObjBiggesHeight " << roi.height << endl;
            // set scaleStart scaleEnd
            double startScale = ObjCanonicalHeight/ObjHeight2winHeightRatio/winSize.height;
            double endScale = roi.height/ObjHeight2winHeightRatio/winSize.height;
            if (endScale <1.)
                continue;
            if (startScale <1.)
                startScale = 1.;

            roi.y = obj_bottom[i].y-roi.height;
            roi.width = roi.height*BiggestWidth2HeightRatio;
            roi.x = obj_bottom[i].x-roi.width/2;
            pt_tl.x = roi.x;
            pt_tl.y = roi.y;
            pt_br.x = roi.x+roi.width;
            pt_br.y = roi.y+roi.height;
            // check if roi
            Size leftSize = left.size();
            CheckRange(pt_tl.x,0,leftSize.width-1);
            CheckRange(pt_br.x,0,leftSize.width-1);
            CheckRange(pt_tl.y,0,leftSize.height-1);
            CheckRange(pt_br.y,0,leftSize.height-1);
            roi.x = pt_tl.x;
            roi.y = pt_tl.y;
            roi.width = pt_br.x-pt_tl.x;
            roi.height = pt_br.y-pt_tl.y;
            if (roi.width < winSize.width|| roi.height < winSize.height){
                continue;
            }

		    // get focused_img
            Mat focused_img( left, roi);
            rectangle(left_objbbx, roi.tl(), roi.br(), Scalar(255,0,0), 1);

		    // modify scale search range
		    cout << "iput scale" << startScale << " "<< endScale << endl;
		    run_rf_detector(focused_img, nCandPerView, startScale, endScale, objBbxes[i], objBbxesConf[i], viewIds[i]);

//            cout << "going to plot det" << endl;
            PlotDetection(left_objbbx, roi, objBbxes[i], objBbxesConf[i], viewIds[i]);
        }
        imshow("left_objbbx", left_objbbx);
        cvWaitKey(500);
    }

    void run_rf_detector(Mat& img, int nCandPerView, double startScale, double endScale, vector<Rect>& objBbxes, vector<float>& objBbxesConf, vector<int>& viewIds)
    {

        Vector< Rect > found;
        Vector< Point > foundIds;
        vector< float > foundConfs;
        vector< int> ScaleChangeBoundary;
        double t = (double)cv::getTickCount();
        // run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog.detectMultiScale(img, found, foundIds, foundConfs, ScaleChangeBoundary, hitThreshold, winStride, padding,
            Scale, startScale, endScale, groupThreshold, ObjHeight2winHeightRatio);
        t = (double)cv::getTickCount() - t;
        printf("%d features plus RF time = %gms\n", found.size(),t*1000./cv::getTickFrequency());

        // get data ready for voting
        vector< Vector<Rect> > ObjCenterWindowsAll;
        vector< vector<float> > ObjCenterConfAll;
        t = (double)cv::getTickCount();
        hog.cast_vots( img, found, foundIds, foundConfs, ScaleChangeBoundary,
            vecNotmVotes, vecViewIds, MaxViewId, AvergeKernel, nCandidatePerDim,
             winStride, ObjHeight2sinStrideRatio, Scale, startScale, MeanShiftMaxIter, ObjCenterWindowsAll,
             ObjCenterConfAll, VisualizeFlag);
        t = (double)cv::getTickCount() - t;
        printf("voting time = %gms\n", t*1000./cv::getTickFrequency());

        vector<int> ObjCenterInd;
        vector<Rect> objBbxes_;
        vector<float> objBbxesConf_;
        vector<int> viewIds_;

        Mat ImgClone;
        for (int viewId = 0; viewId < MaxViewId; viewId++){
            // sort the ObjCenter by Conf
            ObjCenterInd.clear();
            //msun debug
            cout << "num cand bbx per view " << ObjCenterConfAll[viewId].size() << endl;
            for ( unsigned int i = 0; i< ObjCenterConfAll[viewId].size(); i++)
                ObjCenterInd.push_back(i);
            vector<float>* tmp = &ObjCenterConfAll[viewId];
            sort( ObjCenterInd.begin(), ObjCenterInd.end(), index_cmp<vector<float>&>(*(tmp)) );

            int nCandPerView_new;

            if (nCandPerView >= (int) ObjCenterConfAll[viewId].size()){
                nCandPerView_new = (int) ObjCenterConfAll[viewId].size();
            }else{
                nCandPerView_new = nCandPerView;
            }
//            cout << "number detection "<< ObjCenterConfAll[viewId].size() << "nCandPerView_new "<< nCandPerView_new << endl;
            for( int i = 0; i < nCandPerView_new;i++)//(int)ObjCenterWindowsAll[viewId].size(); i++ )//
            {

                objBbxes_.push_back( ObjCenterWindowsAll[viewId][ObjCenterInd[i]]);
                objBbxesConf_.push_back(ObjCenterConfAll[viewId][ObjCenterInd[i]]);
                viewIds_.push_back(viewId);
                //msun debug
//                cout << "viewId "<< viewId <<"candidate " << i << endl;
//                sprintf(buffer,"C:%.5g,V:%d",ObjCenterConfAll[viewId][ObjCenterInd[i]],viewId);
//                ImgClone = img.clone();
//                Rect r =
//                rectangle(ImgClone, r.tl(), r.br(), Scalar(0,255,0), 1);
//                putText( ImgClone, buffer, r.tl(),  CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255));
//                namedWindow("detection", 1);
//                imshow("detection", ImgClone);
//                cv::waitKey(0);
            }

        }

//        cout << "number bbxes " << objBbxesConf_.size() << endl;
//        cout << "number viewIds " << viewIds_.size() << endl;
        ObjCenterInd.clear();
        for ( unsigned int i = 0; i< objBbxesConf_.size(); i++)
            ObjCenterInd.push_back(i);
        vector<float>* tmp = &objBbxesConf_;
        sort( ObjCenterInd.begin(), ObjCenterInd.end(), index_cmp<vector<float>&>(*(tmp)) );
        // sort the objBbxes
//        cout << "sorting objBbxes" << endl;
        for ( unsigned int i = 0; i< objBbxesConf_.size(); i++){
            objBbxes.push_back( objBbxes_[ObjCenterInd[i]]);
            objBbxesConf.push_back( objBbxesConf_[ObjCenterInd[i]]);
//            cout << viewIds_[ObjCenterInd[i]] << endl;
            viewIds.push_back( viewIds_[ObjCenterInd[i]]);//crash
        }

        // sort the objBbxes
//        cout << "sorting objBbxes" << endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rf_detector");
    rf_detector node;
    node.spin();

    return true;
}
