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
#include <sstream>
#include <string>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "t2obj.h"
#include "pcd.h"
#include "Exehog.h"
#include "write.h"
#include "vote.h"
#include "shape2.h"

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
#include "visualization_msgs/Marker.h"

// transform library
#include <tf/transform_listener.h>

using namespace cv;
using namespace std;

int NView = 2;//dummy one for testing
float fgbgSampleRatio = 0.5;
int MAX_CharBuffer = 100;
static const double pi = 3.14159265358979323846;

struct DistPoint
{
    DistPoint(double _eps) : eps(_eps) {}
    inline bool operator()(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const
    {
        double dist = (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
        return dist<=eps*eps;
    }
    double eps;
};

void GetRectFromRegionLabels( Vector<geometry_msgs::Point>& objects_pp, Vector<int>& labels, int nclasses, vector<Rect>& RegionRect)	 {
        vector<int> count;
        count.resize(nclasses);
        for (unsigned int i =0; i< labels.size(); i++){
            count[ labels[i]]++;
        }

        vector<int> index;
        index.resize(nclasses);
        int tmp_count =1;
        for (int i =0; i< nclasses; i++){
            if (count[i] >= 10){
                Rect tmp;
                tmp.x = 100000;
                tmp.y = 100000;
                tmp.width = 0;
                tmp.height = 0;
                RegionRect.push_back( tmp);
                index[i] = tmp_count;
                tmp_count++;
            }
        }
        for (unsigned int i =0; i< labels.size(); i++){
            if (index[labels[i]] !=0){
                if (RegionRect[ index[labels[i]]-1].x > objects_pp[i].x)
                    RegionRect[ index[labels[i]]-1].x = objects_pp[i].x;
                if (RegionRect[ index[labels[i]]-1].y > objects_pp[i].y)
                    RegionRect[ index[labels[i]]-1].y = objects_pp[i].y;
                if (RegionRect[ index[labels[i]]-1].width < objects_pp[i].x)
                    RegionRect[ index[labels[i]]-1].width = objects_pp[i].x;
                if (RegionRect[ index[labels[i]]-1].height < objects_pp[i].y)
                    RegionRect[ index[labels[i]]-1].height = objects_pp[i].y;
            }
        }
        for (unsigned int i =0; i< RegionRect.size(); i++){
            RegionRect[ i].width = RegionRect[ i].width-RegionRect[ i].x;
            RegionRect[ i].height = RegionRect[ i].height-RegionRect[ i].y;
        }
};


float CalRegionOverlap(Rect& roi, Rect& region_roi){
        Point tl_inter;
        Point tl_union;
        if (roi.x > region_roi.x){
            tl_inter.x = roi.x;
            tl_union.x = region_roi.x;
        }else{
            tl_inter.x = region_roi.x;
            tl_union.x = roi.x;
        }
        if (roi.y > region_roi.y){
            tl_inter.y = roi.y;
            tl_union.y = region_roi.y;
        }else{
            tl_inter.y = region_roi.y;
            tl_union.y = roi.y;
        }

        Point br_inter;
        Point br_union;
        if (roi.x+roi.width > region_roi.x+region_roi.width){
            br_inter.x = region_roi.x+region_roi.width;
            br_union.x = roi.x+roi.width;
        }else{
            br_inter.x = roi.x+roi.width;
            br_union.x = region_roi.x+region_roi.width;
        }
        if (roi.y+roi.height > region_roi.y+region_roi.height){
            br_inter.y = region_roi.y+region_roi.height;
            br_union.y = roi.y+roi.height;
        }else{
            br_inter.y = roi.y+roi.height;
            br_union.y = region_roi.y+region_roi.height;
        }

        float inter_width = (br_inter.x-tl_inter.x);
        if (inter_width<0)
            return 0;

        float inter_height = (br_inter.y-tl_inter.y);
        if (inter_height<0)
            return 0;

        return inter_width*inter_height/(float)((br_union.y-tl_union.y)*(br_union.x-tl_union.x));///(float)(region_roi.width*region_roi.height);//
};

float MaxRegionOverlap( Rect& roi, Vector<Rect>& RegionRect, int& RegionRectId){
        float MaxOverlap=0;
        RegionRectId =0;
        for (unsigned int i=0; i< RegionRect.size(); i++){
            float tmp = CalRegionOverlap(roi, RegionRect[i]);
            if (tmp> 0.35 && MaxOverlap < tmp){
                MaxOverlap = tmp;
                RegionRectId = i;
            }
        }
        return MaxOverlap;
};

class rf_detector
{

public:
    ros::NodeHandle n;

    Mat image_clone;

    double min_depth;
    double max_depth;
    int indx;
    double eps_angle;
    geometry_msgs::Point32 plane_axis;
    cv::Size SampleStep;
    double Radius;
    int winStep;
    bool PrineTableFlag;

    // detector input
    string RFmodel_filename;
    string FgBgProb_filename;
    string ClassIdBinary_filename;
    string NormVotesBinary_filename;
    int MaxClassId;
    double hitThreshold;
    Size numCellPerWin;
    Size numCellPerBlock;
    Size padding;
    bool VisualizeFlag;
    double ObjHeight2winHeightRatio;
    int MeanShiftMaxIter;
    int ObjHeightStep;
    double ObjASStep;
    double RectOverlapRatio;
    double ConfThre;
    string ClassId2ViewObjClassMapFile;

    // detector tmp variables
    cv::Size winStride;
    float ObjHeight2sinStrideRatio;
    int groupThreshold;
    vector< vector< float> > vecNormVotes;
    vector< int> vecClassIds;
    int nCandidatePerDim;
    vector<int> unique_classIds;
    vector< CvMat* > VotesMap;
    int MaxObjASBin;
    int MinObjASBin;
    int MaxObjHeightBin;
    int MinObjHeightBin;
    vector< IplImage* > MatObjHeightBin;
    vector< IplImage* > MatObjASBin;
    vector<string> ObjClassStr;
    vector<int> ViewMap;

    sensor_msgs::PointCloud object_pc;
    sensor_msgs::PointCloud plane_pc;
    Vector<geometry_msgs::Point> object_pp;
    Vector<geometry_msgs::Point> plane_pp;
    vector<Rect> RegionRect;
    vector<cv::Size> winSize;
    Vector< geometry_msgs::Point32> interest_pts;
    vector< Vector< cv::Point> > image_patches;
    HogFast hog;
    shape shape_des;

    tf::TransformListener tf_;
    sensor_msgs::ImageConstPtr image_;
    sensor_msgs::CvBridge bridge_;
    sensor_msgs::CameraInfoConstPtr cinfo_;
    sensor_msgs::PointCloudConstPtr cloud_;

    ros::Subscriber image_sub_;
    ros::Subscriber caminfo_image_sub_;
    ros::Subscriber cloud_sub_;

    bool image_call_flag;
    bool caminfo_call_flag;
    bool FirstImgFlag;

    rf_detector()
    {
        // get parameters
        n.param( "~max_depth", max_depth, 1.);
	cout << "max_depth" << max_depth <<endl;
        n.param( "~min_depth", min_depth, 1.);
	cout << "min_depth" << min_depth <<endl;
        n.param( "~eps_angle", eps_angle, 1.5);
	cout <<"eps_angle " << eps_angle <<endl;
	double tmp;
        n.param( "~plane_axis_x", tmp, 0.);
	plane_axis.x = (float)(tmp);
	cout <<"plane_axis.x " << plane_axis.x <<endl;
        n.param( "~plane_axis_y", tmp, 1.);
	plane_axis.y = (float)(tmp);
	cout <<"plane_axis.y " << plane_axis.y <<endl;
        n.param( "~plane_axis_z", tmp, 0.);
	plane_axis.z = (float)(tmp);
	cout <<"plane_axis.z " << plane_axis.z <<endl;
        n.param( "~SampleStep", SampleStep.width, 0);
        cout <<"SampleStep" << SampleStep.width << endl;
        SampleStep.height = SampleStep.width;
        n.param( "~winStep", winStep, 1);
        cout <<"winStep" << winStep << endl;
        n.param( "~Radius", Radius, 1.);
        cout <<"Radius" << Radius << endl;
        n.param( "~PrineTableFlag", PrineTableFlag, false);
        cout <<"PrineTableFlag" << PrineTableFlag << endl;
	// detector input
        string tmp_String = "";
        n.param( "~RFmodel_filename", RFmodel_filename, tmp_String);
	cout << "RFmodel_filename" << RFmodel_filename <<endl;
        n.param( "~FgBgProb_filename", FgBgProb_filename, tmp_String);
	cout << "FgBgProb_filename" << FgBgProb_filename <<endl;
        n.param( "~ClassIdBinary_filename", ClassIdBinary_filename, tmp_String);
	cout << "ClassIdBinary_filename" << ClassIdBinary_filename <<endl;
        n.param( "~NormVotesBinary_filename", NormVotesBinary_filename, tmp_String);
	cout << "NormVotesBinary_filename" << NormVotesBinary_filename <<endl;
        n.param( "~MaxClassId", MaxClassId, 8);
	cout << "MaxClassId" << MaxClassId <<endl;
        n.param( "~hitThreshold", hitThreshold, 0.9);
	cout << "hitThreshold" << hitThreshold <<endl;
        n.param( "~numCellPerWin", numCellPerWin.width, 5);
	cout << "numCellPerWin" << numCellPerWin.width <<endl;
	numCellPerWin.height = numCellPerWin.width;
        n.param( "~numCellPerBlock", numCellPerBlock.width, 3);
	cout << "numCellPerBlock" << numCellPerBlock.width <<endl;
	numCellPerBlock.height = numCellPerBlock.width;
        n.param( "~padding", padding.width, 0);
	cout << "padding" << padding.width <<endl;
	padding.height = padding.width;
        n.param( "~VisualizeFlag", VisualizeFlag, true);
	cout << "VisualizeFlag" << VisualizeFlag <<endl;
        n.param( "~ObjHeight2winHeightRatio", ObjHeight2winHeightRatio, 3.);
	cout << "ObjHeight2winHeightRatio" << ObjHeight2winHeightRatio <<endl;
        n.param( "~MeanShiftMaxIter", MeanShiftMaxIter, 50);
	cout << "MeanShiftMaxIter" << MeanShiftMaxIter <<endl;
        n.param( "~ObjHeightStep", ObjHeightStep, 10);
	cout << "ObjHeightStep" << ObjHeightStep <<endl;
        n.param( "~ObjASStep", ObjASStep, 0.1);
	cout << "ObjASStep" << ObjASStep <<endl;
        n.param( "~RectOverlapRatio", RectOverlapRatio, 0.3);
	cout << "RectOverlapRatio" << RectOverlapRatio <<endl;
        n.param( "~ConfThre", ConfThre, 20.);
	cout << "ConfThre" << ConfThre <<endl;
        n.param( "~ClassId2ViewObjClassMapFile", ClassId2ViewObjClassMapFile, tmp_String);
	cout << "ClassId2ViewObjClassMapFile" << ClassId2ViewObjClassMapFile <<endl;
        cout << "Finish handle input" << endl;

        indx = 0;
        image_call_flag = false;
        caminfo_call_flag = false;
	FirstImgFlag = true;

        // subscribe to topics
        image_sub_ = n.subscribe("t_on_off/image", 1, &rf_detector::ImageCallback, this);
        caminfo_image_sub_ = n.subscribe("t_on_off/caminfo", 1, &rf_detector::CameraInfoCallback, this);
        cloud_sub_ = n.subscribe( "t_on_off/dense_cloud", 1, &rf_detector::cloudCallback, this );

	// Init detector	
	InitRFPcdDetector();
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

    void ImageCallback(const sensor_msgs::Image::ConstPtr& image)
    {
        image_call_flag = true; 
        image_ = image;
        if( bridge_.fromImage(*image_, "bgr8")) {
            cout << " image call back"<<endl;
            image_clone = Mat(bridge_.toIpl(), false);
	    if (FirstImgFlag){
		InitVoteSapce();
		FirstImgFlag = false;
	    }
        }
    }

    void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        caminfo_call_flag = true;
        cout << " camera info call back"<<endl;
        cinfo_ = info;
    }

    void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
    {
        cout << " point cloud call back"<<endl;
        cloud_ = point_cloud;
	cout << image_call_flag<< " " << caminfo_call_flag << " " << endl;
        if ( image_call_flag && caminfo_call_flag){
	    cout << " runObjectDetection" << "cloud_"<< cloud_->points.size() <<endl;
            runObjectDetection();
	    image_call_flag = false;
	    caminfo_call_flag = false;
        }
    }

    void runObjectDetection(){
        sensor_msgs::PointCloud positions;
        vector<geometry_msgs::Point> top;
        vector<float> scales;
  	winSize.clear();
	image_patches.clear();
	interest_pts.clear();
	RegionRect.clear();
 
        double t = (double)cv::getTickCount();
        // do table point pruning
        if (PrineTableFlag){
        	PruneTablePoint();
	}else{
		vector<int> ind;
		for (unsigned int q=0; q< cloud_->points.size(); q++){
			ind.push_back(q);
		}
		cloud_geometry::getPointCloud(*cloud_, ind, object_pc);
    		GetPointCloud2point( object_pc, object_pp, *cinfo_, tf_);
    		GetPointCloud2point( plane_pc, plane_pp, *cinfo_, tf_);
	}

	// take mean z
	float mean_z=0;
	for (unsigned int q=0; q< cloud_->points.size(); q++){
		mean_z += cloud_->points[q].z;	
	}
	mean_z /= cloud_->points.size();

        // Sample interest_point
        vector< cv::Point> image_patch_centers;
        Sample_interest_point( object_pc, plane_pc, object_pp, plane_pp, interest_pts, image_patch_centers,
                *cinfo_, tf_, SampleStep, image_clone.size());
	
	cout << "image_patch_centers size" << image_patch_centers.size() << endl;
	cout << "interest_pts size" << interest_pts.size() << endl;

	// Interest_point2Rois
	Interest_point2Rois( interest_pts, image_patch_centers, image_patches,
		winSize, *cinfo_, (float)(Radius), (unsigned int)(winStep));
        t = (double)cv::getTickCount() - t;
        printf("sample pts time = %gms\n", t*1000./cv::getTickFrequency());
 
        // debuging
	Mat ImgClone = image_clone.clone();
	cv::namedWindow("imgshow", 0);
        for (unsigned int q=0; q<image_patch_centers.size(); q++){
            cv::circle( ImgClone, image_patch_centers[q], 3, CV_RGB(0,255,0));
        }
	
        cv::imshow("imgshow", ImgClone);
        cv::waitKey(100);	
	
        // call detector
        if (image_patches.size()!=0){
	       	RunRFPcdDetector(); 
	}
    }

    void PruneTablePoint(){

	if (false){
        	sensor_msgs::PointCloud outside_pc;
		filterByZBounds( *cloud_, min_depth, max_depth , object_pc, outside_pc );
	}else{
        	vector<double> plane;
        	sensor_msgs::PointCloud outside_pc;
        	geometry_msgs::Point32 orientation;
        	filterTablePlane( *cloud_, plane, object_pc, plane_pc, outside_pc, orientation, eps_angle, min_depth, max_depth);
    		GetPointCloud2point( object_pc, object_pp, *cinfo_, tf_);
    		GetPointCloud2point( plane_pc, plane_pp, *cinfo_, tf_);
		// quick hack: segment the Object_pp
		Vector<int> labels;
		int nclasses = cv::partition(object_pp, labels, DistPoint(10));
		GetRectFromRegionLabels(object_pp, labels, nclasses, RegionRect);
	}
    }

    void InitRFPcdDetector(){

	// read ClassId2ViewObjClassMapFile
	readClassId2ViewObjClassMapFile(ClassId2ViewObjClassMapFile, ObjClassStr, ViewMap);

        // readin binary files
        readFloatBinPlain( NormVotesBinary_filename, 4, vecNormVotes); //norm_ vote=2 line, aspect_ratio=1 line, ObjH2WinHRatio= 1 line
        readIntegerBinPlain( ClassIdBinary_filename, vecClassIds);
	// figure out unique_viewId
	for (unsigned int i=0; i< vecClassIds.size(); i++){
	    	bool unique_flag = true;
		for ( unsigned int  j = 0; j<unique_classIds.size(); j++){
		    if ((vecClassIds[i]-1) == unique_classIds[j]){
			unique_flag = false;
			break;
		    }
		}
		if (unique_flag){
		    unique_classIds.push_back(vecClassIds[i]-1);
	//	    cout << "unique_classIds=" << vecClassIds[i] << endl;
		}
	}
        // deciding the number of bin of ObjHeight and of ObjAspect
    	MaxObjASBin = 0;
    	MinObjASBin = numeric_limits<int>::max();
    	int ObjASBin;
    	for (int Id = 0; Id < vecNormVotes.at(2).size(); Id++)// loop through all MormVotes
    	{
	    ObjASBin = (int)(vecNormVotes.at(2).at(Id)/ObjASStep);
	    if ( ObjASBin > MaxObjASBin){
		MaxObjASBin = ObjASBin;
	    }else if( ObjASBin < MinObjASBin){
		MinObjASBin = ObjASBin;
	    }
	}
	cout << "MinObjASBin="<<MinObjASBin<<" MaxObjASBin="<<MaxObjASBin<<endl;
	// heuristic
	MinObjHeightBin = SampleStep.width*5/ObjHeightStep;

        // load the RF model
        hog.setRFDetector( RFmodel_filename, FgBgProb_filename);

	// setup shape_des
	shape_des.NRadius = Radius;
    }

    void InitVoteSapce(){
	// initial voteMasp, object height and aspect ratio
	cv::Size ImgSize = image_clone.size();
	MaxObjHeightBin = ImgSize.height/ObjHeightStep;
    	cout << "MinObjHeightBin="<<MinObjHeightBin<<" MaxObjHeightBin="<<MaxObjHeightBin<<endl;
   	VotesMap.resize(MaxClassId);
	MatObjHeightBin.resize(MaxClassId);
	MatObjASBin.resize(MaxClassId);
    	for (int k=0; k<unique_classIds.size(); k++){
        	int valid_classId = unique_classIds[k];
        	VotesMap.at(valid_classId) = cvCreateMat( ImgSize.height, ImgSize.width, CV_32FC1);
		MatObjHeightBin[valid_classId] = cvCreateImage( cvSize(ImgSize.width, ImgSize.height), IPL_DEPTH_32F, MaxObjHeightBin-MinObjHeightBin+1);
		MatObjASBin[valid_classId] = cvCreateImage( cvSize(ImgSize.width, ImgSize.height), IPL_DEPTH_32F, MaxObjASBin-MinObjASBin+1);
    	}
    }

    void RunRFPcdDetector(){
        // setup hog parameters
        int nbins =  9;
        int derivAperture = 1;
        double winSigma=-1;
        int histogramNormType=0;
        double L2HysThreshold=0.2;
        bool gammaCorrection=true;
        groupThreshold = 0;
        nCandidatePerDim = 5;

        // initialize the hog descriptor
        hog.winSize = winSize[ winSize.size()/2];
        Size cellSize( cvFloor( hog.winSize.width/numCellPerWin.width), cvFloor( hog.winSize.height/numCellPerWin.height));
        Size blockSize(cvFloor(cellSize.width*numCellPerBlock.width), cvFloor(cellSize.height*numCellPerBlock.height));
        Size blockStride = cellSize;
        winStride = cellSize;
        cout << "winStride" << winStride.width <<" " << winStride.height << endl;
        ObjHeight2sinStrideRatio = (float) hog.winSize.height*ObjHeight2winHeightRatio/winStride.height;
	hog.blockSize = blockSize;
	hog.blockStride = blockStride;
        hog.cellSize = cellSize;
        hog.nbins = nbins;
        hog.derivAperture = hog.derivAperture;
        hog.winSigma = winSigma;
        hog.histogramNormType = histogramNormType;
        hog.L2HysThreshold = L2HysThreshold;
	hog.gammaCorrection = gammaCorrection;
        cout << "Finish initialize hog"<< endl;

	// calculate shape_des
	cv::Vector<cv::Vector<float> > shape_des_results( interest_pts.size());
        double t = (double)cv::getTickCount();
	shape_des.compute( *cloud_, interest_pts, shape_des_results);
        t = (double)cv::getTickCount() - t;
        printf("shape feature time = %gms\n", t*1000./cv::getTickFrequency());

        Vector< Rect > found;
        Vector< Point > foundIds;
        vector< float > foundConfs;
        vector< int> ScaleChangeBoundary;
        // run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        t = (double)cv::getTickCount();
        hog.detectSetOfScaleAppendFea(image_clone, found, foundIds, foundConfs, shape_des_results, ScaleChangeBoundary, hitThreshold, winStride, padding, image_patches, winSize, groupThreshold, (double)(ObjHeight2winHeightRatio));
        t = (double)cv::getTickCount() - t;
        printf("hog feature plus RF time = %gms\n", t*1000./cv::getTickFrequency());

        // get data ready for voting
        if (true){
        Vector<Rect> ObjCenterWindowsAll;
        vector<float> ObjCenterConfAll;
        vector<int> ObjCenterClassIdAll;
        t = (double)cv::getTickCount();
        cast_vots_pcd( image_clone, found, foundIds, foundConfs, ScaleChangeBoundary,
             vecNormVotes, vecClassIds, MaxClassId, nCandidatePerDim,
             winStride, ObjHeight2sinStrideRatio, winSize, MeanShiftMaxIter, ObjCenterWindowsAll,
             ObjCenterConfAll, ObjCenterClassIdAll, VisualizeFlag, hog.rf, 
	     unique_classIds, ObjHeightStep, ObjASStep, MaxObjHeightBin, MinObjHeightBin,
	     MaxObjASBin, MinObjASBin, ConfThre, 
	     VotesMap, MatObjHeightBin, MatObjASBin, RectOverlapRatio);
        t = (double)cv::getTickCount() - t;
        printf("voting time = %gms\n", t*1000./cv::getTickFrequency());

        vector<int> ObjCenterInd;
        vector<Rect> objBbxes_;
        vector<float> objBbxesConf_;
        vector<int> classIds_;

        Mat ImgClone;

	if (false){
		vector<float> ObjCenterPlanePPRatio;
		// use the plane_pp to calculate object_conf
		for (unsigned int q=0; q< ObjCenterWindowsAll.size(); q++){
			unsigned int area = ObjCenterWindowsAll[q].width*ObjCenterWindowsAll[q].height;
			unsigned int plane_pp_count = count_plane_pp( object_pp, ObjCenterWindowsAll[q]);
			ObjCenterPlanePPRatio.push_back( (float)(plane_pp_count)/(float)(area));
			cout <<"area"<< area <<" plane_count" << plane_pp_count << " ratio" << ((float)(plane_pp_count)/(float)(area)) <<endl;
		}	
		//
		
		// sort the ObjCenter by Conf
		ObjCenterInd.clear();
		for ( unsigned int i = 0; i< ObjCenterConfAll.size(); i++)
		    ObjCenterInd.push_back(i);
		
		vector<float>* tmp = &ObjCenterConfAll;
		sort( ObjCenterInd.begin(), ObjCenterInd.end(), index_cmp<vector<float>&>(*(tmp)) );
		for( int i = 0; i < ObjCenterConfAll.size();i++)
		{
			if ( ObjCenterPlanePPRatio[ObjCenterInd[i]] >0.2){
				objBbxes_.push_back( ObjCenterWindowsAll[ObjCenterInd[i]]);
				objBbxesConf_.push_back( ObjCenterConfAll[ObjCenterInd[i]]);
				classIds_.push_back( ObjCenterClassIdAll[ObjCenterInd[i]]);
			}
		}
	}else{ // do objectOverlap
		for (unsigned int q = 0; q < RegionRect.size(); q++){
			int ind;
                	float ratio = MaxRegionOverlap( RegionRect[q], ObjCenterWindowsAll, ind);
			if (ratio!=0){
				objBbxes_.push_back( ObjCenterWindowsAll[ind]);
				//objBbxesConf_.push_back( ratio);
				objBbxesConf_.push_back( ObjCenterConfAll[ind]);
				classIds_.push_back( ObjCenterClassIdAll[ind]);
			}
		}

	}

        char buffer [50];
    	if (VisualizeFlag){
		ImgClone = image_clone.clone();
	}

        for ( unsigned int i = 0; i< objBbxesConf_.size(); i++){
            if (VisualizeFlag){
                sprintf(buffer,"%.5g %s",objBbxesConf_[i], ObjClassStr[classIds_[i]+1].c_str());
                Rect r = objBbxes_[i];
                rectangle(ImgClone, r.tl(), r.br(), Scalar(0,255,0), 1);
                putText( ImgClone, buffer, r.tl(),  CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255));
		// draw arrows of the viewpoint direction
		cv::Point arrow_start, arrow_end;
		switch (ViewMap[classIds_[i]+1]){
		case 1:
			arrow_start.x = (int)( r.x+r.width/2);
			arrow_start.y = r.y+r.height;
			arrow_end.x = arrow_start.x;
			arrow_end.y = r.y;
			break;
		case 2:
			arrow_start = r.br();
			arrow_end = r.tl();
			break;
		case 3:
			arrow_start.y = (int)( r.y+r.height/2);
			arrow_start.x = r.x+r.width;
			arrow_end.y = arrow_start.y;
			arrow_end.x = r.x;
			break;
		case 4:
			arrow_start.x = r.x+r.width;
			arrow_start.y = r.y;
			arrow_end.x = r.x;
			arrow_end.y = r.y+r.height;
			break;
		case 5:
			arrow_end.x = (int)( r.x+r.width/2);
			arrow_end.y = r.y+r.height;
			arrow_start.x = arrow_end.x;
			arrow_start.y = r.y;
			break;
		case 6:
			arrow_start = r.tl();
			arrow_end = r.br();
			break;
		case 7:
			arrow_end.y = (int)( r.y+r.height/2);
			arrow_end.x = r.x+r.width;
			arrow_start.y = arrow_end.y;
			arrow_start.x = r.x;
			break;
		case 8:
			arrow_end.x = r.x+r.width;
			arrow_end.y = r.y;
			arrow_start.x = r.x;
			arrow_start.y = r.y+r.height;
			break;
		default:
			cv::waitKey(0);
			break;
		}
		DrawArrows( ImgClone, arrow_start, arrow_end, Scalar(255,0,0), 3);
            }
        }
        if (VisualizeFlag){
        	namedWindow("detection", 0);
                imshow("detection", ImgClone);
                cv::waitKey(10);
	}

	}
    }


    unsigned int count_plane_pp( Vector<geometry_msgs::Point> pp, cv::Rect r){
	unsigned int count=0;
	for (unsigned int q=0; q<pp.size(); q++ ){
		if (pp[q].x>r.x && pp[q].x<(r.x+r.width) && pp[q].y>r.y && pp[q].y<(r.y+r.height))
		count++;
	}
	return count;
    }

    void DrawArrows( cv::Mat& img, cv::Point arrow_start, cv::Point arrow_end, cv::Scalar color, int thickness){

	cv::line( img, arrow_start, arrow_end, color, thickness);
        cv::circle( img, arrow_end, 3*thickness, color);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rf_detector");
    rf_detector node;

    if (system(NULL)) puts ("Ok");
    else exit (1);

    node.spin();
    return true;
}
