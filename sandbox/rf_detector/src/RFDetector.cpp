#include <ros/ros.h>
#include <std_msgs/String.h>
#include "topic_synchronizer2/topic_synchronizer.h"
#include <sstream>
#include <string>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "write.h"
#include "hog.h"

using namespace cv;
using namespace std;
int NView = 2;
float fgbgSampleRatio = 0.5;
int MAX_CharBuffer = 100;

class rf_detector
{

public:
    ros::NodeHandle n;

    Mat left;
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

    TopicSynchronizer sync_;
    sensor_msgs::ImageConstPtr limage;
    sensor_msgs::CvBridge lbridge;
    ros::Subscriber left_image_sub_;

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

    cout << "RFmodel_filename "<< RFmodel_filename << endl;
    cout << "VisualizeFlag  "<< VisualizeFlag << endl;
    cout << "padding_height "<< padding.height << endl;

    cout << "Finish handle input" << endl;

    namedWindow("left", 1);
    // subscribe to topics
    left_image_sub_ = n.subscribe("stereo/left/image_rect", 1, sync_.synchronize(&rf_detector::leftImageCallback, this));

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
            }
            imshow("left", left);
            cvWaitKey(100);
    }

    void syncCallback()
    {
            run_rf_detector();
            cout << "syncCallback" << endl;
    }

    void run_rf_detector()
    {
        float ObjHeight2sinStrideRatio = (float)winSize.height*ObjHeight2winHeightRatio/winStride.height;

        // setup hog parameters
        Size cellSize( cvFloor(winSize.width/numCellPerWin.width), cvFloor(winSize.height/numCellPerWin.height));
        Size blockSize(cvFloor(cellSize.width*numCellPerBlock.width), cvFloor(cellSize.height*numCellPerBlock.height));
        Size blockStride = cellSize;
        int nbins =  9;
        int derivAperture = 1;
        double winSigma=-1;
        int histogramNormType=0;
        double L2HysThreshold=0.2;
        bool gammaCorrection=true;
        int groupThreshold = 0;
        vector<double> AvergeKernel;
        AvergeKernel.push_back(0.25);
        AvergeKernel.push_back(0.5);
        int nCandidatePerDim = 5;

        // readin binary files
        vector< vector< float> > vecNotmVotes;
        vector< int> vecViewIds;
        readFloattext( NormVotesBinary_filename, vecNotmVotes);
        readIntegertext( ViewIdBinary_filename, vecViewIds);
        cout << vecNotmVotes.size() << " " << vecNotmVotes.at(0).size() << endl;
        cout << vecViewIds.size() << endl;

        // initialize the hog descriptor
        HogFast hog( winSize, blockSize, blockStride, cellSize,
                        nbins, derivAperture, winSigma,
                        histogramNormType, L2HysThreshold, gammaCorrection);
        cout << "Finish initialize hog"<< endl;

        // load the RF model
        hog.setRFDetector( RFmodel_filename, FgBgProb_filename);
        Vector< Rect > found;
        Vector< Point > foundIds;
        vector< float > foundConfs;
        vector< int> ScaleChangeBoundary;
        double t = (double)cv::getTickCount();
        // run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog.detectMultiScale(left, found, foundIds, foundConfs, ScaleChangeBoundary, hitThreshold, winStride, padding, Scale, groupThreshold, ObjHeight2winHeightRatio);
        t = (double)cv::getTickCount() - t;
        printf("feature plus RF time = %gms\n", t*1000./cv::getTickFrequency());

        // get data ready for voting
        vector< Vector<Rect> > ObjCenterWindowsAll;
        vector< vector<float> > ObjCenterConfAll;
        t = (double)cv::getTickCount();
        hog.cast_vots( left, found, foundIds, foundConfs, ScaleChangeBoundary,
            vecNotmVotes, vecViewIds, MaxViewId, AvergeKernel, nCandidatePerDim,
             winStride, ObjHeight2sinStrideRatio, Scale, MeanShiftMaxIter, ObjCenterWindowsAll,
             ObjCenterConfAll, VisualizeFlag);
        t = (double)cv::getTickCount() - t;
        printf("voting time = %gms\n", t*1000./cv::getTickFrequency());

        vector<int> ObjCenterInd;
        char buffer [50];
        Mat ImgClone;
        for (int viewId = 0; viewId < MaxViewId; viewId++){
            // sort the ObjCenter by Conf
            ObjCenterInd.clear();
            for ( int i = 0; i< ObjCenterConfAll[viewId].size(); i++)
                ObjCenterInd.push_back(i);
            vector<float>* tmp = &ObjCenterConfAll[viewId];
            sort( ObjCenterInd.begin(), ObjCenterInd.end(), index_cmp<vector<float>&>(*(tmp)) );
            for( int i = 0; i < 5;i++)//(int)ObjCenterWindowsAll[viewId].size(); i++ )//
            {
                sprintf(buffer,"C:%.5g,V:%d",ObjCenterConfAll[viewId][ObjCenterInd[i]],viewId);
                ImgClone = left.clone();
                Rect r = ObjCenterWindowsAll[viewId][ObjCenterInd[i]];
                rectangle(ImgClone, r.tl(), r.br(), Scalar(0,255,0), 1);
                putText( ImgClone, buffer, r.tl(),  CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255));
                namedWindow("detection", 1);
                imshow("detection", ImgClone);
                cv::waitKey(0);
            }

        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rf_detector");
    rf_detector node;
    node.spin();

    return true;
}
