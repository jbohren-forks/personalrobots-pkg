#ifndef WGTEST3DPOSEESTIMATE_H_
#define WGTEST3DPOSEESTIMATE_H_

#include "CvStereoCamModel.h"
#include "PointTracks.h"
#include "VisOdom.h"
using namespace cv::willow;

#include <vector>
#include <queue>
#include <opencv/cvwimage.h>
using namespace cv;
using namespace std;


// http://www.videredesign.com/vision/stereo_manuals.htm
//  Small Vision System Calibration Addendum - Version 4.x
// http://www.videredesign.com/docs/calibrate_4.4d.pdf
// This supplement to the User Manual for SVS contains more detailed information on the calibration procedure.

class CvTest3DPoseEstimate: public CvStereoCamModel
{
public:
    typedef CvStereoCamModel Parent;
    typedef enum {
    	Cartesian,
    	Disparity,
    	CartAndDisp,
    	Video,
    	Video2,
    	Video3,
    	/// bundle adjustment over a sequence of video images
    	VideoBundleAdj,
    	BundleAdj,
    	BundleAdjUTest
    } TestType;
    typedef enum {
      Indoor1,
      James4,
      SyntheticLoop1
    } DataSet;
    CvTest3DPoseEstimate(double Fx, double Fy, double Tx, double Clx = 0.0, double Crx = 0.0, double Cy = 0.0);
    CvTest3DPoseEstimate();
    virtual ~CvTest3DPoseEstimate();
    bool testPointClouds();
    bool testVideo();
    bool testVideo2();
    bool testVideo3();
    bool testVideo4();
    bool testVideoBundleAdj();
    bool testBundleAdj(
        string& points_file, string& frames_file,
        int num_free_frames, int num_fixed_frames, int num_iterations,
        int repeats,
        bool disturb_frames, bool disturb_points, bool disturb_obsvs);
    bool test();
    TestType mTestType;

    bool testVideo4OneFrame(queue<StereoFrame> inputImageQueue,
        FrameSeq& frameSeq, CamTracker& tracker);

    void setInputData(DataSet data_set);

    string input_data_path_;
    string output_data_path_;
    bool   verbose_;

    /// image size of input video sequences or images
    CvSize img_size_;
    /// input file sequence
    FileSeq* input_file_sequence_;

protected:
    void _init();
    void transform(CvMat *points0, CvMat *points1);
    void transform(CvMat *R, CvMat *points0, CvMat *T, CvMat *points1);
    void randomize(CvMat *xyzs, int num, double maxVal);
    double randReal(double min, double max);
    void disturb(const CvMat *xyzs, CvMat *xyzsNoised);
    static void MyMouseCallback(int event, int x, int y, int flagsm, void *param);
//    bool showDisparityMap(WImageBuffer1_16s & dispMap, string & winname, string & outputDirname, int frameIndex, int maxDisp);
//    bool drawKeypoints(WImage3_b & image, vector<Keypoint> & keyPointsLast, vector<Keypoint> & keyPointsCurr);
    void loadStereoImagePair(string & dirname, int & frameIndex, WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage);
    void disturbFrames(vector<FramePose*>& free_frames);
    void disturbPoints(PointTracks* tracks);
    void disturbObsvs(PointTracks* tracks);
    CvPoint3D64f mEulerAngle;
    CvPoint3D64f mTranslation;
    double mRotData[9];
    CvMat mRot;
    double mTransData[3];
    CvMat mTrans;
    CvRNG  mRng;
	double mDisturbScale;
	double mOutlierScale;
	double mOutlierPercentage;

	bool mStop;
};
#endif /*WGTEST3DPOSEESTIMATE_H_*/
