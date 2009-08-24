#include <iostream>
#include <fstream>
#include <vector>
#include <cv.h>
#include "highgui.h"
#include "cxcore.h"
#include "cvaux.h"
#include "geometric_blur/gb.h"

using namespace std;
using namespace cv;

void writeFloat2Bin( string filename, float dsize,  float num_item, vector< vector< float> > & features)
{
    ofstream fout(filename.c_str(), ios::out | ios::binary);;

    fout.write((char *) &dsize, sizeof(float));
    fout.write((char *) &num_item, sizeof(float));
    for (int k = 0; k < features.size(); k++){
        for (int i = 0; i < features[k].size(); i++){
            fout.write((char *) &features[k][i], sizeof(float));
        }
    }
    fout.close();
}
void writeRect2text( string filename, vector<Rect> & Rect_)
{
    ofstream fout;
    fout.open(filename.c_str());
    if (!fout)
        return; // Failure

    int vector_size = Rect_.size();
    for (int k = 0; k < vector_size; k++){
        fout << Rect_[k].x << " " << Rect_[k].y << " "
            << Rect_[k].width << " " << Rect_[k].height << endl;
    }
    fout.close();
}

int main(int argc, char** argv)
{
    // parameters
    Size paddingTL_(0,0);
    Size paddingBR_(0,0);
    bool DenseSample_ = true;
    bool gammaCorrection_=true;
    int NumScale_=64;
    Size winSize_;
    Size winStride_;
    float alpha_ = 0.5;
    float beta_ = 1;
    int nbins_ = 4;
    cv::Mat img;

    // get input
    float ScaleRatio_= atof(argv[1]);;
    winSize_.width = atoi(argv[2]);
    winSize_.height = atoi(argv[3]);
    winStride_.width = atoi(argv[4]);
    winStride_.height = atoi(argv[5]);
    img = cv::imread(argv[6]);
    string FeatureFilename = argv[7];
    string ROIFilename = argv[8];

    gb gb_des( nbins_, gammaCorrection_, paddingTL_, paddingBR_,
        DenseSample_, ScaleRatio_, NumScale_, winSize_, winStride_,
        alpha_, beta_);

    vector<Mat> fbr;
    double t = (double)cv::getTickCount();
    gb_des.compute_orient_edges(img, fbr);
    t = (double)cv::getTickCount() - t;
    printf("oriented edge computing time = %gms\n", t*1000./cv::getTickFrequency());

    // calculate gb_des
    vector< vector<float> >  features;
    t = (double)cv::getTickCount();
    gb_des.compute_gb(fbr, features);
    t = (double)cv::getTickCount() - t;
    printf("gb descriptor computing time = %gms\n", t*1000./cv::getTickFrequency());
    cout << "feaDim " << features[0].size() << " nSamples " << features.size() <<endl;

    writeFloat2Bin( FeatureFilename, features[0].size(), features.size(), features);
    writeRect2text( ROIFilename, gb_des.ROI);
}
