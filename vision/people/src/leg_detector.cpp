/*********************************************************************
* Software License Agreement (BSD License)
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

#include "laser_processor.h"
#include "calc_leg_features.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "rosrecord/Player.h"

using namespace std;
using namespace laser_processor;
using namespace ros;

class LegDetector : public node
{
public:
  std_msgs::LaserScan scan_;
  std_msgs::PointCloud cloud_;

  ScanMask mask_;

  int mask_count_;

  vector< vector<float> > pos_data_;
  vector< vector<float> > neg_data_;
  vector< vector<float> > test_data_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  LegDetector() : node("laser_processor"), mask_count_(0), connected_thresh_(0.08), feat_count_(0)
  {
    save_[0] = 0;

    if (!strcmp(argv[1],"--train"))
    {
      bool loading_neg = false;
      bool loading_test = false;

      for (int i = 2; i < argc; i++)
      {
        if (!strcmp(argv[i],"--neg"))
        {
          loading_neg = true;
          loading_test = false;
        }
        else if (!strcmp(argv[i],"--test"))
        {
          loading_neg  = false;
          loading_test = true;
        }
        else if (!strcmp(argv[i],"--save"))
        {
          strncpy(save_,argv[++i],100);
        }
        else
        {
          ros::record::Player p;
          p.open(argv[i], ros::Time(0ull));
          
          if (loading_neg)
            p.addHandler<std_msgs::LaserScan>(string("*"), &LegDetector::loadNegData, this, NULL);
          else if (loading_test)
            p.addHandler<std_msgs::LaserScan>(string("*"), &LegDetector::loadTestData, this, NULL);
          else
            p.addHandler<std_msgs::LaserScan>(string("*"), &LegDetector::loadPosData, this, NULL);

          mask_.clear();
          mask_count_ = 0;

          while (p.nextMsg())
          {}
        }
      }
      printf("Done loading %d pos %d neg\n", pos_data_.size(), neg_data_.size());

      int sample_size = pos_data_.size() + neg_data_.size();

      int feat_count_ = pos_data_[0].size();
      printf("Using feature count of: %d\n", feat_count_);

      // opencv time
      CvMat* cv_data = cvCreateMat( sample_size, feat_count_, CV_32FC1);
      CvMat* cv_resp = cvCreateMat( sample_size, 1, CV_32S);

      int j = 0;
      for (vector< vector<float> >::iterator i = pos_data_.begin();
           i != pos_data_.end();
           i++)
      {
        float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
        for (int k = 0; k < feat_count_; k++)
          data_row[k] = (*i)[k];
        
        cv_resp->data.i[j] = 1;
        j++;
      }
      for (vector< vector<float> >::iterator i = neg_data_.begin();
           i != neg_data_.end();
           i++)
      {
        float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
        for (int k = 0; k < feat_count_; k++)
          data_row[k] = (*i)[k];
        
        cv_resp->data.i[j] = -1;
        j++;
      }

      CvMat* var_type = cvCreateMat( 1, feat_count_ + 1, CV_8U );
      cvSet( var_type, cvScalarAll(CV_VAR_ORDERED));
      cvSetReal1D( var_type, feat_count_, CV_VAR_CATEGORICAL );

      float priors[] = {1.0, 1.0};

      CvRTParams fparam(12,10,0,false,10,priors,true,5,100,0.001f,CV_TERMCRIT_ITER);
      fparam.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1);

      forest.train( cv_data, CV_ROW_SAMPLE, cv_resp, 0, 0, var_type, 0,
                    fparam);

      printf("Done training forest\n");

      cvReleaseMat(&cv_data);
      cvReleaseMat(&cv_resp);
      cvReleaseMat(&var_type);

      CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

      int pos_right = 0;
      int pos_total = 0;
      int neg_right = 0;
      int neg_total = 0;
      int test_right = 0;
      int test_total = 0;
      
      for (vector< vector<float> >::iterator i = pos_data_.begin();
           i != pos_data_.end();
           i++)
      {
        for (int k = 0; k < feat_count_; k++)
          tmp_mat->data.fl[k] = (float)((*i)[k]);
        if (forest.predict( tmp_mat) > 0)
          pos_right++;
        pos_total++;
      }

      for (vector< vector<float> >::iterator i = neg_data_.begin();
           i != neg_data_.end();
           i++)
      {
        for (int k = 0; k < feat_count_; k++)
          tmp_mat->data.fl[k] = (float)((*i)[k]);
        if (forest.predict( tmp_mat ) < 0)
          neg_right++;
        neg_total++;
      }

      for (vector< vector<float> >::iterator i = test_data_.begin();
           i != test_data_.end();
           i++)
      {
        for (int k = 0; k < feat_count_; k++)
          tmp_mat->data.fl[k] = (float)((*i)[k]);
        if (forest.predict( tmp_mat ) > 0)
          test_right++;
        test_total++;
      }

      printf("Forests: %d/%d %g %d/%d %g %d/%d %g\n", pos_right, pos_total, (float)(pos_right)/pos_total, neg_right, neg_total, (float)(neg_right)/neg_total, test_right, test_total, (float)(test_right)/test_total);

      if (strlen(save_) > 0)
      {
        forest.save(save_);
        printf("Trained classifier saved as: %s\n", save_);
      }

    } else if (argc > 1) {
      if (forest.load( argv[1] ))
      {
        feat_count_ = forest.get_active_var_mask()->cols;
        printf("Loaded forest has %d features\n", feat_count_);
      } else {
        printf("Could not load file\n");
        self_destruct();
      }
    } else {
      printf("Please provide a trained random forests classifier as an input.\n");
      self_destruct();
    }

    mask_.clear();

    subscribe("scan", scan_, &LegDetector::laserCallback, 1);
    advertise<std_msgs::PointCloud>("filt_cloud",1);
  }

  void loadPosData(string name, std_msgs::LaserScan* scan, ros::Time t, void* n)
  {
    if (mask_count_++ < 20)
    {
      mask_.addScan(*scan);
    }
    else
    {
      ScanProcessor processor(*scan,mask_);
      processor.splitConnected(connected_thresh_);
      processor.removeLessThan(5);
    
      for (list<SampleSet*>::iterator i = processor.getClusters().begin();
           i != processor.getClusters().end();
           i++)
        pos_data_.push_back( calcLegFeatures(*i, *scan));
    }
  }

  void loadNegData(string name, std_msgs::LaserScan* scan, ros::Time t, void* n)
  {
    ScanProcessor processor(*scan,mask_);
    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);
    
    for (list<SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
      neg_data_.push_back( calcLegFeatures(*i, *scan ));
  }

  void loadTestData(string name, std_msgs::LaserScan* scan, ros::Time t, void* n)
  {
    if (mask_count_++ < 20)
    {
      mask_.addScan(*scan);
    }
    else
    {
      ScanProcessor processor(*scan,mask_);
      processor.splitConnected(connected_thresh_);
      processor.removeLessThan(5);
    
      for (list<SampleSet*>::iterator i = processor.getClusters().begin();
           i != processor.getClusters().end();
           i++)
        test_data_.push_back( calcLegFeatures(*i, *scan ));
    }
  }

  void laserCallback()
  {
    cloud_.pts.clear();
    cloud_.chan.clear();
    cloud_.chan.resize(1);
    cloud_.chan[0].name = "rgb";

    ScanProcessor processor(scan_, mask_);

    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);

    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

    static int detect = 0;
    static int total = 0;

    //    printf("Real features\n");
    for (list<SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      vector<float> f = calcLegFeatures(*i, scan_);


      for (vector<float>::iterator j = f.begin();
           j != f.end();
           j++)
      {
        printf("%f ", *j);
      }

      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)(f[k]);


      bool is_leg = (forest.predict( tmp_mat ) > 0);
      
      if (is_leg)
      {
        (*i)->appendToCloud(cloud_, 255, 0, 0);
        detect++;
        printf(" * \n");
      }
      else
      {
        (*i)->appendToCloud(cloud_, 0, 255, 0);
        printf("\n");
      }
      
      total++;
    }

    cloud_.header = scan_.header;

    publish("filt_cloud", cloud_);

    cvReleaseMat(&tmp_mat);

    printf("Got scan with %f detected (%d/%d)\n", (float)(detect)/(float)(total), detect, total);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  LegDetector ld;
  ld.spin();
  ros::fini();
  return 0;
}
