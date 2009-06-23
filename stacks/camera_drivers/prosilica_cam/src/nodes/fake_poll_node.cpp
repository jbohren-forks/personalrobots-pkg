#include "ros/node.h"
#include "image_msgs/Image.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/FillImage.h"
#include "prosilica_cam/PolledImage.h"
#include "prosilica_cam/CamInfo.h"

#include <cv.h>
#include <cvwimage.h>
#include <highgui.h>

#include <cstring>
#include <cstdio>
#include <cassert>

// Extra stuff for snarfing directories
#include <dirent.h>
#include <sys/stat.h>
#ifndef S_ISDIR 
#define S_ISDIR(x) (((x) & S_IFMT) == S_IFDIR) 
#endif

class FakePublisher
{
private:
  std::vector<std::string> files_;
  std::vector<std::string>::const_iterator current_iter_, end_iter_;
  std::vector<std::string> intrinsics_;
  std::vector<std::string>::const_iterator current_intrinsic_;
  CvMat *K_, *D_;

  void loadIntrinsics(const std::string &intrinsics_file)
  {
    CvFileStorage* fs = cvOpenFileStorage(intrinsics_file.c_str(), 0, CV_STORAGE_READ);
    assert(fs);
    cvReleaseMat(&K_);
    cvReleaseMat(&D_);
    K_ = (CvMat*)cvReadByName(fs, 0, "camera_matrix");
    D_ = (CvMat*)cvReadByName(fs, 0, "distortion_coefficients");
    cvReleaseFileStorage(&fs);
  }

public:
  FakePublisher(const std::vector<std::string> &intrinsics_files,
                const std::vector<std::string> &image_files)
    : files_(image_files), current_iter_(files_.begin()), end_iter_(files_.end()),
      intrinsics_(intrinsics_files), current_intrinsic_(intrinsics_.begin()),
      K_(NULL), D_(NULL)
  {
    loadIntrinsics(intrinsics_[0]);
    
    ros::Node::instance()->advertiseService("/prosilica/poll", &FakePublisher::grab, this);
    ros::Node::instance()->advertiseService("/prosilica/cam_info_service",
                                            &FakePublisher::camInfo, this, 0);
  }

  ~FakePublisher()
  {
    cvReleaseMat(&K_);
    cvReleaseMat(&D_);
  }

  bool camInfo(prosilica_cam::CamInfo::Request &req,
               prosilica_cam::CamInfo::Response &rsp)
  {
    memcpy((char*)(&rsp.cam_info.D[0]), (char*)(D_->data.db), 5*sizeof(double));
    memcpy((char*)(&rsp.cam_info.K[0]), (char*)(K_->data.db), 9*sizeof(double));
    return true;
  }

  bool grab(prosilica_cam::PolledImage::Request &req,
            prosilica_cam::PolledImage::Response &res)
  {
    if (current_iter_ == end_iter_) {
      ros::Node::instance()->unadvertiseService("/prosilica/poll");
      return false;
    }

    // Load intrinsics from file
    loadIntrinsics(*current_intrinsic_);
    
    // Load image from file
    ROS_INFO("Loading %s", current_iter_->c_str());
    cv::WImageBuffer3_b img( cvLoadImage(current_iter_->c_str()) );
    if (img.IsNull())
      return false;

    // Copy into result
    fillImage(res.image, "image", img.Height(), img.Width(), 3,
              "bgr", "uint8", img.ImageData(), 3, img.WidthStep());
    
    // Copy cam info we care about
    memcpy(&res.cam_info.D[0], D_->data.db, 5*sizeof(double));
    memcpy(&res.cam_info.K[0], K_->data.db, 9*sizeof(double));

    current_iter_++;
    current_intrinsic_++;

    return true;
  }
};


// Function to filter for *.jpg
static int
filter_jpg(const struct dirent* d)
{
  std::string dstr(d->d_name);
  if((dstr.size() > 4) && 
     ((dstr.substr(dstr.size()-4,4) == ".jpg") ||
       (dstr.substr(dstr.size()-4,4) == ".JPG")))
    return 1;
  else
    return 0;
}

int main(int argc, char** argv)
{
  // Call init first, to get roscpp to consume the ROS-specific args
  ros::init(argc, argv);

  if (argc < 2) {
    printf("Usage: %s intrinsics.yml [FILES]\n", argv[0]);
    return 0;
  }
  
  std::vector<std::string> images, intrinsics;
  images.reserve(argc - 2);
  intrinsics.reserve(argc - 2);
  bool roi = strcmp(argv[1], "--roi") == 0;
  for (int i = 2; i < argc; ++i) {
    std::string name = argv[i];

    // Is it a directory?
    struct stat s;
    if((stat(name.c_str(), &s) == 0) && 
       (S_ISDIR(s.st_mode)))
    {
      // Yep; so pick out *.jpg from within.
      struct dirent **matches;
      int num_matches;
      if((num_matches = scandir(name.c_str(), &matches, filter_jpg, alphasort)) < 0)
        perror("scandir");
      else
      {
        for(int j=0;j<num_matches;j++)
        {
          std::string fname = name + "/" + matches[j]->d_name;
          images.push_back( fname.c_str() );
        }
      }
    }
    else
    {
          images.push_back( name );
    }
  }

  for (std::vector<std::string>::const_iterator it = images.begin();
       it != images.end();
       ++it)
  {
    printf("file:%s:\n", it->c_str());
    std::string name = *it;
    if (roi) {
      name.replace(name.begin() + name.find_last_of('.'), name.end(), ".yml");
      intrinsics.push_back(name);
    } else {
      intrinsics.push_back(argv[1]);
    }
  }
  
  ros::Node n("fake_publisher");
  FakePublisher fp(intrinsics, images);
  
  n.spin();

  return 0;
}
