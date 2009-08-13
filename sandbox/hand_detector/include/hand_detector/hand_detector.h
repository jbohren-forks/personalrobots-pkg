#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <string>
#include <fstream>
#include <tinyxml/tinyxml.h>
#include <descriptors_2d/descriptors_2d.h>
#include <descriptors_2d_gpl/descriptors_2d_gpl.h>
#include <dorylus.h>

class HandDetector {
 public:
  HandDetector();
  HandDetector(std::string classifier_filename);
  //! Collect features on the even images.
  DorylusDataset* collectDataset(std::string results_dir, size_t num_samples);
  //! Train a boosting classifier. 
  Dorylus* train(std::string dataset_filename, int max_secs=0, int max_wcs=2000, int num_candidates=10);
  //! Get results on the odd images.
  void test(std::string results_dir);
  //! Visualize results on the odd images.
  void visualize(std::string classifier_filename, std::string results_dir, size_t num_samples);
  void showLabels(std::string results_dir);
  //! Sets the HandDetector debug_ flag as well as debugging flags for all the descriptors.
  void setDebug(bool debug);

 private:
  //! Boosting classifier.
  Dorylus d_;
  vector<ImageDescriptor*> descriptors_;
  bool debug_; 
 
  
  void collectRandomFeatures(IplImage* img, int num_samples, const cv::Vector< cv::Vector<cv::Point> >& polys, 
			     cv::Vector<cv::KeyPoint>* keypoints, std::vector<object*>* objects);
  
};

std::vector<ImageDescriptor*> setupImageDescriptors();
void releaseImageDescriptors(std::vector<ImageDescriptor*>* desc);
void getPolysFromTinyXML(std::string filename, cv::Vector< cv::Vector<cv::Point> >* polys);
void createLabelMaps(map<std::string, int>* str2int, std::vector<std::string>* int2str);
void showLabelPolys(IplImage* img, const cv::Vector< cv::Vector<cv::Point> >& polys);
Eigen::MatrixXf* cvVector2Eigen(const cv::Vector<float>& v);
void getRandomKeypoints(IplImage* img, int num_samples, cv::Vector<cv::KeyPoint>* keypoints);
