#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <string>
#include <fstream>
#include <tinyxml/tinyxml.h>
#include <descriptors_2d/descriptors_2d.h>
#include <descriptors_2d_gpl/descriptors_2d_gpl.h>
#include <dorylus/dorylus.h>

class HandDetector {
 public:
  string experiment_;
  
  HandDetector();
  //! Collect features on the even images from polygons.
  DorylusDataset* collectDatasetFromPolygons(std::string results_dir, size_t num_samples);
  //! Collect features on the even images from directories of positive and negative examples.
  //! @param only_odd If null, get all data.  If true, only get data from odd numbered images.
  DorylusDataset* collectDatasetFromDirs(std::string positives_dir, std::string negatives_dir, size_t num_samples, bool* only_odd = NULL);
  //! Train a boosting classifier. 
  Dorylus* train(std::string dataset_filename, int max_secs=0, int max_wcs=2000, int num_candidates=10);
  //! Get classification results on the odd images.
  void test(std::string classifier_filename, std::string results_dir, size_t num_samples);
  //! Get classification results on the odd images for aligned 128x128 data.
  void testOnDirs(std::string classifier_filename, std::string positives_dir, std::string negatives_dir, size_t num_samples);
  void showLabels(std::string results_dir);
  //! Sets the HandDetector debug_ flag as well as debugging flags for all the descriptors.
  void setDebug(bool debug);
  
 private:
  //! Boosting classifier.
  Dorylus d_;
  vector<ImageDescriptor*> descriptors_;
  bool debug_; 
  
  //! @param plabel If not null, forces the label of all features collected to be the value of *plabel.  polys can be empty in this case.
  void collectRandomFeatures(IplImage* img, int num_samples, const cv::Vector< cv::Vector<cv::Point> >& polys, 
			     cv::Vector<cv::KeyPoint>* keypoints, std::vector<object*>* objects, int* plabel = NULL);
};

std::vector<ImageDescriptor*> setupImageDescriptors();
void releaseImageDescriptors(std::vector<ImageDescriptor*>* desc);
void getPolysFromTinyXML(std::string filename, cv::Vector< cv::Vector<cv::Point> >* polys);
void createLabelMaps(map<std::string, int>* str2int, std::vector<std::string>* int2str);
void showLabelPolys(IplImage* img, const cv::Vector< cv::Vector<cv::Point> >& polys);
Eigen::MatrixXf* cvVector2Eigen(const cv::Vector<float>& v);
void getRandomKeypoints(IplImage* img, int num_samples, cv::Vector<cv::KeyPoint>* keypoints);
