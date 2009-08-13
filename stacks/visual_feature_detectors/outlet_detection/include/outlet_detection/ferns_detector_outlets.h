
#if !defined(_FERNS_OUTLET_DETECTOR_H)
#define _FERNS_OUTLET_DETECTOR_H


#include "outlet_detection/outlet_model.h"
#include <cvaux.h>
using namespace cv;

static const int blurKSize = 3;
static const double sigma = 0;

void read_training_base(const char* config_path, char* outlet_filename, 
					 vector<feature_t>& train_features);

void ferns_l_detector_initialize(PlanarObjectDetector& detector, LDetector& ldetector, const char* outlet_config_path, Mat& object,Size patchSize);

int ferns_l_detect_outlets(Mat& _image, Mat& object, const char* outlet_config_path, PlanarObjectDetector& detector, LDetector& ldetector, vector<feature_t>& train_features,
                            vector<outlet_t>& holes, const char* output_path = 0, const char* output_filename = 0);

// Returns 1 if loading is successful, 0 otherwise
int ferns_detector_load(FernClassifier& detector, const char* outlet_config_path);

void ferns_detector_initialize(Vector<Point2f>& object_keypoints, FernClassifier& detector, const char* outlet_config_path, Mat& object, Size patchSize, PatchGenerator& gen);

int ferns_detect_outlets(Mat& _image, Mat& object, const char* outlet_config_path, FernClassifier& detector, Vector<Point2f>& object_keypoints, Vector<Point2f>& image_keypoints, vector<feature_t>& train_features,
                            vector<outlet_t>& holes, const char* output_path = 0, const char* output_filename = 0);



#endif