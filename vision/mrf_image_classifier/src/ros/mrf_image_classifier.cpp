#include <cv.h>
#include <highgui.h>
#include <std_msgs/ImageArray.h>
#include <ros/node.h>

#include "classifier/NaryImageClassifier.hh"
#include "display/LabelingViewer.hh"
#include "image_utils/cv_bridge.h"

// resize images to this size
#define IMAGE_WIDTH 160
#define IMAGE_HEIGHT 120

class ROSImageClassifier : public ros::node
{
public:
  /**
     @param trainingFile Name of the file containing classifier state
   */
  ROSImageClassifier(const char* trainingFile, std::istream& objectsFile) :
    ros::node("mrf_image_classifier"),
    bridgeIn(NULL),
    currFrame(NULL),
    scaledImage(NULL),
    segmented(NULL),
    cTree(NULL),
    classifier(NULL),
    objectSet(objectsFile)
  {
    // initialize classifier 
    ifstream tFileStream(trainingFile);
    cTree = BinaryClassTreeUtils::deserialize(tFileStream);
    classifier = new NaryImageClassifier(cTree);

    scaledImage = 
      cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
  
    subscribe("videre/images", images, 
	      &ROSImageClassifier::processFrame, this, 1);
  }

  virtual ~ROSImageClassifier() {
    delete bridgeIn;
    delete cTree;
    delete classifier;

    if (currFrame != NULL)
      cvReleaseImage(&currFrame);

    if (segmented != NULL)
      cvReleaseImage(&segmented);

    if (scaledImage != NULL)
      cvReleaseImage(&scaledImage);
  }

  void processFrame() {
    if (bridgeIn == NULL)
      bridgeIn = new CvBridge<std_msgs::Image>
	// FIXME: should this be 0 or 1?
	(&images.images[0],
	 CvBridge<std_msgs::Image>::CORRECT_BGR | 
	 CvBridge<std_msgs::Image>::MAXDEPTH_8U
	 );

    if (currFrame != NULL)
      cvReleaseImage(&currFrame);

    bridgeIn->to_cv(&currFrame);

    cvResize(currFrame, scaledImage, CV_INTER_LINEAR);
      
    if (segmented == NULL)
      segmented = 
	cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_32S, 3);
      //      segmented = cvCreateImage(cvGetSize(currFrame), IPL_DEPTH_32S, 3);

    vector<int> labeling;
    vector<blobStat> blobStats;
    vector<std::pair<int,int> > edges;
    classifier->evaluate(scaledImage, segmented, labeling, blobStats, edges);

    LabelingViewer::viewLabeling("Segmented", 3, 
				 currFrame, segmented, 
				 objectSet,
				 labeling, blobStats, edges);

    cvNamedWindow("testing", 0);
    cvShowImage("testing", scaledImage);
    cvWaitKey(50);
  }

private:
  CvBridge<std_msgs::Image> *bridgeIn;
  std_msgs::ImageArray images;

  IplImage *currFrame;
  IplImage *scaledImage;
  IplImage *segmented;
  
  BinaryClassifierTree* cTree;
  NaryImageClassifier* classifier;
  ObjectSet objectSet;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv);

  assert(argc > 2);
  char* trainingFile = argv[1];
  char* objectsFile = argv[2];
  
  ifstream objStream(objectsFile);

  ROSImageClassifier classifier(trainingFile, objStream);
  
  objStream.close();

  while(1)
    sleep(1);
}
