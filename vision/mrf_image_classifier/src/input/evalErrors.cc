#include "display/LabelingViewer.hh"
#include "classifier/NaryImageClassifier.hh"
#include "util/ThreadSafeLogger.hh"

#include "highgui.h"

#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <dirent.h>
#include <assert.h>

/// @fixme: make sure the classifier evaluator uses the same image size
#define IMAGE_WIDTH 160
#define IMAGE_HEIGHT 120
/*
#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 768
*/


/**
   
 **/
int getObjectClasses(const string fname, ObjectSet& objset);

void getImages(DIR* imageDir, 
	       const string dirpath,
	       vector<IplImage*>& images,
	       vector<IplImage*>& segmentations,
	       vector<string>& fileNames);

static IplImage* CurrSegmented = NULL;
static ObjectSet Classes;

void mouseCallback(int event, int xc, int yc, int flags, void* param);

int main(int argc, char *argv[]) {
  assert(getenv("oTrainingFile") != NULL);
  assert(getenv("oTestDir") != NULL);

  srand48(2);

  //  ObjectSet classes;
  string objfile(getenv("oTestDir"));
  objfile += "/objects.dat";
  int maxClass = getObjectClasses(objfile, Classes);

  // get training data
  DIR* imageDir = opendir(getenv("oTestDir"));
  string dirpath(getenv("oTestDir"));
  assert(imageDir != NULL);
  vector<IplImage*> images;
  vector<IplImage*> segmentations;
  vector<string> imageFileNames;
  getImages(imageDir, dirpath, images, segmentations, imageFileNames);

  // gui stuff
  cvNamedWindow("original",0);
  cvNamedWindow("segmented",0);
  cvNamedWindow("true segmentation",0);
  cvNamedWindow("segmentation errors",0);
  cvSetMouseCallback("segmented", mouseCallback);
  cvSetMouseCallback("true segmentation", mouseCallback);
  cvResizeWindow("segmented", 600, 600);
  cvResizeWindow("true segmentation", 600, 600);

  // initialize classifier tree
  ifstream ifile(getenv("oTrainingFile"));
  BinaryClassifierTree* tree = BinaryClassTreeUtils::deserialize(ifile);
  ifile.close();

  // initialize classifier, load training data
  NaryImageClassifier classifier(tree);
  //  classifier.loadTrainingData(images, segmentations);

  // gui stuff
  IplImage* segmented = 
    cvCreateImage(cvGetSize(images[0]), IPL_DEPTH_32S, 1);

  IplImage* segmentErrors = 
    cvCreateImage(cvGetSize(images[0]), IPL_DEPTH_8U, 1);

  for (int ii = 0; ii < (int)images.size(); ii++) {
    cvShowImage("original", images[ii]);
    
    // evaluate classifier, get blob info
    vector<int> labeling;
    vector<blobStat> blobStats;
    vector<std::pair<int,int> > edges;
    classifier.evaluate(images[ii], segmented, labeling, blobStats, edges);
    double pctCorr = 
      classifier.evaluateErrors(segmented, segmentations[ii], segmentErrors);
    
    CurrSegmented = segmented;
    
    cout << "% correct = " << pctCorr * 100 << endl;
    
    IplImage* origImage = 
      cvLoadImage(imageFileNames[ii].c_str(), CV_LOAD_IMAGE_COLOR);
    
    if (getenv("oDisplayOn")) {
      LabelingViewer::
	viewLabeling("segmented", maxClass + 1, 
		     origImage, segmented,
		     Classes, labeling, blobStats, edges);

      cvShowImage("segmentation errors", segmentErrors);
      
      // display blob graph
      FeatureGraphExtractor<tFeatureMatrix> fgraph(images[ii]);
      fgraph.displayGraph();
      
      // display true segmentation
      Blobber* blobber = fgraph.getBlobber();
      vector<int> trueBlobLabels;
      SegmentationLoader::loadBlobSegmentation(*blobber, 
					       segmentations[ii], 
					       trueBlobLabels);
      
      LabelingViewer::
	viewLabeling("true segmentation", maxClass + 1, 
		     origImage, segmentations[ii],
		     Classes, trueBlobLabels, blobStats, edges);
	
      
      
      //	cvWaitKey(10e3);
      cvWaitKey();
      cvReleaseImage(&origImage);
    }
  }
}

void mouseCallback(int event, int xc, int yc, int flags, void* param) {
  switch (event) {
  case CV_EVENT_LBUTTONDOWN: {
    int label = ((int*)(CurrSegmented->imageData + yc*CurrSegmented->widthStep))[xc];

    if (label == NULL_OBJECT_LABEL) {
      cerr << "(could not classify--insufficient training data)" << endl; 
      return;
    }

    //    cout << "L " << label << " x " << xc << " y " << yc << endl;
    string objName = Classes.findName(label);
    cout << objName << endl;
  }
    break;
  }
}

int getObjectClasses(const string fname, ObjectSet& objset) {
  ifstream objectsFile(fname.c_str());
  assert(!objectsFile.fail());
  ObjectSet temp(objectsFile);
  objset = temp;
  objectsFile.close();
  return objset.size();
}

// caller must free images
// FIXME: adapt to other types of images
void getImages(DIR* imageDir, 
	       const string dirpath,
	       vector<IplImage*>& images,
	       vector<IplImage*>& segmentations,
	       vector<string>& imageFileNames) {

  struct dirent *dent = readdir(imageDir);

  if (dent == NULL) 
    return;

  // skip non-png's
  string fname = dent->d_name;
  string::size_type dotpos = fname.find(".png");

  if (dotpos != string::npos) {
    string segPath = dirpath + "/" + fname.substr(0,dotpos) + ".seg";
    string imPath = dirpath + "/" + fname;

    // skip images w/out segmentations
    struct stat statBuf;
    if (stat(segPath.c_str(), &statBuf) == 0) { // if segmentation file exists
      IplImage *seg = SegmentationLoader::loadSegmentation(segPath);
      IplImage *image = cvLoadImage(imPath.c_str(), CV_LOAD_IMAGE_COLOR);

      IplImage *segScaled = 
	cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_32S, 1);
      IplImage *imageScaled = 
	cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);

      cvResize(seg, segScaled, CV_INTER_NN);
      cvResize(image, imageScaled, CV_INTER_LINEAR);

      cout << "I " << image->depth << endl;

      images.push_back(imageScaled);
      segmentations.push_back(segScaled);
      imageFileNames.push_back(imPath);

      cvReleaseImage(&seg);
      cvReleaseImage(&image);
      cout << "L " << imPath << endl;
    } else {
      perror("");
    }
  }

  getImages(imageDir, dirpath, images, segmentations, imageFileNames);
}

