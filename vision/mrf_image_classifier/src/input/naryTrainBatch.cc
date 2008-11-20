/**
   @file

   @b naryTrainBatch is a batch training program for 
   N-ary (i.e., multiclass) image segmentation.

   @verbatim
   $ naryTrainBatch [training_dir] [saved_classifier]
   @endverbatim

   @param training_dir A directory containing an objects.dat file,
   image files, and segmentations.

   @param saved_classifier A file containing a saved classifier state   
*/

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


/// Per-pixel standard deviation of Gaussian image perturbations
#define PERTURB_SIGMA 2.0

/// This number of small peturbations of the original image are produced
#define NUM_PETURBATIONS 4
//#define NUM_PETURBATIONS 1
#define EPOCHS HUGE_VAL
#define NUM_ITERATIONS 1000


/**
   
 **/
int getObjectClasses(const string fname, ObjectSet& objset);

void perturbImage(IplImage* image, IplImage* destImage, double sigma);

void getImages(DIR* imageDir, 
	       const string dirpath,
	       vector<IplImage*>& images,
	       vector<IplImage*>& segmentations,
	       vector<string>& fileNames);

static IplImage* CurrSegmented = NULL;
static ObjectSet Classes;

void mouseCallback(int event, int xc, int yc, int flags, void* param);

int main(int argc, char *argv[]) {
  if (argc < 2) {
    cerr << "usage: naryTrainBatch <training dir> <tree file?>" << endl;
    exit(1);
  }

  srand48(2);

  //  ObjectSet classes;
  string objfile(argv[1]);
  objfile += "/objects.dat";
  int maxClass = getObjectClasses(objfile, Classes);

  // get training data
  DIR* imageDir = opendir(argv[1]);
  string dirpath(argv[1]);
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

  // record initial time for saving weights
  char sstartTime[1000], fname[1000], buf[1000];
  time_t thetime = time(NULL);
  struct tm* startTime = localtime(&thetime);

  strftime(sstartTime, sizeof(sstartTime), "%d_%m_%y_%T", startTime);
  snprintf(fname, sizeof(fname), "mweights-%s.dat", sstartTime);
  snprintf(buf, sizeof(buf), "ln -sf %s mweights-last.dat", fname);
  system(buf);

  if (getenv("oOutputFile")) {
    strcpy(fname, getenv("oOutputFile"));
  }

  cout << "SAVE FILE " << fname << endl;

  // initialize classifier tree
  BinaryClassifierTree* tree = NULL;
  if (argc > 2) {
    ifstream ifile(argv[2]);
    tree = BinaryClassTreeUtils::deserialize(ifile);
    ifile.close();
  } else {
    tree = 
      NaryImageClassifier::makeRandomBCTree(Classes, images);
  }

  // set up logging
  ThreadSafeLogger *logger = NULL;
  ofstream *logStream = NULL;
  if (getenv("oLogOn")) {
    char namebuf[255];
    snprintf(namebuf, sizeof(namebuf), "/tmp/log-%s.dat", sstartTime);
    logStream = new std::ofstream(namebuf);
    assert(!logStream->fail());
    logger = new ThreadSafeLogger(logStream);
  }

  // initialize classifier, load training data
  NaryImageClassifier classifier(tree, logger);
  classifier.loadTrainingData(images, segmentations);

  // gui stuff
  IplImage* segmented = 
    cvCreateImage(cvGetSize(images[0]), IPL_DEPTH_32S, 1);
  //  IplImage* segmented8 = 
  //    cvCreateImage(cvGetSize(images[0]), IPL_DEPTH_8U, 

  IplImage* segmentErrors = 
    cvCreateImage(cvGetSize(images[0]), IPL_DEPTH_8U, 1);

  //debugging
  //  maxClass = 4;

  int epochs = (getenv("oEpochs") ? atof(getenv("oEpochs")) : EPOCHS);

  for (int epoch = 0; epoch < epochs; epoch++) {
    if (logger != NULL)
      logger->flush();

    if (getenv("oDisplayOn")) {
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
	
	/*
	cvConvert(segmented, segmented8);

	LabelingViewer::viewLabeling("segmented", maxClass + 1, 
				     images[ii], segmented);
	LabelingViewer::viewLabeling("true segmentation", maxClass + 1, 
				     images[ii], segmentations[ii]);
	*/	
	IplImage* origImage = 
	  cvLoadImage(imageFileNames[ii].c_str(), CV_LOAD_IMAGE_COLOR);

	LabelingViewer::
	  viewLabeling("segmented", maxClass + 1, 
		       origImage, segmented,
		       Classes, labeling, blobStats, edges);
	/*
	LabelingViewer::
	  imagesc("true segmentation 2", segmentations[ii]);
	*/

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

    classifier.train(NUM_ITERATIONS);
    
    // save classifier
    ofstream file;
    assert(!file.fail());
    file.open(fname);
    classifier.saveClassifierTree(file);
    //    file << "FOOBAR" << endl;
    file.close();
    cout << "SAVED TO FILE " << fname << endl;
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
  /*
  const int nClasses = 101;

  char buf[1000];
  for (int ii = 0; ii < nClasses; ii++) {
    snprintf(buf, sizeof(buf), "%d", ii);
    ObjectClass info(ii, buf);
    cinfos.push_back(info);
  }

  return nClasses;  
  */
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

      for (int ii = 0; ii < NUM_PETURBATIONS; ii++) {
	IplImage *imrand = 
	  cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
      
	perturbImage(imageScaled, imrand, PERTURB_SIGMA);

	images.push_back(imrand);
	segmentations.push_back(segScaled);
	imageFileNames.push_back(imPath);
      }

      cvReleaseImage(&seg);
      cvReleaseImage(&image);
      cvReleaseImage(&imageScaled);

      cout << "L " << imPath << endl;
    } else {
      perror("");
    }
  }

  getImages(imageDir, dirpath, images, segmentations, imageFileNames);
}

/// Add Gaussian random noise to an image
void perturbImage(IplImage* image, IplImage* destImage, double sigma) {
  static CvRNG rng;
  static bool initialized = false;
  //  const int seed = 0;

  if (!initialized) {
    rng = CvRNG(time(NULL));
    initialized = true;
  }

  IplImage* randim[3];
  for (int chan = 0; chan < 3; chan++) {
    randim[chan] = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    cvRandArr(&rng, randim[chan], CV_RAND_NORMAL, 
	      cvRealScalar(0), cvRealScalar(sigma));
  }

  IplImage *rand3chan = 
    cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
  cvMerge(randim[0], randim[1], randim[2], NULL, rand3chan);

  cvAdd(image, rand3chan, destImage);

  for (int chan = 0; chan < 3; chan++)
    cvReleaseImage(&randim[chan]);
  cvReleaseImage(&rand3chan);
}
