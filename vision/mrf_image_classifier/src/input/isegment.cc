#include "optimize/SubgradientOptimizer.hh"
#include "features/FeatureMatrix.hh"
#include "objective/L2Regularization.hh"
#include "features/FeatureGraphExtractor.h"
#include "objective/BinarySubmodularObjective.hh"
#include "input/LabelingViewer.hh"
#include "input/SegmentationLoader.hh"

#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>

#include "cv.h"
#include "highgui.h"

#include <map>
#include <vector>
#include <string>

using namespace std;

#define LEARN_ITER 10

#define SKIP_IMAGES 20

static DIR* ImagesDir;
//static char* ImagesDirName;
static string ImagesDirName;
static FeatureGraphExtractor* FGraphEx;
static UndirectedFeatureGraph* FGraph;

static vector<int> CurrLabeling;
//static vector<double> LabelWeights;
static map<int,int> ConstrainedNodes; // maps nodes to labels

static IplImage *CurrImage;
//static IplImage *CurrSegImage;
static string CurrImageName;

static GraphCutMinimizer *GCut;
//static BinarySubmodularObjective *Objective;
static Dvec0 *LearnedWeights;

static LabelingViewer *SegViewManual;
static LabelingViewer *SegViewLearned;
static LabelingViewer *SegViewConstrained;
//static int CurrentIm;

void processUserInput(int icin);

int getNextImage(const string &imdirname, 
		 DIR* imagesdir,
		 IplImage **image, 
		 IplImage **segmentation, 
		 string& imName);

void processMouse(int event, int xc, int yc, int flags, void* param);

int getXYBlob(int xc, int yc);

Dvec loadWeights(const string& fname);

int main(int argc, char *argv[]) {
  if (argc < 2) {
    cerr << "usage: isegment <images dir> <weights file?>" << endl;
    exit(1);
  }

  ImagesDirName = argv[1];
  ImagesDir = opendir(ImagesDirName.c_str());
  //  CurrentIm = 0;
  
  assert(ImagesDir != NULL);
  
  SegViewManual = NULL;
  SegViewLearned = NULL;
  SegViewConstrained = NULL;
  CurrImage = NULL;
  //  CurrSegImage = NULL;
  LearnedWeights = NULL;

  if (argc > 2) {
    ifstream file(argv[2]);
    LearnedWeights = Dvec0::deserialize(file);
    file.close();
    cout << "Loaded weights file " << argv[2] << endl;
  }


  cvNamedWindow("Manual segmentation",0);
  cvResizeWindow("Manual segmentation", 1000, 1000);
  //  cvNamedWindow("Learned labeling",0);

  cvSetMouseCallback("Manual segmentation", processMouse);

  processUserInput('n');

  while (1) {
    processUserInput(cvWaitKey());
  }
}

void skipFiles(DIR* dir, int num) {
  for (int ii = 0; ii < num; ii++) {
    struct dirent *dent = readdir(dir);

    // if end of directory, jump back to beginning
    if (dent == NULL) 
      rewinddir(dir);
  }
}

string getSegFilename(const string pathname) {
  int dotpos = pathname.find(".");

  return (pathname.substr(0, dotpos) + ".seg");
}

// Returns nonzero on error / end of directory
int getNextImage(const string &imdirname, 
		 DIR* imagesdir,
		 IplImage **image, 
		 string& imName) {
  struct dirent *dent = readdir(ImagesDir);

  // if end of directory, jump back to beginning
  // FIXME: check for infinite recursion...
  if (dent == NULL) {
    //    assert(errno == 0);
    rewinddir(imagesdir);
    getNextImage(imdirname, imagesdir, image, imName);
    return 1;
  }

  imName = imdirname + "/" + dent->d_name;

  cout << "F " << imName << endl;

  *image = cvLoadImage(imName.c_str(), CV_LOAD_IMAGE_COLOR);
  
  // skip all non-images
  if (*image == NULL) { 
    cout << "Skipping file: " << dent->d_name << endl;
    return getNextImage(imdirname, imagesdir, image, imName);
  }

  assert(*image != NULL);

  return 0;
}

void processMouse(int event, int xc, int yc, int flags, void* param) {
  static int lastBlob = -1;

  switch (event) {
  default: {
    int newlabel = -1;

    switch (flags) {
    case CV_EVENT_FLAG_LBUTTON:
      newlabel = 1;
      break;
    case CV_EVENT_FLAG_RBUTTON:
      newlabel = 0;
      break;
    default:
      break;
    }

    if (newlabel == -1) 
      break;

    int bnum = getXYBlob(xc, yc);

    if (bnum == lastBlob) { 
      //      cerr << "break..." << endl;
      break;
    }
    
    cerr << "Blob " << bnum << endl;

    lastBlob = bnum;

    ConstrainedNodes[bnum] = newlabel;

    if (CurrLabeling[bnum] != newlabel) {
      CurrLabeling[bnum] = newlabel;
      SegViewManual->viewLabeling(CurrLabeling);
    }

    // FIXME: efficiency
    vector<int> groundState;
    BinarySubmodularEnergy nrg(FGraph);
    nrg.groundState(*LearnedWeights, groundState, *GCut, ConstrainedNodes);
    SegViewConstrained->viewLabeling(groundState);

    break;
  }

  }
}

// constrain outer and center points
void setConstrainedNodes(const IplImage* image, 
			 map<int,int>& constrainedNodes) {
  int iw = image->width - 1, ih = image->height - 1;

  constrainedNodes[getXYBlob(0,0)] = 0;
  constrainedNodes[getXYBlob(iw,0)] = 0;
  constrainedNodes[getXYBlob(0,ih)] = 0;
  constrainedNodes[getXYBlob(iw,ih)] = 0;
  constrainedNodes[getXYBlob(iw/2,ih/2)] = 1;
}

// returns id of blob at xy position
int getXYBlob(int xc, int yc) {
  const Blobber *blobber = FGraphEx->getBlobber();
  const IplImage *blobim = blobber->getLabeledImage();
  return ((int*)(blobim->imageData + blobim->widthStep * yc))[xc];
}

// Returns 1 if segmentation was loaded, 0 otherwise
int processNewImage(const IplImage *currImage,
		    const string currImageName,
		    vector<int>& currLabeling) {

  int loadedSeg = 0;

  FGraphEx = new FeatureGraphExtractor(currImage);
  FGraph = FGraphEx->getFeatureGraph();

  FGraphEx->displayGraph();
  /*
  UndirectedFeatureGraph::
    display(currImage, FGraphEx->getContourImage(), 
	    *FGraphEx->getBlobber()->getBlobStats(),
	    *FGraph->getEdgeList());
	    */

  cvNamedWindow("Original image", 0);
  cvShowImage("Original image", currImage);

  SegViewManual = 
    new LabelingViewer("Manual segmentation",
		       *FGraphEx->getBlobber(), 
		       2);

  SegViewLearned = 
    new LabelingViewer("Learned segmentation",
		       *FGraphEx->getBlobber(), 
		       2);

  SegViewConstrained = 
    new LabelingViewer("Constrained segmentation",
		       *FGraphEx->getBlobber(), 
		       2);

  // load segmentation. 
  int error = SegmentationLoader::
    loadBlobSegmentation(*(FGraphEx->getBlobber()), 
			 getSegFilename(currImageName), 
			 currLabeling);

  // if no segmentation exists, create zero labeling
  if (error) {
    currLabeling.clear();
    for (int ii = 0; ii < FGraphEx->getBlobber()->numBlobs(); ii++)
      currLabeling.push_back(0);
    loadedSeg = 0;
    cout << "Copying learned segmentation " << endl;
  } else {
    loadedSeg = 1;
    cout << "Loaded segmentation file " << getSegFilename(currImageName) << endl;
  }

  SegViewManual->viewLabeling(currLabeling);

  // set up classifier
  GCut = new GraphCutMinimizer(FGraph->getEdgeList());
  BinarySubmodularEnergy energy(FGraph);
  /*
  FGraphEx->getBlobWeights(LabelWeights);
  Objective = 
    new BinarySubmodularObjective(energy, *GCut, currLabeling, LabelWeights);
  */

  if (LearnedWeights == NULL) {
    cout << "Creating new weight vector " << endl;
    LearnedWeights = new Dvec0(energy.getWeightDim());
  }

  vector<int> groundState;
  energy.groundState(*LearnedWeights, groundState, *GCut);
  SegViewLearned->viewLabeling(groundState);

  vector<int> groundState2;
  setConstrainedNodes(currImage, ConstrainedNodes);
  energy.groundState(*LearnedWeights, groundState2, *GCut, ConstrainedNodes);
  SegViewConstrained->viewLabeling(groundState2);

  return loadedSeg;
}

void deleteGlobalState() {
  if (CurrImage != NULL) {
    cvReleaseImage(&CurrImage);
    //    cvReleaseImage(&CurrSegImage);
  }

  ConstrainedNodes.clear();

  CurrImageName = "";

  CurrLabeling.clear();

  if (FGraphEx != NULL) 
    delete FGraphEx;
  if (FGraph != NULL)
    delete FGraph;
  if (SegViewManual != NULL)
    delete SegViewManual;
  if (SegViewConstrained != NULL)
    delete SegViewConstrained;
  if (SegViewLearned != NULL)
    delete SegViewLearned;
  if (GCut != NULL)
    delete GCut;
}

/*
// FIXME: error checking
Dvec loadWeights(const string& fname) {
  ifstream file(fname.c_str());

  vector<double> wvec;

  //  while( !file.eof() ) {
  while( file.good() ) {
    double val;
    if (! (file >> val) ) {
      break;
    }
    wvec.push_back(val);
  }

  file.close();

  Dvec0 weights(wvec.size());

  for (int ii = 0; ii < (int)wvec.size(); ii++) 
    weights(ii) = wvec[ii];

  return weights;
}
*/

void saveWeights(const string& fname, const Dvec& weights) {
  ofstream file(fname.c_str());
  file << weights;
  file.close();
}

void processUserInput(int icin) {
  char cin = (char)icin;

  switch (cin) {
  case 'n': { 			// load next image
    deleteGlobalState();
    getNextImage(ImagesDirName, ImagesDir,
		 &CurrImage, CurrImageName);

    int loadedSeg = 
      processNewImage(CurrImage, CurrImageName, CurrLabeling);

    if (!loadedSeg) 
      processUserInput('c');	// if did not load segmentation, copy learned seg.
  }
    break;
  case 'w': {
    ofstream file("intWeights.dat");
    LearnedWeights->serialize(file);
    file.close();
    cout << "Weights saved to intWeights.dat " << endl;
  }
    break;
  case 'p': 			// move back an image
    seekdir(ImagesDir, telldir(ImagesDir) - 2);
    processUserInput('n');
    break;
  case 's':			// save segmentation
    SegmentationLoader::
      writeBlobSegmentation(*(FGraphEx->getBlobber()), CurrLabeling, 
			    getSegFilename(CurrImageName));
    cout << "Saved segmentation to " << getSegFilename(CurrImageName) << endl;
    break;
  case 'l': {			// do learning iterations
    vector<double> labelWeights;
    FGraphEx->getBlobWeights(labelWeights);
    BinarySubmodularEnergy energy(FGraph);
    BinarySubmodularObjective objective(energy, *GCut, 
					CurrLabeling, labelWeights);

    L2Regularization regularization;

    SubgradientOptimizer sgo(&objective, 
			     &regularization, 
			     *LearnedWeights, 
			     1e4, 1.0, 1e2);

    //    cout << "TEST " << FGraph->getNodeFeat()->getCols() << endl;
    
    for (int it = 0; it < LEARN_ITER; it++) {
      sgo.iterate();
      sgo.objective();
    }

    sgo.currentSolution(*LearnedWeights);

    vector<int> groundState;
    energy.groundState(*LearnedWeights, groundState, *GCut);
    SegViewLearned->viewLabeling(groundState);

    vector<int> groundState2;
    setConstrainedNodes(CurrImage, ConstrainedNodes);
    energy.groundState(*LearnedWeights, groundState2, *GCut, ConstrainedNodes);
    SegViewConstrained->viewLabeling(groundState2);
  }
    break;
  case 'c': {			// copy constrained segmentation buffer
    vector<int> groundState;
    BinarySubmodularEnergy energy(FGraph);

    energy.groundState(*LearnedWeights, groundState, *GCut, ConstrainedNodes);
    
    CurrLabeling = groundState;

    SegViewManual->viewLabeling(CurrLabeling);    
  }
    break;
  case 'k': {		// skip some images
    skipFiles(ImagesDir, SKIP_IMAGES);
    processUserInput('n');
  }
    break;
  case 'q':			// quit
    deleteGlobalState();
    exit(0);
    break;
  }
}
