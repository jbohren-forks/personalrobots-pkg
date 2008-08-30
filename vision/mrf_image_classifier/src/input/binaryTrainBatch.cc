/**
   @mainpage 

   @section Executables

   @subsection binaryTrainBatch
 **/

#include "cv.h"

#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "classifier/BinarySubmodularImageClassifier.hh"
#include "util/Dvec.hh"
#include "input/LabelingViewer.hh"

void getImages(DIR* imageDir,
	       const string dirpath,
	       vector<IplImage*>& images,
	       vector<IplImage*>& segmentations);

int main(int argc, char *argv[]) {
  if (argc < 3) {
    cerr << "usage: <input dir> <output file> <input weights?>" << endl;
    exit(1);
  }

  srand48(time(NULL));

  DIR* dir = opendir(argv[1]);
  string dirname(argv[1]);
  string saveFile(argv[2]);
  const char* inWeights = argc > 3 ? argv[3] : NULL;
  //  string outputdir = string(argv[2]);
  
  assert(dir != NULL);

  vector<IplImage*> images;
  vector<IplImage*> segmentations;

  Dvec0 *wvec0;

  if (inWeights != NULL) {
    cout << "Loaded file " << inWeights << endl;
    ifstream file(inWeights);
    wvec0 = Dvec0::deserialize(file);
    file.close();
  } else {
    wvec0 = NULL;
  }

  BinarySubmodularImageClassifier imclass(wvec0, 1e6, 1e-5);

  getImages(dir, dirname, images, segmentations);





  /*
  // debugging
  for (vector<IplImage*>::iterator it = images.begin();
       it != images.end();
       it++) {
    FeatureGraphExtractor fgx(*it);
    fgx.displayGraph();
    cvWaitKey();
  }
  */



  cvNamedWindow("original",0);

  imclass.loadTrainingData(images, segmentations);

  for (int it = 0; it < 1000000; it++) {
    if (1) {
      for (int im = 0; im < (int)images.size(); im++) {
	cvShowImage("original", images[im]);

	IplImage* outImage = 
	  cvCreateImage(cvGetSize(images[im]), IPL_DEPTH_32S, 3);
	imclass.evaluate(images[im], outImage);
	LabelingViewer::viewLabeling("test", 2, images[im], outImage);
	cvWaitKey();
	cvReleaseImage(&outImage);
      }
    }

    imclass.train(1000);

    //    continue;
    ofstream file(saveFile.c_str());
    Dvec0 wvec = *imclass.getWeightVector();
    wvec.serialize(file);
    cout << "It. " << it << endl;
    file.close();

    //    getchar();
  }
}

// caller must free images
// FIXME: adapt to other types of images
void getImages(DIR* imageDir, 
	       const string dirpath,
	       vector<IplImage*>& images,
	       vector<IplImage*>& segmentations) {

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
      IplImage *seg = SegmentationLoader::loadSegmentation(segPath);;
      IplImage *image = cvLoadImage(imPath.c_str());

      images.push_back(image);
      segmentations.push_back(seg);

      cout << "L " << imPath << endl;
    }
  }

  getImages(imageDir, dirpath, images, segmentations);
}

