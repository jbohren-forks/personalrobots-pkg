#include <hand_detector/hand_detector.h>

int main(int argc, char** argv) {
  HandDetector hd;

  // -- Get env var options.
  if(getenv("DEBUG") != NULL) {
    hd.setDebug(true);
  }

  size_t num_samples = 1000;
  if(getenv("NSAMPLES") != NULL)
    num_samples = atoi(getenv("NSAMPLES"));
  
  int max_wcs = 2000;
  if(getenv("MAX_WCS") != NULL)
    max_wcs = atoi(getenv("MAX_WCS"));
  
  int max_secs = 0;
  if(getenv("MAX_SECS") != NULL)
    max_secs = atoi(getenv("MAX_SECS"));
  
  size_t num_candidates = 10;
  if(getenv("NCANDIDATES") != NULL)
    num_candidates = atoi(getenv("NCANDIDATES"));

  if(getenv("EXPERIMENT") != NULL)
    hd.experiment_ = getenv("EXPERIMENT");

  // -- Parse cmd line args.
  if(argc > 3 && !strcmp(argv[1], "--collectDatasetFromPolygons")) {
    cout << "Collecting a dataset for labels dir " << argv[2] << ", saving with name " << argv[3] << endl;  
    DorylusDataset* dd = hd.collectDatasetFromPolygons(argv[2], num_samples);
    if(!dd) {
      ROS_WARN("Dataset collection failed.");
    }
    else {
      cout << dd->status() << endl;
      dd->save(argv[3]);
      delete dd;
    }
  }
  if(argc > 4 && !strcmp(argv[1], "--collectDatasetFromDirs")) {
    cout << "Collecting a dataset for positive labels in " << argv[2] << " and negative labels in " << argv[3] << ", saving with name " << argv[3] << endl;  
    bool only_odd = false;
    DorylusDataset* dd = hd.collectDatasetFromDirs(argv[2], argv[3], num_samples, &only_odd);
    if(!dd) {
      ROS_WARN("Dataset collection failed.");
    }
    else {
      cout << dd->status() << endl;
      dd->save(argv[4]);
      delete dd;
    }
  }
  else if(argc > 3 && !strcmp(argv[1], "--train")) {
    cout << "Training a classifier on " << argv[2] << ", saving with name " << argv[3] << endl;  

    Dorylus* d = hd.train(argv[2], max_secs, max_wcs, num_candidates);
    if(!d)
      ROS_WARN("Classifier training failed.");
    else 
      d->save(argv[3]);    
  }
  else if(argc > 3 && !strcmp(argv[1], "--test")) {
    cout << "Visualizing results for classifier " << argv[3] << " on labeled data in " << argv[2] << endl;
    hd.test(argv[3], argv[2], num_samples);
  }

  else if(argc == 5 && !strcmp(argv[1], "--testOnDirs")) {
    cout << "Visualizing results for classifier " << argv[4] << " on labeled data in " << argv[2] << " and " << argv[3] << endl;
    hd.testOnDirs(argv[4], argv[2], argv[3], num_samples);
  }

  else if(argc > 2 && !strcmp(argv[1], "--showLabels")) {
    cout << "Visualizing labels in " << argv[2] << endl;
    hd.showLabels(argv[2]);
  }

  // -- Usage.
  else {
    cout << "usage: " << endl;

    cout << argv[0] << " --collectDatasetFromPolygons LABELS DATASET_SAVENAME" << endl;
    cout << " Environment variable options: " << endl;
    cout << "   DEBUG= sets the debugging flag." << endl;
    cout << "   NSAMPLES=x is the number of points per image to sample." << endl;
    cout << endl;

    cout << argv[0] << " --test LABELS CLASSIFIER" << endl;
    cout << " Environment variable options: " << endl;
    cout << "   NSAMPLES=x is the number of points per image to sample." << endl;
    cout << "   EXP=x is the experiment name.  Results will go into experiments/x/" << endl;
    cout << endl;

    cout << argv[0] << " --collectDatasetFromDirs POSITIVES_DIR NEGATIVES_DIR DATASET_SAVENAME" << endl;
    cout << " Environment variable options: " << endl;
    cout << "   DEBUG= sets the debugging flag." << endl;
    cout << "   NSAMPLES=x is the number of points per image to sample." << endl;
    cout << endl;

    cout << argv[0] << " --testOnDirs POSITIVES_DIR NEGATIVES_DIR CLASSIFIER" << endl;
    cout << " Environment variable options: " << endl;
    cout << "   NSAMPLES=x is the number of points per image to sample." << endl;
    cout << "   EXP=x is the experiment name.  Results will go into experiments/x/" << endl;
    cout << endl;

    cout << argv[0] << " --showLabels LABELS" << endl;
    cout << endl;


    cout << argv[0] << " --train DATASET CLASSIFIER_SAVENAME" << endl;
    cout << " Environment variable options: " << endl;
    cout << "   MAX_WCS=x is the max number of weak classifiers to add to the boosting classifier. Default 2000." << endl;
    cout << "   MAX_SECS=x is the max number of seconds to train for.  Default infinite (can C^c)." << endl;
    cout << "   NCANDIDATES=x is the max number of weak classifier candidates to use at each stage.  Default 10." << endl;
    cout << endl;

    cout << " where LABELS is a dir that contains polygons/ and images/ dirs. " << endl;
    //    cout << argv[0] << " --status DATASET" << endl;
    //    cout << argv[0] << " --status CLASSIFIER" << endl;

  }

}
