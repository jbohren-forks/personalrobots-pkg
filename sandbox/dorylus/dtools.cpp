#include <dorylus/dorylus.h>

using namespace std;

int main(int argc, char** argv) {
  Dorylus d;
  DorylusDataset dd;

  // -- Get env var options.  
  int max_wcs = 0;
  if(getenv("MAX_WCS") != NULL)
    max_wcs = atoi(getenv("MAX_WCS"));
  
  int max_secs = 0;
  if(getenv("MAX_SECS") != NULL)
    max_secs = atoi(getenv("MAX_SECS"));
  
  size_t num_candidates = 10;
  if(getenv("NCANDIDATES") != NULL)
    num_candidates = atoi(getenv("NCANDIDATES"));

  double min_util = 0;
  if(getenv("MIN_UTIL") != NULL) 
    min_util = atof(getenv("MIN_UTIL"));

  // -- Parse cmd line args.

  if(argc == 4 && !strcmp(argv[1], "--train")) {
    cout << "Training new classifier on " << argv[2] << ", saving with name " << argv[3] << " using " << num_candidates 
	 << " candidates, training until " << max_secs << " seconds, " << max_wcs << " wcs, or min utility " << min_util << endl;
    Dorylus d;
    DorylusDataset dd;
    if(!dd.load(argv[2])) 
      return 1;
    d.useDataset(&dd);
    d.train(num_candidates, max_secs, max_wcs, min_util);
    d.save(argv[3]);
  }

  else if(argc == 4 && !strcmp(argv[1], "--classify")) {
    cout << "Running classifier " << argv[3] <<  " on dataset "  << argv[2] << endl;
    Dorylus d;
    DorylusDataset dd;
    if(!dd.load(argv[2])) {
      cout << "Failed to load dataset " << argv[2] << endl;
      return 1;
    }
    if(!d.load(argv[3])) {
      cout << "Failed to load classifier " << argv[3] << endl;
      return 1;
    }
    cout << "Objective: " << d.classify(dd) << endl;
  }

  else if(argc == 5 && !strcmp(argv[1], "--relearnResponses")) {
    cout << "Relearning responses on classifier " << argv[3] << " using dataset " << argv[2] 
	 << ", saving new classifier with name " << argv[4] << endl;
    Dorylus d;
    DorylusDataset dd;
    if(!dd.load(argv[2])) {
      cout << "Failed to load dataset " << argv[2] << endl;
      return 1;
    }
    if(!d.load(argv[3])) {
      cout << "Failed to load classifier " << argv[3] << endl;
      return 1;
    }
    
    d.relearnResponses(dd);
    d.save(argv[4]);
  }


  else if(argc == 5 && !strcmp(argv[1], "--resumeTraining")) {
    cout << "Resuming learning of weak classifiers on " << argv[3] << " using dataset " << argv[2] 
	 << ", saving new classifier with name " << argv[4] << endl;
    Dorylus d;
    DorylusDataset dd;
    if(!dd.load(argv[2])) {
      cout << "Failed to load dataset " << argv[2] << endl;
      return 1;
    }
    if(!d.load(argv[3])) {
      cout << "Failed to load classifier " << argv[3] << endl;
      return 1;
    }
    
    d.useDataset(&dd);
    d.resumeTraining(num_candidates, max_secs, max_wcs, min_util);
    d.save(argv[4]);
  }

  else if(argc == 3 && !strcmp(argv[1], "--status")) {
    cout << "Examining " << argv[2] << endl;

    // -- Figure out what it is.
    ifstream file(argv[2]);
    if(!file.is_open()) {
      cout << "Could not open " << argv[2] << endl;
      return 1;
    }
    string line;
    getline(file, line);

    if(line.find("#DORYLUS CLASSIFIER LOG") != string::npos) {
      Dorylus d;
      if(!d.load(argv[2]))
	return 1;
      cout << d.status() << endl;
    }

    if(line.find("#DORYLUS DATASET LOG") != string::npos) {
      DorylusDataset dd;
      if(!dd.load(argv[2]))
	return 1;
      cout << dd.status() << endl;
    }
  }

  else if(argc == 6 && !strcmp(argv[1], "--join")) {
    cout << "Joining " << argv[2] << " and " << argv[3] << ", saving as " << argv[4] << endl;
    DorylusDataset dd1;
    DorylusDataset dd2;
    dd1.load(argv[2]);
    dd2.load(argv[3]);
    dd2.join(dd1);
    dd2.save(argv[4]);
  }

  // -- Usage.
  else {
    cout << "usage: " << endl;

    cout << argv[0] << " --status DATASET" << endl;
    cout << endl;
    cout << argv[0] << " --status CLASSIFIER" << endl;
    cout << endl;
    cout << argv[0] << " --train DATASET CLASSIFIER_SAVENAME" << endl;
    cout << " C^c to end training manually." << endl;
    cout << " Environment variable options: " << endl;
    cout << "   MAX_WCS=x is the max number of weak classifiers to add to the boosting classifier. Default infinite." << endl;
    cout << "   MAX_SECS=x is the max number of seconds to train for.  Default infinite." << endl;
    cout << "   MIN_UTIL=x is the minimum utility that an added weak classifier must have to continue training.  Default 0." << endl;
    cout << "   NCANDIDATES=x is the max number of weak classifier candidates to use at each stage. Default 10." << endl;
    cout << endl;

//     cout << argv[0] << " --relearnResponses DATASET OLD_CLASSIFIER NEW_CLASSIFIER_SAVENAME" << endl;
//     cout << endl;

    cout << argv[0] << " --resumeTraining DATASET OLD_CLASSIFIER NEW_CLASSIFIER_SAVENAME" << endl;
    cout << " C^c to end training manually." << endl;
    cout << " Environment variable options: " << endl;
    cout << "   MAX_WCS=x is the max number of weak classifiers to add to the boosting classifier. Default infinite." << endl;
    cout << "   MAX_SECS=x is the max number of seconds to train for.  Default infinite." << endl;
    cout << "   MIN_UTIL=x is the minimum utility that an added weak classifier must have to continue training.  Default 0." << endl;
    cout << "   NCANDIDATES=x is the max number of weak classifier candidates to use at each stage. Default 10." << endl;
    cout << endl;

    cout << argv[0] << " --classify DATASET CLASSIFIER" << endl;
    cout << " Runs CLASSIFIER on DATASET and prints the objective." << endl;
    cout << endl;

    cout << argv[0] << " --join DATASET1 DATASET2 DATASET_SAVENAME" << endl;
    cout << endl;
  }
}
