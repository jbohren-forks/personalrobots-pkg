#include <hand_detector/hand_detector.h>

using namespace std;
using namespace cv;
USING_PART_OF_NAMESPACE_EIGEN;

HandDetector::HandDetector() :
  experiment_("default"),
  d_(Dorylus()),
  descriptors_(vector<ImageDescriptor*>()),
  debug_(false)
{
  descriptors_ = setupImageDescriptors();
}
  

void HandDetector::setDebug(bool debug) {
  debug_ = debug;
  for(size_t i=0; i<descriptors_.size(); ++i) {
    descriptors_[i]->debug_ = debug;
  }
}

Dorylus* HandDetector::train(string dataset_filename, int max_secs, int max_wcs, int num_candidates) {
  DorylusDataset dd;
  if(!dd.load(dataset_filename)) {
    ROS_FATAL("Could not load dataset %s", dataset_filename.c_str());
    return NULL;
  }
  
  Dorylus* d = new Dorylus();
  d->useDataset(&dd);
  d->train(num_candidates, max_secs, max_wcs);
  return d;
}


bool loadImgAndLabels(string filename, string results_dir, bool only_odd, IplImage** pimg, Vector< Vector<Point> >* polys) {

    // -- Get the next even-numbered xml file.
    if(filename.find(".xml") == string::npos) {
      return false;
    }

    assert(true == 1);
    assert(false == 0);
    int img_number = atoi(filename.substr(5, 10).c_str());
    if(img_number % 2 != only_odd) {
      return false;
    }

    // -- Get its polygons.
    getPolysFromTinyXML(results_dir + "/polygons/" + filename, polys);

    // -- Load the corresponding image.
    string img_filename = filename.substr(0, filename.find("_polygons.xml")) + ".jpg";
    img_filename = results_dir + "/images/" + img_filename;
    IplImage* img = cvLoadImage(img_filename.c_str());
    if(!img) {
      ROS_FATAL("Could not load image %s.", img_filename.c_str());
      return false;
    }

    *pimg = img;
    return true;
}

void HandDetector::collectRandomFeatures(IplImage* img, int num_samples, const Vector< Vector<Point> >& polys, 
					 Vector<KeyPoint>* keypoints, vector<object*>* objects, int* plabel) {
    getRandomKeypoints(img, num_samples, keypoints);

    // -- Get labels for each keypoint.
    vector<size_t> labels(keypoints->size());
    for(size_t j=0; j<keypoints->size(); ++j) {
      if(plabel)
	labels[j] = *plabel;
      else {
	int x = (*keypoints)[j].pt.x;
	int y = (*keypoints)[j].pt.y;
	
	labels[j] = 0;
	for(size_t k=0; k<polys.size(); ++k) {
	  if(pointPolygonTest(polys[k], Point2f(x, y), false) >= 0) {
	    labels[j] = 1;
	    break;
	  }
	}
      }

      // -- Show the labels.
//       if(debug_ && labels[j]==1) {
// 	cout << "This point is labeled " << labels[j] << endl;
// 	IplImage* vis = cvCloneImage(img);
// 	cvLine(vis, cvPoint(x-10, y), cvPoint(x+10, y), cvScalar(0,0,255));
// 	cvLine(vis, cvPoint(x, y-10), cvPoint(x, y+10), cvScalar(0,0,255));
// 	CVSHOW("Label", vis);
// 	cvWaitKey(0);
// 	cvReleaseImage(&vis);
//       }
    }

    // -- Collect features.
    Vector<vvf> results(descriptors_.size());
    for(size_t j=0; j<descriptors_.size(); ++j) {
      descriptors_[j]->compute(img, *keypoints, results[j]);
    }

    // -- Put into objects
    assert((int)keypoints->size() == num_samples);
    Vector<KeyPoint> actual_keypoints;
    actual_keypoints.reserve(keypoints->size());
    for(size_t k=0; k<(size_t)num_samples; k++)  {
      object* obj = new object;
      obj->label = labels[k];
      
      // -- Only accept those that have all valid descriptors. 
      bool success = true;
      for(size_t j=0; j<descriptors_.size(); j++) {
	if(results[j][k].empty()) {
	  success = false;
	  break;
	}
	obj->features[descriptors_[j]->getName()] = cvVector2Eigen(results[j][k]);
      }
      if(success) {
	objects->push_back(obj);
	actual_keypoints.push_back((*keypoints)[k]);
      }
      else
	delete obj;      
    }

    *keypoints = actual_keypoints; 
}



bool isHand(int x, int y, const Vector< Vector<Point> >& polys) {
  bool inside = false; 
  for(size_t l=0; l<polys.size(); ++l) {
    inside = cv::pointPolygonTest(polys[l], Point2f(x, y), false) >= 0;
    if(inside) {
      return true;
    }
  }
  return false;
}


void HandDetector::testOnDirs(std::string classifier_filename, std::string positives_dir, std::string negatives_dir, size_t num_samples) {
  bool tmp = true;
  bool* only_odd = &tmp;

  Dorylus d;
  d.load(classifier_filename);


  // -- Setup experiment stuff.
  string output_dir = "experiments/" + experiment_ + "/classification";
  mkdir("experiments", S_IRWXG | S_IRWXU | S_IRWXO);
  mkdir(("experiments/" + experiment_).c_str(), S_IRWXG | S_IRWXU | S_IRWXO);
  mkdir(output_dir.c_str(), S_IRWXG | S_IRWXU | S_IRWXO);


  map<int, float> average_response;
  average_response[0] = 0;
  average_response[1] = 0;
  map<int, int> num_objs;
  num_objs[0] = 0;
  num_objs[1] = 0;
  float tp=0, fp=0, tn=0, fn=0;

  for(size_t label=0; label<2; ++label) {
    vector<string> files;
    if(label == 1) 
      getDir(positives_dir, files);
    else
      getDir(negatives_dir, files);

    for(size_t i=0; i<files.size(); ++i) {

      if(files[i].find(".jpg") == string::npos) 
	continue;
      if(only_odd && *only_odd && atoi(files[i].substr(5,8).c_str()) % 2 == 0)
	continue;
      if(only_odd && !*only_odd && atoi(files[i].substr(5,8).c_str()) % 2 == 1)
	continue;

      
      string filename;
      if(label == 1) 
	filename = positives_dir + "/" + files[i];
      else
	filename = negatives_dir + "/" + files[i];
	
      IplImage* img = cvLoadImage(filename.c_str());  
      if(!img) 
	cout << "Could not load " << filename << endl;

      cout << "Working on file " << filename << ", " << i << " out of " << files.size() << endl;
      Vector<KeyPoint> keypoints; 
      Vector< Vector<Point> > polys;
      vector<object*> objects;
      collectRandomFeatures(img, num_samples, polys, &keypoints, &objects);

      // -- Run classification.
      assert(keypoints.size() == objects.size());
      IplImage* vis = cvCloneImage(img);
      for(size_t j=0; j<objects.size(); ++j) {

	// -- Make predictions and display.
	MatrixXf response = d.classify(*objects[j]);
	int size = ceil(log(ceil(abs(response(0,0))+.001)));
	int x = keypoints[j].pt.x;
	int y = keypoints[j].pt.y;
	if(response(0,0) > 0)
	  cvCircle(vis, cvPoint(x, y), size, cvScalar(0,255,0), -1);
	else
	  cvCircle(vis, cvPoint(x, y), size, cvScalar(0,0,255), -1);

	// -- Collect statistics about correctness.
	average_response[label] += response(0,0);
	num_objs[label]++;
	if(response(0,0) > 0 && label==1)
	  tp++;
	else if(response(0,0) > 0 && label==0)
	  fp++;
	else if(response(0,0) <= 0 && label==1)
	  fn++;
	else if(response(0,0) <= 0 && label==0)
	  tn++;
	else {
	  cout << "wrong" << endl;
	  return;
	}
      }

      // -- Show and save the output.
      CVSHOW("Classification", vis);
      cvWaitKey(100);
      string output_filename = output_dir + "/" + files[i].substr(0, files[i].find(".xml")) + ".jpg";
      cvSaveImage(output_filename.c_str(), vis);
      cvReleaseImage(&vis);

    }
  }

  // -- Save text results.
  average_response[0] /= num_objs[0];
  average_response[1] /= num_objs[1];
  float total = num_objs[0] + num_objs[1];
  ofstream file;
  file.open(("experiments/" + experiment_ + "/results.txt").c_str(), ios::out);
  file << "Results for " << classifier_filename << " on positive labels in " << positives_dir << " and negatives in " << negatives_dir << endl;
  file << "Total points tested:\t\t" << total << endl;
  file << "Average response for bg:\t" << average_response[0] << endl;
  file << "Average response for hand:\t" << average_response[1] << endl;
  file << "True positives:\t\t\t" << tp << endl;
  file << "True negatives:\t\t\t" << tn << endl;
  file << "False positives:\t\t" << fp << endl;
  file << "False negatives:\t\t" << fn << endl;
  file << "Precision (tp/(tp+fp)):\t\t" << tp / (tp + fp) << endl;
  file << "Recall (tp/(tp+fn)):\t\t" << tp / (tp + fn) << endl;
  file.close();


}

//! For running tests on data labeled with polygons.
void HandDetector::test(std::string classifier_filename, std::string results_dir, size_t num_samples) {
  Dorylus d;
  d.load(classifier_filename);

  // -- Setup experiment stuff.
  string output_dir = "experiments/" + experiment_ + "/classification";
  mkdir("experiments", S_IRWXG | S_IRWXU | S_IRWXO);
  mkdir(("experiments/" + experiment_).c_str(), S_IRWXG | S_IRWXU | S_IRWXO);
  mkdir(output_dir.c_str(), S_IRWXG | S_IRWXU | S_IRWXO);


  // -- Setup vars for collecting statistics of results.
  vector<string> int2str;
  createLabelMaps(NULL, &int2str);
  map<int, float> average_response;
  average_response[0] = 0;
  average_response[1] = 0;
  map<int, int> num_objs;
  num_objs[0] = 0;
  num_objs[1] = 0;
  float tp=0, fp=0, tn=0, fn=0;

  
  vector<string> files;
  getDir(results_dir + "/polygons", files);
  for(size_t i=0; i<files.size(); ++i) {

    // -- Get image and labels.
    bool only_odd = true;
    Vector< Vector<Point> > polys;
    IplImage* img = 0;
    bool success = loadImgAndLabels(files[i], results_dir, only_odd, &img, &polys);

    if(!success) {
      cout << "Rejected " << files[i] << endl;
      continue;
    }
    cout << "Working on file " << files[i] << ", " << i << " out of " << files.size() << endl;

    // -- Collect features from the image.
    Vector<KeyPoint> keypoints; 
    vector<object*> objects;
    collectRandomFeatures(img, num_samples, polys, &keypoints, &objects);


    assert(keypoints.size() == objects.size());
    IplImage* vis = cvCloneImage(img);
    for(size_t j=0; j<objects.size(); ++j) {

      // -- Make predictions and display.
      MatrixXf response = d.classify(*objects[j]);
      int size = ceil(log(ceil(abs(response(0,0))+.001)));
      int x = keypoints[j].pt.x;
      int y = keypoints[j].pt.y;
      if(response(0,0) > 0)
	cvCircle(vis, cvPoint(x, y), size, cvScalar(0,255,0), -1);
      else
	cvCircle(vis, cvPoint(x, y), size, cvScalar(0,0,255), -1);

      // -- Collect statistics about correctness.
      if(isHand(x, y, polys)) {
	average_response[1] += response(0,0);
	num_objs[1]++;
	if(response(0,0) > 0)
	  tp++;
	else
	  fn++;
      }
      else {
	average_response[0] += response(0,0);
	num_objs[0]++;
	if(response(0,0) > 0)
	  fp++;
	else
	  tn++;
      }
    }

    // -- Show and save the output.
    CVSHOW("Classification", vis);
    cvWaitKey(100);
    string output_filename = output_dir + "/" + files[i].substr(0, files[i].find(".xml")) + ".jpg";
    cvSaveImage(output_filename.c_str(), vis);
    cvReleaseImage(&vis);
  }

  // -- Save text results.
  average_response[0] /= num_objs[0];
  average_response[1] /= num_objs[1];
  float total = num_objs[0] + num_objs[1];
  ofstream file;
  file.open(("experiments/" + experiment_ + "/results.txt").c_str(), ios::out);
  file << "Results for " << classifier_filename << " on labels in " << results_dir << endl;
  file << "Total points tested:\t\t" << total << endl;
  file << "Average response for bg:\t" << average_response[0] << endl;
  file << "Average response for hand:\t" << average_response[1] << endl;
  file << "True positives:\t\t\t" << tp << endl;
  file << "True negatives:\t\t\t" << tn << endl;
  file << "False positives:\t\t" << fp << endl;
  file << "False negatives:\t\t" << fn << endl;
  file << "Precision (tp/(tp+fp)):\t\t" << tp / (tp + fp) << endl;
  file << "Recall (tp/(tp+fn)):\t\t" << tp / (tp + fn) << endl;
  file.close();
}

DorylusDataset* HandDetector::collectDatasetFromDirs(std::string positives_dir, std::string negatives_dir, size_t num_samples, bool* only_odd) {
  vector<object*> objects;
  
  vector<string> positives;
  getDir(positives_dir, positives);
  for(size_t i=0; i<positives.size(); ++i) {

    if(positives[i].find(".jpg") == string::npos) 
      continue;
    if(only_odd && *only_odd && atoi(positives[i].substr(5,8).c_str()) % 2 == 0)
      continue;
    if(only_odd && !*only_odd && atoi(positives[i].substr(5,8).c_str()) % 2 == 1)
      continue;


    string filename = positives_dir + "/" + positives[i];
    IplImage* img = cvLoadImage(filename.c_str());  
    cout << "Working on file " << filename << ", " << i << " out of " << positives.size() << endl;
    Vector<KeyPoint> keypoints; 
    int label = 1;
    Vector< Vector<Point> > polys;
    collectRandomFeatures(img, num_samples, polys, &keypoints, &objects, &label);
  }

  vector<string> negatives;
  getDir(negatives_dir, negatives);
  for(size_t i=0; i<negatives.size(); ++i) {

    if(negatives[i].find(".jpg") == string::npos) 
      continue;
    if(only_odd && *only_odd && atoi(negatives[i].substr(5,8).c_str()) % 2 == 0)
      continue;
    if(only_odd && !*only_odd && atoi(negatives[i].substr(5,8).c_str()) % 2 == 1)
      continue;

      
    string filename = negatives_dir + "/" + negatives[i];
    IplImage* img = cvLoadImage(filename.c_str());  
    cout << "Working on file " << filename << ", " << i << " out of " << negatives.size() << endl;
    Vector<KeyPoint> keypoints; 
    int label = 0;
    Vector< Vector<Point> > polys;
    collectRandomFeatures(img, num_samples, polys, &keypoints, &objects, &label);
  }
    
  // -- Put into DorylusDataset and save.
  DorylusDataset* dd = new DorylusDataset();
  dd->setObjs(objects);
  return dd;
}  


DorylusDataset* HandDetector::collectDatasetFromPolygons(string results_dir, size_t num_samples) {

  vector<string> files;
  getDir(results_dir + "/polygons", files);

  vector<object*> objects;
  for(size_t i=0; i<files.size(); ++i) {
    bool only_odd = false;
    Vector< Vector<Point> > polys;
    IplImage* img = 0;
    bool success = loadImgAndLabels(files[i], results_dir, only_odd, &img, &polys);

    if(!success) 
      continue;

    cout << "Working on file " << files[i] << ", " << i << " out of " << files.size() << endl;
    Vector<KeyPoint> keypoints; 
    collectRandomFeatures(img, num_samples, polys, &keypoints, &objects);
  }
  
  // -- Put into DorylusDataset and save.
  DorylusDataset* dd = new DorylusDataset();
  dd->setObjs(objects);
  return dd;
}

void HandDetector::showLabels(string results_dir) {
  vector<string> files;
  getDir(results_dir + "/polygons", files);
  for(size_t i=0; i<files.size(); ++i) {
    Vector< Vector<Point> > polys;
    IplImage* img = 0;
    bool only_odd = false;
    bool success = loadImgAndLabels(files[i], results_dir, only_odd, &img, &polys);

    if(!success) {
      success = loadImgAndLabels(files[i], results_dir, !only_odd, &img, &polys);
      if(!success) //not an xml.
	continue;
    }

    cout << "Found " << polys.size() << " label polys." << endl;
    showLabelPolys(img, polys);
  }
}

void getRandomKeypoints(IplImage* img, int num_samples, Vector<KeyPoint>* keypoints) { 
  assert(keypoints->empty());
  
  keypoints->reserve(num_samples);
  vector<int> labels(num_samples);
  for(int i=0; i<num_samples; i++)  {
    int x = rand() % img->width;
    int y = rand() % img->height;
    int size = 1;
    
    keypoints->push_back(KeyPoint(x, y, size));
  }
}

void showLabelPolys(IplImage* img, const Vector< Vector<Point> >& polys) {
  IplImage* label = cvCreateImage(cvGetSize(img), 8, 3);
  for(int x=0; x<img->width; ++x) {
    for(int y=0; y<img->height; ++y) {

      // -- Make all pixels gray.
      uchar gray = (CV_IMAGE_ELEM(img, uchar, y, x*3+0) + 
		    CV_IMAGE_ELEM(img, uchar, y, x*3+1) + 
		    CV_IMAGE_ELEM(img, uchar, y, x*3+2)) / 3;
      CV_IMAGE_ELEM(label, uchar, y, x*3+0) = gray;
      CV_IMAGE_ELEM(label, uchar, y, x*3+1) = gray;
      CV_IMAGE_ELEM(label, uchar, y, x*3+2) = gray;
    
      // -- Color the hands.
      for(size_t l=0; l<polys.size(); ++l) {
	bool inside; 
	inside = cv::pointPolygonTest(polys[l], Point2f(x, y), false) >= 0;
	if(inside) {
	  CV_IMAGE_ELEM(label, uchar, y, x*3) = CV_IMAGE_ELEM(img, uchar, y, x*3);
	  CV_IMAGE_ELEM(label, uchar, y, x*3+1) = CV_IMAGE_ELEM(img, uchar, y, x*3+1);
	  CV_IMAGE_ELEM(label, uchar, y, x*3+2) = CV_IMAGE_ELEM(img, uchar, y, x*3+2);
	}
      }
    }
  }
  CVSHOW("polygon", label);
  cvWaitKey(0);
  cvReleaseImage(&label);
}


void getPolysFromTinyXML(string filename, Vector< Vector<Point> >* polys) {
  assert(polys->empty());

  // -- Setup XML.
  TiXmlDocument XMLdoc(filename);
  bool loadOkay = XMLdoc.LoadFile();
  if (!loadOkay) {
    cout << "Could not load labels xml " << filename << endl;
    return;
  } 

  TiXmlElement *annotations, *annotation, *polygon, *results, *pt;
  annotations = XMLdoc.FirstChildElement("annotations");
  results = annotations->FirstChildElement("results");
  annotation = results->FirstChildElement("annotation");
  polygon = annotation->FirstChildElement("polygon");

  // -- For each poly, get the label and the poly.
  while(polygon) {
    pt = polygon->FirstChildElement("pt");
    Vector<Point> poly;
    while(pt) {
      poly.push_back(Point(atoi(pt->Attribute("x")), atoi(pt->Attribute("y"))));
      pt = pt->NextSiblingElement("pt");
    }
    polys->push_back(poly);
    polygon = polygon->NextSiblingElement("polygon");
  }
}

void createLabelMaps(map<string, int>* str2int, vector<string>* int2str) {
  if(str2int) {
    map<string, int>& s2i = *str2int;
    s2i["Unlabeled"] = 0;
    s2i["Hand"] = 1;
  }

  if(int2str) {
    vector<string>& i2s = *int2str;
    i2s.push_back("Unlabeled");
    i2s.push_back("Hand");
  }
}

// TODO: Make this not copy data.  
MatrixXf* cvVector2Eigen(const Vector<float>& v) {
  MatrixXf* m = new MatrixXf(v.size(), 1);
  for(size_t i=0; i<v.size(); i++) {
    (*m)(i,0) = v[i];
  }
  return m;
}


void releaseImageDescriptors(vector<ImageDescriptor*>* desc) {
  for(size_t i=0; i<desc->size(); i++) {
    delete (*desc)[i];
  }
  desc->clear();
}



vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> d;

// HogWrapper::HogWrapper(Size winSize, Size blockSize, Size blockStride, Size cellSize,
// 			int num_bins, int derivAperture, double winSigma,
// 			int histogramNormType, double L2HysThreshold, bool gammaCorrection) : 


  // -- Hog basic
//   d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 7, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 7, 1, -1, 0, 0.2, true));
 
  // -- Hog for hands.
  d.push_back(new HogWrapper(Size(100,100), Size(50,50), Size(25,25), Size(25,25), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(100,100), Size(50,50), Size(50,50), Size(25,25), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 9, 1, -1, 0, 0.2, true));


  // -- Hog extended
//   d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(16,16), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(8,8), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(4,4), 9, 1, -1, 0, 0.2, true));
//   d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(4,4), 9, 1, -1, 0, 0.2, true));
 
  SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10);
  SuperpixelColorHistogram* sch2 = new SuperpixelColorHistogram(5, 0.5, 10, NULL, sch1);
  SuperpixelColorHistogram* sch3 = new SuperpixelColorHistogram(5, 1, 10, NULL, sch1);
  SuperpixelColorHistogram* sch4 = new SuperpixelColorHistogram(5, .25, 10, NULL, sch1);
  d.push_back(sch1);
  d.push_back(sch2);
  d.push_back(sch3);
  d.push_back(sch4);
 
//   d.push_back(new SurfWrapper(true, 150));
//   d.push_back(new SurfWrapper(true, 100));
//   d.push_back(new SurfWrapper(true, 50));
//   d.push_back(new SurfWrapper(true, 25));
//   d.push_back(new SurfWrapper(true, 10));
   
//   Daisy* base_daisy = new Daisy(25, 3, 8, 8, NULL);
//   d.push_back(base_daisy);
//   d.push_back(new Daisy(50, 3, 8, 8, base_daisy));
//   d.push_back(new Daisy(75, 3, 8, 8, base_daisy));
//   d.push_back(new Daisy(100, 3, 8, 8, base_daisy));
//   d.push_back(new Daisy(150, 3, 8, 8, base_daisy));

  return d;
}
