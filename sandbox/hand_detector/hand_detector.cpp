#include <hand_detector/hand_detector.h>

using namespace std;
using namespace cv;
USING_PART_OF_NAMESPACE_EIGEN;

HandDetector::HandDetector() :
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

void HandDetector::collectRandomFeatures(IplImage* img, int num_samples, const Vector< Vector<Point> >& polys, Vector<KeyPoint>* keypoints, vector<object*>* objects) {
    getRandomKeypoints(img, num_samples, keypoints);

    // -- Get labels for each keypoint.
    vector<size_t> labels(keypoints->size());
    for(size_t j=0; j<keypoints->size(); ++j) {
      int x = (*keypoints)[j].pt.x;
      int y = (*keypoints)[j].pt.y;
      
      labels[j] = 0;
      for(size_t k=0; k<polys.size(); ++k) {
	if(pointPolygonTest(polys[k], Point2f(x, y), false) >= 0) {
	  labels[j] = 1;
	  break;
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

void HandDetector::visualize(std::string classifier_filename, std::string results_dir, size_t num_samples) {
  Dorylus d;
  d.load(classifier_filename);

  vector<string> files;
  getDir(results_dir + "/polygons", files);
  for(size_t i=0; i<files.size(); ++i) {
    bool only_odd = true;
    Vector< Vector<Point> > polys;
    IplImage* img = 0;
    bool success = loadImgAndLabels(files[i], results_dir, only_odd, &img, &polys);

    if(!success) {
      cout << "Rejected " << files[i] << endl;
      continue;
    }

    cout << "Working on file " << files[i] << ", " << i << " out of " << files.size() << endl;

    Vector<KeyPoint> keypoints; 
    vector<object*> objects;
    collectRandomFeatures(img, num_samples, polys, &keypoints, &objects);

    // -- Make predictions and display.
    assert(keypoints.size() == objects.size());
    IplImage* vis = cvCloneImage(img);
    for(size_t j=0; j<objects.size(); ++j) {
      int x = keypoints[j].pt.x;
      int y = keypoints[j].pt.y;

      MatrixXf response = d.classify(*objects[j]);
      
      int size = ceil(log(ceil(abs(response(0,0)))));
      if(response(0,0) > 0)
	cvCircle(vis, cvPoint(x, y), size, cvScalar(0,255,0), -1);
      else
	cvCircle(vis, cvPoint(x, y), size, cvScalar(0,0,255), -1);
    }

    CVSHOW("Classification", vis);
    cvWaitKey(100);

    string output_dir = "classification";
    mkdir(output_dir.c_str(), S_IRWXG | S_IRWXU | S_IRWXO);
    string output_filename = output_dir + "/" + files[i].substr(0, files[i].find(".xml")) + ".jpg";
    cvSaveImage(output_filename.c_str(), vis);

    cvReleaseImage(&vis);
  }
}


DorylusDataset* HandDetector::collectDataset(string results_dir, size_t num_samples) {

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

void HandDetector::test(string results_dir) {}

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

  d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(16,16), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(8,8), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(4,4), 9, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(4,4), 9, 1, -1, 0, 0.2, true));
 
//   SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10);
//   SuperpixelColorHistogram* sch2 = new SuperpixelColorHistogram(5, 0.5, 10, NULL, sch1);
//   SuperpixelColorHistogram* sch3 = new SuperpixelColorHistogram(5, 1, 10, NULL, sch1);
//   SuperpixelColorHistogram* sch4 = new SuperpixelColorHistogram(5, .25, 10, NULL, sch1);
//   d.push_back(sch1);
//   d.push_back(sch2);
//   d.push_back(sch3);
//   d.push_back(sch4);
 
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
