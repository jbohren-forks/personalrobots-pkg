#include <dorylus_node.h>

#define Matrix NEWMAT::Matrix

using namespace std;
using namespace features;

int g_mouse_x=0, g_mouse_y=0;
bool g_recent_mouse_move=false;

void on_mouse(int event, int x, int y, int flags, void* param) {
  //cout << "Event " << event << " at " << x << " " << y << " with flags " << flags  << endl;
  g_mouse_x = x;
  g_mouse_y = y;
  g_recent_mouse_move = true;
}


class DorylusNode : public ros::node
{
public:

  DorylusDataset dd_;
  vector<Descriptor*> descriptors_;
  //FixedSpinLarge spinL_;
  SpinImage *spinM_fixed_, *spinL_fixed_, *spinS_fixed_;
  SpinImage *spinM_fixed_HR_, *spinL_fixed_HR_, *spinS_fixed_HR_;
  SpinImage *spinM_nat_, *spinL_nat_, *spinS_nat_;
  Calonder *cal_land50_, *cal_land30_;
  vector<int> times;
  Dorylus d_;

  std_msgs::String cal_params_msg_;
  std_msgs::PointCloudFloat32 ptcld_msg_;
  SmartScan ss_;
  std_msgs::ImageArray images_msg_;
  IplImage *img_, *vis_;
  CvBridge<std_msgs::Image> *bridge_;
  bool built_bridge_;
  bool has_cal_params_, has_ptcld_, has_img_;
  Matrix transform_;

  vector<CvScalar> colors_;
  string ptcld_topic_;

  DorylusNode() : ros::node("dorylus_node")
  {
    // -- Setup descriptor functions.
    //descriptors_.push_back(&spinL_);

    ptcld_topic_ = string("spacetime_stereo");
    //ptcld_topic_ = string("videre/cloud_smallv");

    transform_ = Matrix(3,4); transform_ = 0.0;

    spinL_fixed_ = new SpinImage(string("FixedSpinLarge"), .20, 50, true);
    spinM_fixed_ = new SpinImage(string("FixedSpinMedium"), .10, 100, true);
    spinS_fixed_ = new SpinImage(string("FixedSpinSmall"), .05, 200, true);
    spinL_fixed_HR_ = new SpinImage(string("FixedSpinLargeHighRes"), .20, 100, true);
    spinM_fixed_HR_ = new SpinImage(string("FixedSpinMediumHighRes"), .10, 200, true);
    spinS_fixed_HR_ = new SpinImage(string("FixedSpinSmallHighRes"), .05, 400, true);
    cal_land50_ = new Calonder("Calonder_Land50", "/u/mihelich/Public/land50.trees");
    cal_land30_ = new Calonder("Calonder_Land30", "/u/mihelich/Public/land30.trees");
//     spinL_nat_ = new SpinImage(string("NatSpinLarge"), .20, 50, false);
//     spinM_nat_ = new SpinImage(string("NatSpinMedium"), .10, 100, false);
//     spinS_nat_ = new SpinImage(string("NatSpinSmall"), .05, 200, false);

    descriptors_.push_back(cal_land50_);  
    descriptors_.push_back(cal_land30_);  
    descriptors_.push_back(spinL_fixed_);  
    descriptors_.push_back(spinM_fixed_);  
    descriptors_.push_back(spinS_fixed_);  
//     descriptors_.push_back(spinL_fixed_HR_);  
//     descriptors_.push_back(spinM_fixed_HR_);  
//     descriptors_.push_back(spinS_fixed_HR_);  

    has_ptcld_ = has_img_ = has_cal_params_ = false;
    built_bridge_ = false;
    bridge_ = NULL;
    img_ = NULL;

    advertise<std_msgs::VisualizationMarker>("visualizationMarker", 100);

    colors_.push_back(cvScalar(255,0,0));
    colors_.push_back(cvScalar(0,255,0));
    colors_.push_back(cvScalar(0,0,255));
    colors_.push_back(cvScalar(255,255,0));
    colors_.push_back(cvScalar(255,0,255));
    colors_.push_back(cvScalar(0,255,255));
    colors_.push_back(cvScalar(127,255,0));
    colors_.push_back(cvScalar(127,0,255));
    colors_.push_back(cvScalar(0,127,255));
    colors_.push_back(cvScalar(255,127,0));
    colors_.push_back(cvScalar(255,0,127));
    colors_.push_back(cvScalar(0,255,127));
    colors_.push_back(cvScalar(255,255,255));

  }

  ~DorylusNode() {
//     if(bridge_)
//       delete bridge_;
  }

  void cbImageArray() {
    cout << " in imagearray cb" << endl;
    if(has_img_)
      return;

    if(!built_bridge_) {
      bridge_ = 
	new CvBridge<std_msgs::Image>
	(&images_msg_.images[1], 
	 CvBridge<std_msgs::Image>::CORRECT_BGR | 
	 CvBridge<std_msgs::Image>::MAXDEPTH_8U);
      built_bridge_ = true;
    }

    bridge_->to_cv(&img_); 
    vis_ = cvCloneImage(img_);
    has_img_ = true;
  }

  void cbPtcld() {
    cout << " in ptcld cb" << endl;
    if(has_ptcld_)
      return;

    ss_.setFromRosCloud(ptcld_msg_);
    has_ptcld_ = true;
  }

  void cbCalParams() {
    cout << " in cal cb" << endl;

    // -- Extract the transformation matrix.
    string cal = cal_params_msg_.data;
    string proj = cal.substr(cal.find("proj"), cal.find("rect"));
    NEWMAT::Real trnsele[12];
    sscanf(proj.c_str(), "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %*s", &trnsele[0], &trnsele[1], &trnsele[2], &trnsele[3], &trnsele[4], &trnsele[5], &trnsele[6], &trnsele[7], &trnsele[8], &trnsele[9], &trnsele[10], &trnsele[11]);
    transform_ << trnsele;
    cout << transform_;

    has_cal_params_ = true;
  }   
    
  void classifyPoint(int randId=-1) {
    if(!has_img_ || !has_ptcld_ || !has_cal_params_) {
      usleep(100000);
      cout << "Don't have all msgs yet..." << endl;
      return;
    }


    cout << "classifying point." << endl;

    // -- Get a random point from the pointcloud and project it into the image.
    if(randId==-1)
      randId = rand() % ss_.size();

    std_msgs::Point3DFloat32 pt = ss_.getPoint(randId);
    float x = pt.x;
    float y = pt.y;
    float z = pt.z;

    Matrix point(4,1);
    point(1,1) = pt.x;
    point(2,1) = pt.y;
    point(3,1) = pt.z;
    point(4,1) = 1;

    Matrix projected = transform_ * point;
    projected(1,1) = projected(1,1) / projected(3,1);
    projected(2,1) = projected(2,1) / projected(3,1);
    projected(3,1) = 1;
    int row = (int)projected(2,1);
    int col = (int)projected(1,1);

    // -- Get all the features for that point and construct the point object.
    clock_t start, end;
    Matrix* result;
    object obj;
    obj.label = -1; //To be classified.
    start = clock();
    for(unsigned int d=0; d<descriptors_.size(); d++) {
      (*descriptors_[d])(ss_, *img_, x, y, z, row, col, &result);
      obj.features[descriptors_[d]->name_] = *result;      
      delete result; result = NULL;

//       cout << descriptors_[d]->name_ << " descriptor." << endl;
//       cout << obj.features[descriptors_[d]->name_];
    }
    end = clock();
    cout << "Feature computation took " << (double(end)-double(start))/CLOCKS_PER_SEC << " seconds." << endl;
    

    // -- Classify 
    start = clock();
    Matrix response = d_.classify(obj);
    end = clock();
    cout << "Classification took " << (double(end)-double(start))/CLOCKS_PER_SEC << " seconds." << endl;

    int label_idx, trash;
    float val;
    val = response.Maximum2(label_idx, trash);
    int label = d_.classes_[label_idx-1];
    char label_str[10];
    
    cout << "Response: " << endl << response;
    cout << "Class " << label << endl;  
    cout << "Confidence " << val << endl;

    if(val == 0) 
      return;

    // -- Visualize
    CvFont font;
    double hScale=fabs(val)*.2 + .1;
    double vScale=fabs(val)*.2 + .1;
    int    lineWidth=(int)(fabs(val)*.2);
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);
    if(val < 0) {
      cvPutText(vis_,"-",cvPoint(col, row), &font, cvScalar(30,30,30));
    }
    else {
      sprintf(label_str,"%d",label);
      cvPutText(vis_,label_str,cvPoint(col, row), &font, colors_[label_idx-1]);
    }

    cvShowImage("Classification Visualization", vis_);
    //cvWaitKey(0);
  }
  
  
  void run(string classifier_file, bool mouse) {
    subscribe("videre/images", images_msg_, &DorylusNode::cbImageArray, 10); 
    subscribe(ptcld_topic_, ptcld_msg_, &DorylusNode::cbPtcld, 10); 
    subscribe("videre/cal_params", cal_params_msg_, &DorylusNode::cbCalParams, 10); 

    d_.load(classifier_file);

    cout << "Press spacebar to listen for new data, q to quit." << endl;
    cvNamedWindow("Classification Visualization", 0);

    SceneLabelerStereo sls(this);
    if(mouse) {
      int mode = 0;
      cvSetMouseCallback("Classification Visualization", on_mouse, &mode );
    }

    bool first = true;
    while(true) {
      char c = cvWaitKey(5);
      if(c == 'q')
	return;
      if(c == ' ' || first) {
	first = false;
	if(img_)
	  cvReleaseImage(&img_);
	img_ = NULL;
	has_img_ = has_ptcld_ = has_cal_params_ = false;
	subscribe("videre/images", images_msg_, &DorylusNode::cbImageArray, 1); 
	
	//Wait for a new videre/images.
	cout << "waiting for messages" << endl;
	while(!has_img_ || !has_ptcld_ || !has_cal_params_) 
	  usleep(100000);
	cout << "Done." << endl;
	cvShowImage("Classification Visualization", vis_);

	if(mouse) {
	  //Setup index of image points to 3d points.
	  sls.loadMsgsFromMem(images_msg_, ptcld_msg_, cal_params_msg_);
	  sls.projectAndCrossIndex();  //sls.xidx_ is available now.
	}

      }

      if(has_img_)
	unsubscribe(images_msg_);

      if(mouse && !g_recent_mouse_move) {
	continue;
      }
      g_recent_mouse_move = false;

      if(mouse) {
	int pt_id = sls.xidx_[g_mouse_y][g_mouse_x];
	if(pt_id != -1) //If there's a corresponding 3d point.
	  classifyPoint(pt_id);
      }
      else
	classifyPoint();
    }
  }

  void train(int nCandidates, int max_secs, int max_wcs) {
    time_t start, end;
    time(&start);
  
    cout << "Objective: " << d_.computeObjective() << endl;
    cout << "Objective (from classify()): " << d_.classify(dd_) << endl;
    int wcs=0;
    while(true) {
      bool found_better = d_.learnWC(nCandidates);
      if(!found_better) {
	continue;
      }
      wcs++;
      time(&end);
      cout << "Objective: " << d_.computeObjective() << endl;
//       cout << "Objective (from classify()): " << d_.classify(dd_) << endl;
//       cout << "Difference: " << d_.computeObjective() - d_.classify(dd_) << endl;

      // -- Display weak classifier.
//       for(unsigned int d=0; d<descriptors_.size(); d++) {
// 	weak_classifier wc = *d_.pwcs_.back();
// 	displayWeakClassifier(wc);
// 	if(descriptors_[d]->name_.compare(wc.descriptor) == 0) {
// 	  descriptors_[d]->display(wc.center);
// 	  break;
// 	}
//       }

      if(difftime(end,start) > max_secs)
	break;
      if(wcs >= max_wcs)
	break;
    }

    cout << "Done training." << endl;
  }


  object getObject(SceneLabelerStereo &sls, SmartScan &ss, IplImage& img, unsigned int nSamples, bool debug) {
    object obj;
    Matrix* result = NULL;

	
    
    // -- Initialize features map for this object.
    for(unsigned int iDesc=0; iDesc<descriptors_.size(); iDesc++) {
      obj.features[descriptors_[iDesc]->name_] = Matrix(descriptors_[iDesc]->result_size_, nSamples); 
      obj.features[descriptors_[iDesc]->name_] = 0.0;
    }
	 
    float x, y, z;
    int row, col;

    for(unsigned int iSample=0; iSample<nSamples; iSample++) {
      x = 0; y = 0; z = 0; row = 0; col = 0;
      //bool found3d = sls.getRandomPointFromImage(&x, &y, &z, &row, &col, obj.label);
      bool found3d = true;
      sls.getRandomPointFromPointcloud(&x, &y, &z, &row, &col, &ss);
      if(!found3d) {
	x = 1e20;
	y = 1e20;
	z = 1e20;
      }

      //cout << "displaying at " << row << " " << col << endl;

      IplImage *copy;
      std_msgs::VisualizationMarker mark;
      if(debug) {
	usleep(100000);
	publish(ptcld_topic_, sls.cloud_);
	usleep(100000);
	copy = cvCloneImage(sls.left_);
	cvNamedWindow("left", 1);
	cvCircle(copy, cvPoint(col, row), 5, cvScalar(0,255,0), 1);
	cvShowImage("left", copy);
	cvWaitKey(50);
	     
	if(found3d) {
	  mark.id = 1001;
	  mark.type = 1;
	  mark.action = 0;
	  mark.x = x;
	  mark.y = y;
	  mark.z = z;
	  mark.roll = 0;
	  mark.pitch = 0;
	  mark.yaw = 0;
	  mark.xScale = .02;
	  mark.yScale = .02;
	  mark.zScale = .02;
	  mark.alpha = 255;
	  mark.r = 0;
	  mark.g = 255;
	  mark.b = 0;
	  mark.text = string("");
	  publish("visualizationMarker", mark);
	}
	else {
	  cout << "No 3d point.  Press a key." << endl;
	  cvWaitKey();
	}
      }
	    
      // -- Compute the descriptors.
      time_t start,end; time(&start);
      for(unsigned int iDesc=0; iDesc<descriptors_.size(); iDesc++) {
	//cout << "Computing " << descriptors_[iDesc]->name_ << endl;

	//If debug, this will wait for input.
	(*descriptors_[iDesc])(sls.ss_cloud_, img, x, y, z, row, col, &result, debug);
	obj.features[descriptors_[iDesc]->name_].Column(iSample+1) = *result;
	delete result; result = NULL;

	//TODO: Mark down which descriptors are successful.	    
      }
      time(&end);
      times.push_back(difftime(end,start));
	  
      // -- Cleanup vis.
      if(debug) {
	mark.action = 2;
	publish("visualizationMarker", mark);
      }
    }

    return obj;
  }

  void buildDataset(unsigned int nSamples, vector<string> datafiles, string savename, bool debug=false, int nBG_pts=0, int nRepetitions_per_obj=1)
  {
    vector<object> objs;

    cout << "Building dataset." << endl;
    for(unsigned int iFile=0; iFile<datafiles.size(); iFile++) {
      SceneLabelerStereo sls(this);
      cout << "** Loading scene " << iFile << " out of " << datafiles.size() << " with name "  << datafiles[iFile] << endl;
      sls.loadMsgsFromFile(datafiles[iFile]);
      sls.processMsgs();
      
      object obj;
      // -- Collect data from background.
      for(int i=0; i<nBG_pts; i++) {
	SmartScan &ss = sls.ss_labels_[0]; //Ptcld of background.
	IplImage &img = *sls.left_;
	obj = getObject(sls, ss, img, 1, debug);
	obj.label = 0;
	objs.push_back(obj);
	cout << "Adding object of class " << obj.label << " with " << obj.features.size() << " descriptors." << endl;
      }


      // -- Collect data from objects.
      for(unsigned int iSS=0; iSS<sls.ss_objs_.size(); iSS++) {
	//cout << "Object " << iSS << ": class " << sls.ss_objs_[iSS].first << ", " << sls.ss_objs_[iSS].second->size() << " points." << endl;
	SmartScan &ss = *sls.ss_objs_[iSS].second;
	IplImage &img = *sls.left_;
	for(int iRep=0; iRep<nRepetitions_per_obj; iRep++) {
	  obj = getObject(sls, ss, img, nSamples, debug);
	  obj.label = sls.ss_objs_[iSS].first;
	  cout << "Adding object of class " << obj.label << " with " << obj.features.size() << " descriptors." << endl;
	  objs.push_back(obj);
	}
      }	

    }

    // -- Report average time to compute a descriptor.
    float ave=0.0;
    for(unsigned int i=0; i<times.size(); i++) {
      ave += times[i];
    }
    ave = ave/times.size();
    cout << "Average computation time for all descriptors at a point: " << ave << endl; 
    
    dd_.setObjs(objs);
    dd_.save(savename);
    cout << dd_.status() << endl;
    cout << dd_.displayYmc() << endl;
    //cout << endl << dd.status() << endl;
  }

private:
  //Dorylus dory;
  
};


// bool FixedSpinLarge::operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, bool* success, bool debug) {
//   Matrix result(5,1); result = 5.0;
//   return result;
// }



bool SpinImage::operator()(SmartScan &ss, IplImage &img, float x, float y, float z, int row, int col, Matrix** result, bool debug) {

  // -- Reset psi_.
  for(int i=0; i<width_; i++) {
    for(int j=0; j<height_; j++) {
      psi_->setElement(j,i, 0.0);
    }
  }
 

  Matrix* res = new Matrix(height_*width_,1); (*res) = 0.0;
  bool si_success;
  if(fixed_)
    si_success = ss.computeSpinImageFixedOrientation(*psi_, x, y, z, support_, pixelsPerMeter_, string("-y"));
  else
    si_success = ss.computeSpinImageNatural(*psi_, x, y, z, support_, pixelsPerMeter_);


  if(!si_success) {
    *result = res;
    return false;
  }

  for(int i=0; i<width_; i++) {
    for(int j=0; j<height_; j++) {
      (*res)(i*height_+j+1,1) = (float)psi_->getElement(j,i);
    }
  }

  // -- Normalize.
  Matrix div(1,1); div = 1/res->MaximumAbsoluteValue();
  *res = KP(*res, div);
  *result = res;

  if(debug) {
    display(*res);
  }
    
  return true;
}

bool Calonder::operator()(SmartScan &ss, IplImage &img, float x, float y, float z, int row, int col, Matrix** result, bool debug) {
  CvPoint pt = cvPoint(col, row); 
  cv::WImageView_b view = extractPatch3(&img, pt);
  IplImage *patch = view.Ipl();

  IplImage *gray = cvCreateImage(cvSize(patch->width, patch->height), IPL_DEPTH_8U, 1);
  cvCvtColor(patch,gray,CV_BGR2GRAY);
  DenseSignature sig = rt_.getDenseSignature(gray);

  Matrix* res = new Matrix(sig.size(),1); 
  for(unsigned int i=1; i<=sig.size(); i++) {
    (*res)(i,1) = sig(i-1);
  }

  // -- Normalize.  This may be a terrible idea.
  Matrix div(1,1); div = 1/res->MaximumAbsoluteValue();
  *res = KP(*res, div);

  *result = res;
  
  if(debug) {
    cvNamedWindow("patch", 0);
    cvShowImage("patch", gray);
    cvWaitKey(100);
    display(**result);
  }
  //cvReleaseImage(&patch);  cvwimage.
  cvReleaseImage(&gray);
  return true;
}

void Calonder::display(const Matrix& result) {
  cout << "Starting display of " << name_ << " feature." << endl;
  cout << result.t() << endl;

  cout << "Press Enter to continue. . .\n";
  cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

}

void SpinImage::display(const Matrix& result) {
  cout << "Starting display of " << name_ << " feature." << endl;

  //Display the spin image with gnuplot.
  char cmd[] = "echo \'set pm3d map; set size ratio -1; splot \"gnuplot_tmp\"\' | gnuplot -persist";
  char filename[] = "gnuplot_tmp";
  ofstream f(filename); 
  if(!f.is_open()) {
    cerr << "Could not open temporary file." << endl;
  }

  for(int h=height_-1; h>-1; h--) {
    for(int w=0; w<width_; w++) {
      //f << psi_->getElement(h, w) << endl;
      f << result(h+w*height_+1, 1) << endl;
    }
    f << endl;
  }
  f.close();
  system(cmd);
  remove(filename);

  cout << "Press Enter to continue. . .\n";
  cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  system("killall gnuplot");
}


int main(int argc, char **argv) {
  ros::init(argc, argv);
  DorylusNode dn;

  if(argc > 2 && !strcmp(argv[1], "--dataset")) {

    vector<string> datafiles;
    for(int i=3; i<argc; i++) {
      datafiles.push_back(string(argv[i]));
    }
    
    //d.buildDataset(2, datafiles, string("savename.dd"), true);
    string savename(argv[2]);
    if(savename.find(".dd") == savename.npos) {
      cerr << "Savename must have .dd extension.  Did you specify a savename?" << endl;
    }
    else {
      int nSamples = 1;
      int nBG_pts = 500;
      int nRepetitions_per_obj = 50;
      bool debug = false;

      if(getenv("TEST") != NULL) {
	cout << "TEST environment variable set.  Testing..." << endl;
	nSamples = 1;
	nBG_pts = 2;
	nRepetitions_per_obj = 2;
	debug = false;
      }
      dn.buildDataset(nSamples, datafiles, savename, debug, nBG_pts, nRepetitions_per_obj);
    }
//  DorylusDataset dd2;  dd2.load(string("savename.dd"));
//  dd2.save(string("savename2.dd"));
//  cout << d.dd_.status() << endl;
  }

  else if(argc == 2 && !strcmp(argv[1], "--testDatasetSave")) {
    dn.dd_.testSave();
  }

  else if(argc == 4 && !strcmp(argv[1], "--train")) {
    dn.dd_.load(string(argv[3]));
    //cout << dn.dd_.displayFeatures() << endl;
    cout << dn.dd_.status() << endl;
    dn.d_.loadDataset(&dn.dd_);

    if(getenv("TEST") != NULL) {
      cout << "TEST environment variable set.  Testing..." << endl;
      dn.train(50, 60*60*10, 1);
    }
    else {
      dn.train(50, 60*60*1, 1000);
      dn.d_.save(string(argv[2]));
    }

//     Dorylus d2;
//     d2.load(argv[2]);
//     d2.save(string(argv[2]) + "2");

  }

  else if(argc == 3 && !strcmp(argv[1], "--status")) {

    if(dn.dd_.load(string(argv[2]), true))
      cout << dn.dd_.status() << endl;

    if(dn.d_.load(string(argv[2]), true))
      cout << dn.d_.status() << endl;
  }

  else if(argc == 3 && !strcmp(argv[1], "--run")) {
    dn.run(string(argv[2]), false);
  }

  else if(argc == 3 && !strcmp(argv[1], "--mouse")) {
    dn.run(string(argv[2]), true);
  }
	
  else {
    cout << "Usage: " << endl;
    cout << "  dorylus_node --dataset [DORYLUS DATASET SAVENAME] [BAGFILES]" << endl;
    cout << "  dorylus_node --train [CLASSIFIER SAVENAME] [DORYLUS DATASET]" << endl;
    cout << "  dorylus_node --run [CLASSIFIER]" << endl;
    cout << "  dorylus_node --mouse [CLASSIFIER]" << endl;
    cout << "  dorylus_node --testDatasetSave" << endl;
  }

  usleep(250000);
  ros::fini();
  return 0;
}


