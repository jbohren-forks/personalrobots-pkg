#include <dorylus_node.h>

#define Matrix NEWMAT::Matrix
using namespace std;

class DorylusNode : public ros::node
{
public:

  DorylusDataset dd_;
  vector<Descriptor*> descriptors_;
  //FixedSpinLarge spinL_;
  SpinImage *spinM_fixed_, *spinL_fixed_, *spinS_fixed_;
  SpinImage *spinM_fixed_HR_, *spinL_fixed_HR_, *spinS_fixed_HR_;
  SpinImage *spinM_nat_, *spinL_nat_, *spinS_nat_;
  vector<int> times;
  Dorylus d_;

  DorylusNode() : ros::node("dorylus_node")
  {
    // -- Setup descriptor functions.
    //descriptors_.push_back(&spinL_);

    spinL_fixed_ = new SpinImage(string("FixedSpinLarge"), .20, 50, true);
    spinM_fixed_ = new SpinImage(string("FixedSpinMedium"), .10, 100, true);
    spinS_fixed_ = new SpinImage(string("FixedSpinSmall"), .05, 200, true);
    spinL_fixed_HR_ = new SpinImage(string("FixedSpinLargeHighRes"), .20, 100, true);
    spinM_fixed_HR_ = new SpinImage(string("FixedSpinMediumHighRes"), .10, 200, true);
    spinS_fixed_HR_ = new SpinImage(string("FixedSpinSmallHighRes"), .05, 400, true);
    spinL_nat_ = new SpinImage(string("NatSpinLarge"), .20, 50, false);
    spinM_nat_ = new SpinImage(string("NatSpinMedium"), .10, 100, false);
    spinS_nat_ = new SpinImage(string("NatSpinSmall"), .05, 200, false);

    descriptors_.push_back(spinL_fixed_);  
    descriptors_.push_back(spinM_fixed_);  
    descriptors_.push_back(spinS_fixed_);  
    descriptors_.push_back(spinL_fixed_HR_);  
    descriptors_.push_back(spinM_fixed_HR_);  
    descriptors_.push_back(spinS_fixed_HR_);  
    //WAY too slow.
//     descriptors_.push_back(spinL_nat_);  
//     descriptors_.push_back(spinM_nat_);  
//     descriptors_.push_back(spinS_nat_);  

    advertise<std_msgs::VisualizationMarker>("visualizationMarker", 100);
  }

  void buildDataset(unsigned int nSamples, vector<string> datafiles, string savename, bool debug=false)
  {
    float x, y, z;
    int row, col;
    vector<object> objs;
    Matrix* result = NULL;

    cout << "Building dataset." << endl;
    for(unsigned int iFile=0; iFile<datafiles.size(); iFile++) {
      SceneLabelerStereo sls(this);
      cout << "** Loading scene " << iFile << " out of " << datafiles.size() << " with name "  << datafiles[iFile] << endl;
      sls.processMsgs(datafiles[iFile]);
      

      for(unsigned int iSS=0; iSS<sls.ss_objs_.size(); iSS++) {
	//cout << "Object " << iSS << ": class " << sls.ss_objs_[iSS].first << ", " << sls.ss_objs_[iSS].second->size() << " points." << endl;
	SmartScan &ss = *sls.ss_objs_[iSS].second;
	IplImage &img = *sls.left_;
	object obj;
	obj.label = sls.ss_objs_[iSS].first;


	// -- Initialize features map for this object.
	for(unsigned int iDesc=0; iDesc<descriptors_.size(); iDesc++) {
	  obj.features[descriptors_[iDesc]->name_] = Matrix(descriptors_[iDesc]->result_size_, nSamples); 
	  obj.features[descriptors_[iDesc]->name_] = 0.0;
	}
	 
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
	    publish("videre/cloud", sls.cloud_);
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
	
	objs.push_back(obj);
	//cout << "Adding object of class " << obj.label << " with " << obj.features.size() << " descriptors." << endl;
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
    //cout << endl << dd.status() << endl;
  }

private:
  //Dorylus dory;
  
};


// bool FixedSpinLarge::operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, bool* success, bool debug) {
//   Matrix result(5,1); result = 5.0;
//   return result;
// }



bool SpinImage::operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, Matrix** result, bool debug) {

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
    else
      dn.buildDataset(50, datafiles, savename, false);

//  DorylusDataset dd2;  dd2.load(string("savename.dd"));
//  dd2.save(string("savename2.dd"));
//  cout << d.dd_.status() << endl;
  }

  else if(!strcmp(argv[1], "--testDatasetSave")) {
    dn.dd_.testSave();
  }

  else if(!strcmp(argv[1], "--learnwc")) {
    dn.dd_.load(string(argv[2]));
    //cout << dn.dd_.displayFeatures() << endl;
    cout << dn.dd_.status() << endl;
    
    dn.d_.loadDataset(&dn.dd_);
    dn.d_.learnWC(10, 0);
  }

  else {
    cout << "Usage: " << endl;
    cout << "  dorylus_node --dataset savename [BAGFILES]" << endl;
  }

  usleep(250000);
  ros::fini();
  return 0;
}


