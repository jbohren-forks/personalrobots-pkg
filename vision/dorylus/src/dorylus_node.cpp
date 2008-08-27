#include <dorylus_node.h>

#define Matrix NEWMAT::Matrix
using namespace std;

class DorylusNode : public ros::node
{
public:

  vector<Descriptor*> descriptors_;
  //FixedSpinLarge spinL_;
  FixedSpinMedium spinM_;

  DorylusNode() : ros::node("dorylus_node")
  {
    // -- Setup descriptor functions.
    //descriptors_.push_back(&spinL_);
    descriptors_.push_back(&spinM_);  
    SceneLabelerStereo sls(this);

    advertise<std_msgs::VisualizationMarker>("visualizationMarker", 100);
  }

  void buildDataset(unsigned int nSamples, vector<string> datafiles, string savename, bool debug=false)
  {
    float x, y, z;
    int row, col;
    vector<object> objs;
    Matrix* result;

    cout << "Building dataset." << endl;
    for(unsigned int iFile=0; iFile<datafiles.size(); iFile++) {
      SceneLabelerStereo sls(this);
      //cout << "Loading " << datafiles[iFile] << endl;
      sls.processMsgs(datafiles[iFile]);

      for(unsigned int iSS=0; iSS<sls.ss_objs_.size(); iSS++) {
	//cout << "Object " << iSS << ": class " << sls.ss_objs_[iSS].first << ", " << sls.ss_objs_[iSS].second->size() << " points." << endl;
	SmartScan &ss = *sls.ss_objs_[iSS].second;
	IplImage &img = *sls.left_;
	object obj;
	obj.label = sls.ss_objs_[iSS].first;

	for(unsigned int iDesc=0; iDesc<descriptors_.size(); iDesc++) {
	  cout << "Computing " << descriptors_[iDesc]->name << endl;
	  x=0; y=0; z=0;
	  sls.getRandomPointFromImage(&x, &y, &z, &row, &col);

	  (*descriptors_[iDesc])(ss, img, x, y, z, row, col, &result); 
	  //cout << "nrows: " << tmp.Nrows() << endl;
	  Matrix feat(result->Nrows(), nSamples); feat = 0.0;
	  for(unsigned int iSample=0; iSample<nSamples; iSample++) {
	    
	    //if(!sls.getRandomPointFromImage(&x, &y, &z, &row, &col, obj.label)) { //Gets a point from the object.
	      

	    IplImage *copy;
	    std_msgs::VisualizationMarker mark;
	    if(debug) {
	      publish("videre/cloud", sls.cloud_);
	      copy = cvCloneImage(sls.left_);
	      cvNamedWindow("left", 1);
	      cvCircle(copy, cvPoint(row, col), 5, cvScalar(0,255,0), 1);
	      cvShowImage("left", copy);
	      cvWaitKey(50);
	     
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
	    
	    // -- Compute the descriptor.
	    //If debug, this will wait.
	    result = NULL;
	    (*descriptors_[iDesc])(sls.ss_cloud_, img, x, y, z, row, col, &result, debug);
	    feat.Column(iSample+1) = *result;
	    //TODO: Mark down which descriptors are successful.
	    
	    if(debug) {
	      mark.action = 2;
	      publish("visualizationMarker", mark);
	    }
	  }
	  
	  obj.features[descriptors_[iDesc]->name] = feat;
	}
	
	objs.push_back(obj);
	//cout << "Adding object of class " << obj.label << " with " << obj.features.size() << " descriptors." << endl;
      }
    }

    dd.setObjs(objs);
    dd.save(savename);
    cout << endl << dd.status() << endl;
  }

private:
  DorylusDataset dd;
  //Dorylus dory;
  
};


// bool FixedSpinLarge::operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, bool* success, bool debug) {
//   Matrix result(5,1); result = 5.0;
//   return result;
// }



bool FixedSpinMedium::operator()(SmartScan &ss, const IplImage &img, float x, float y, float z, int row, int col, Matrix** result, bool debug) {

  // -- Reset psi_.
  for(int i=0; i<width_; i++) {
    for(int j=0; j<height_; j++) {
      psi_->setElement(j,i, 0.0);
    }
  }
 

  Matrix* res = new Matrix(height_*width_,1); (*res) = 0.0;
  bool si_success = ss.computeSpinImageFixedOrientation(*psi_, x, y, z, support_, pixelsPerMeter_, string("-y"));
  if(!si_success) {
    *result = res;
    return false;
  }

  cout << "si is " << height_ << " x " << width_ << endl;
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


void FixedSpinMedium::display(const Matrix& result) {
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
  DorylusNode d;

  if(argc > 2 && !strcmp(argv[1], "--dataset")) {

  vector<string> datafiles;
  for(int i=2; i<argc; i++) {
    datafiles.push_back(string(argv[i]));
  }

  d.buildDataset(2, datafiles, string("savename.dd"), true);
  }
  else {
    cout << "Usage: " << endl;
    cout << "  dorylus_node --dataset [BAGFILES]" << endl;
  }

  usleep(250000);
  ros::fini();
  return 0;
}


