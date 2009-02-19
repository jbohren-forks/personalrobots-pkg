//Read the xml files produced by the script session_results.py
// And turn than into image masks
// Gary Bradski, 2/11/09
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


using namespace std;
// Read a file into a vector
void help(char *program) {
  cout << "\n" << program << " <[path/]filelist> "<<"<xml_dir>" << " <base_dir>" << " <widthxheight>" << " [scale_factor]" << endl;
  cout << "     filelist      file should be a list of the xml files with no path information\n" << endl;
  cout << "     xml_dir       is where the xml files, listed in filelist reside\n" << endl;
  cout << "     base_dir      is the directory that contains the original image sub-directories\n" << endl; 
  cout << "     widthxheight  for example: 640x480\n" << endl;
  cout << "     scale_factor  If images was downsampled prior to submit_img.py, upsample with this, DEFAULT: 1.0\n\n" << endl;
}

//SUPPORT FUNCTIONS
int xy_pairs_from_polygon_string(string s, vector<float> &x, vector<float> &y, CvRect &r, float image_scaling = 1.0)
{
	x.clear(); y.clear();
	string::size_type idx1,idx2 = 0,idxlen;
	string sval;
	float val;
	float xmin = 50000000.0,xmax = 0.0,ymin = 50000000.0, ymax = 0.0; //Calculate bounding box while were at it
	while(1)
	{
		//Parse out the schtuff
		//x
		if((idx1 = s.find("x=\"",idx2)) == string::npos) break;
		idx1 += 3;
		if((idx2 = s.find("\"",idx1)) == string::npos) break;
		idxlen = idx2 - idx1;
		sval = s.substr(idx1,idxlen); //Got the value of x
		val = atof(sval.c_str())*image_scaling;
		x.push_back(val);
		if(val < xmin) xmin = val;
		if(val > xmax) xmax = val;
		//y
		if((idx1 = s.find("y=\"",idx2)) == string::npos) break;
		idx1 += 3;
		if((idx2 = s.find("\"",idx1)) == string::npos) break;
		idxlen = idx2 - idx1;
		sval = s.substr(idx1,idxlen);
		val = atof(sval.c_str())*image_scaling;
		y.push_back(val);
		if(val < ymin) ymin = val;
		if(val > ymax) ymax = val;
	}
	//Calculate CvRect
	r.x = (int)(xmin + 0.5);
	r.y = (int)(ymin + 0.5);
	r.width = (int)(xmax - xmin + 0.5);
	r.height = (int)(ymax - ymin + 0.5);
	//Report our results
	int xylen = (int)x.size();
	if (xylen > (int)y.size()) xylen = (int)y.size();
	return xylen;
} 
			

void image_mask_out(const char *path_filename, int width, int height, vector<float> &x, vector<float> &y)
{
 	IplImage *I = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
 	cvSetZero(I); //clean image to write "white" on.
 	int len = (int)x.size();
 	if(len > (int)y.size()) len = (int)y.size();
 	//Set the poly points into an array
 	CvPoint *p = new CvPoint [len];
 	for(int i = 0; i<len; ++i)
 	{
 		p[i] = cvPoint(x[i],y[i]);
	}
	//Draw the mask
	int npts[3];
	npts[0] = len;
	cvFillPoly(I,&p, npts, 1, CV_RGB(255,255,255));
	//Store it away
	cvSaveImage(path_filename, I);
 	delete [] p;
	cvReleaseImage(&I); 	
}
  	
//======================================================================================
int main(int argc, char* argv[])
{
  //Command line stuff
  if (argc < 5) { help(argv[0]); return 1; }
  std::vector<std::string> file;
  std::string line;
  file.clear();
  std::ifstream infile (argv[1], std::ios_base::in);
  std::string xml_dir = argv[2];
  std::string base_dir = argv[3];
  std::string x_by_y = argv[4];
  string::size_type xmark = x_by_y.find("x");
  if(xmark == string::npos) { cout << "Error in param " << x_by_y << ", widthxheight (example 640x480)" << endl; return -1;} 
  int width = atoi((x_by_y.substr(0,xmark)).c_str());
  int height = atoi((x_by_y.substr(xmark+1, string::npos)).c_str());
  float image_scaling = 1.0;
  if(argc > 5)
  	image_scaling = atof(argv[5]);
  cout << "Width = " << width << ", height= " << height << "; scale = " << image_scaling << endl;

  //Get the list of xml files ...
  while (getline(infile, line, '\n'))
  {
    file.push_back (line);
  }
  infile.close();
  std::cout << "Read " << file.size() << " lines.\n";
  
  //Parse into directories and files by looking for the "__" separator
  vector<string> img_dir,img_file;
  string::size_type beg1,len1,beg2,len2;
  int i;
  for(i=0; i<(int)file.size(); ++i)
  {
  	cout << i << ": "; //<< " From " << file[i] << endl;
  		beg1 = 0;
  		if((len1 = file[i].find("__")) == string::npos) break; //protect against not finding anything
  		len1 -= beg1;
  		beg2 = len1 + beg1 + 2;
  		if((len2 = file[i].find(".xml")) == string::npos) break;
  		len2 -= beg2;
  		img_dir.push_back(file[i].substr(beg1,len1));
  		img_file.push_back(file[i].substr(beg2,len2));
  		cout << img_dir[i] << ", " << img_file[i] << endl;
	}
	int num_files = i;
	
	//Now attack the xml files themselves
  	string::size_type idx1,idx2,idxlen;
 	std::ifstream xml_file;
 	std::ofstream out_file;
 	int swidth = (int)((float)width*image_scaling + 0.5); //Scaled width and height
 	int sheight = (int)((float)height*image_scaling + 0.5);
  	for(i=0; i<num_files; ++i) // For each xml file
  	{
  		string file_name = xml_dir + "/" + img_dir[i] + "__" + img_file[i] + ".xml"; //dir + recreate file string
  		string file_out =  base_dir + "/" + img_dir[i] + "/" + img_file[i] + "_CvRects.csv"; //For writing rectangles
  		string Imask_out = base_dir + "/" + img_dir[i] + "/" + img_file[i] + "_Mask"; //Base name for writing out image masks
  		cout << "Opening " << file_name << ", rectangles and masks will be stored in: " << file_out << endl;
  		xml_file.open(file_name.c_str(), ifstream::in);
  		out_file.open(file_out.c_str());
  		string xml_strings, polygon;
  		//Read the xml file into one big string
  		while(getline(xml_file, line, '\n'))
  		{
  			xml_strings += line;
		}
		//Parse out all polygon strings in this xml file
		vector<vector<float> > X,Y;
		vector<CvRect> R;
		vector<float> x,y;
		CvRect r;
		idx2 = 0;
		while(1)
		{
			if((idx1 = xml_strings.find("<polygon",idx2)) == string::npos) break;
			if((idx2 = xml_strings.find("</polygon",idx1 + 8)) == string::npos) break;
			idx2 += 9;
			idxlen = idx2 - idx1;
			if(xy_pairs_from_polygon_string(xml_strings.substr(idx1,idxlen), x, y, r, image_scaling) > 0) {
				X.push_back(x); Y.push_back(y); R.push_back(r);
			}
		}
		cout << "  * Found " << X.size() << " strings" << endl;
		//RECORD IMAGE MASKS AND BOUNDING BOXES FOR EACH RECTANGLE
		int polies = (int)X.size();
		cout << "   MASKS:" << endl;
		for(int xx = 0; xx<polies; ++xx)
		{
			out_file << "xywh" << xx << ", " << R[xx].x << ", " << R[xx].y << ", " << R[xx].width << ", " << R[xx].height << "," << endl;
			std::stringstream numb;
			numb << xx;
			string img_out = Imask_out + "_" + numb.str() + ".png";
			cout << "   " << img_out << endl;   
			image_mask_out(img_out.c_str(), swidth, sheight, X[xx], Y[xx]);
		}
		xml_strings.clear();
		xml_file.close();
		out_file.close();
	}
  	
	cout << "\nDONE\n" << endl;  	
  return 0;
}


