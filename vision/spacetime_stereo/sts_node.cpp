
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>

#include <opencv/cv.h>
//#include <opencv/cxmisc.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "ros/node.h"
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/ImageArray.h>
#include <std_msgs/Image.h>
#include "limits.h"
#include "image_utils/cv_bridge.h"
#include <vector.h>

#define SUBPIXEL 16
//#define FILTERS

#define RADIUS 4
#define MAXDISP 120
#define NIMAGES 25

struct calib_params{

float b;		//baseline, meters

float u0;		//x, central point, meters
float v0;		//y, central point, meters

float dpx;	//pixel2meters ratio, x
float dpy;	//pixel2meters ratio, y

float tx;	//translation vector between two views
float ty;
float tz;

float feq;
float fx;	//x focal, pixels
float fy;	//y focal, pixels

};

calib_params cpar;


struct stereo_params{

unsigned char* L;	//original (raw) intensity image (Left) (array)
unsigned char* R;	//original (raw) intensity image (Right) (array)

short int* disp;	//disparity map

int w;
int h;

int maxdisp;
double disp_scale;
int radius;		//radius of correlation window

//filters
int peak_filter;
int rel_filter;		//threshold for rel_filter
int ratio_filter;	//threshold for ratio filter
int unique;		//boolean for application of uniqueness filter

int **mins;		//for spacetime stereo
int firstframe;		//for playing point cloud sequences

};

stereo_params par;

void cv2array(IplImage *A, unsigned char *B){

int w = A->width;
int h = A->height;
int x,y;

for(y=0; y<h; y++){
for(x=0; x<w; x++){
	B[y*w+x] = ((unsigned char *)(A->imageData))[y*A->widthStep +x];
}}

}

void spacetime_stereo(vector<IplImage*> left_frames, vector<IplImage*> right_frames, unsigned int nImages, short int* disp);

// -- ROS Node class for getting Videre images.
class SpacetimeStereoNode : public ros::node
{
	public:
	//IplImage *frame;
	std_msgs::ImageArray frame_msg;
	bool builtBridge;
	CvBridge<std_msgs::Image> *left_bridge_in;
	CvBridge<std_msgs::Image> *right_bridge_in;
	//ros::thread::mutex frame_mutex_;
		
	vector<IplImage*> left_frames;
	vector<IplImage*> right_frames;
	unsigned int nImages;
	
	SpacetimeStereoNode() : ros::node("spacetime_stereo_node"), builtBridge(false), nImages(NIMAGES)
	{
		left_bridge_in = NULL;
		right_bridge_in = NULL;
		
		subscribe("videre/images", frame_msg, &SpacetimeStereoNode::processFrame, 1);
		advertise<std_msgs::PointCloudFloat32>("spacetime_stereo");
	}
	
	~SpacetimeStereoNode()
	{
		delete right_bridge_in;
		delete left_bridge_in;
	}
	
	void compute_point_cloud(calib_params cpar, short int* disp, int w, int h);

	//Copies the image out of the frame_msg and into frame.
	void processFrame() 
	{
		
		if(!builtBridge) {
			left_bridge_in = new CvBridge<std_msgs::Image>(&frame_msg.images[0], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
			right_bridge_in = new CvBridge<std_msgs::Image>(&frame_msg.images[1], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
			builtBridge = true;
		}
		
		if(left_frames.size() != nImages) {
			
		
			IplImage* thisframe_left = NULL;
			IplImage* thisframe_right = NULL;
			left_bridge_in->to_cv(&thisframe_left);
			assert(thisframe_left);
			left_frames.push_back(thisframe_left);
			right_bridge_in->to_cv(&thisframe_right);
			assert(thisframe_right);
			right_frames.push_back(thisframe_right);
			
			
// 			frame_mutex_.unlock();
		}
		else {
			int w = left_frames[0]->width;
			int h = left_frames[0]->height;
			short int* disp = NULL;
			spacetime_stereo(left_frames, right_frames, nImages, disp);
			compute_point_cloud(cpar, disp, w, h);
			
			for(unsigned int i=0; i<nImages; i++){
				cvReleaseImage(&left_frames[i]);	
				cvReleaseImage(&right_frames[i]);
			}
			free(disp);
			
		}
	}  
};








void stereo_standard_sad(stereo_params par){

int maxdisp = par.maxdisp;
int r = par.radius;
int w = par.w;
int h = par.h;

unsigned char *L = par.L;
unsigned char *R = par.R;

int x,y,i,j,d;
int n = 2*r+1;

int *acc = (int *)calloc(maxdisp, sizeof(int));	
int **V = (int **)calloc(w, sizeof(int*));
for(d=0; d<w; d++)
	V[d] = (int *)calloc(maxdisp, sizeof(int)); 

int **mins = par.mins;

//FIRST ROW
//STAGE 1 - init acc
for(d=0; d<maxdisp; d++){
	for(i=maxdisp; i<maxdisp+n; i++){
		V[i][d] = 0;
		for(j=0; j<n; j++){
			V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
		}
		//acc[d] += V[i][d];			
	}		
}

//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	for(d=0; d<maxdisp; d++){
		V[x+r][d] = 0;
		for(j=0; j<n; j++)
			V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
		//acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
	}//d
}//x
	
	
unsigned char *lp, *rp, *lpp, *rpp;
int ind1 = r+r+maxdisp;
//OTHER ROWS
for(y=r+1; y<h-r; y++){
	
	//first position
	for(d=0; d<maxdisp; d++){
		acc[d] = 0;
		for(i=maxdisp; i<maxdisp+n; i++){
			V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
			acc[d] += V[i][d];			
		}	
	}
	
	//other positions
	lp = (unsigned char *) L + (y+r)*w + ind1;
	rp =  (unsigned char *) R + (y+r)*w + ind1;
	lpp = (unsigned char *) L + (y-r-1)*w + ind1;
	rpp = (unsigned char *) R + (y-r-1)*w + ind1;

	for(x=maxdisp+r+1; x<w-r; x++){
		
		lp++;
		rp++;
		lpp++;
		rpp++;
		
		for(d=0; d<maxdisp; d++){
			V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
			rp--;
			rpp--;
			acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
			
			mins[y*w+x][d] += acc[d] ;
		}//d

		rp += maxdisp;
		rpp += maxdisp;	
	}//x
}//y


for(d=0; d<w;d++){
	free(V[d]);
}
free(V);
free(acc);

}


void SpacetimeStereoNode::compute_point_cloud(calib_params cpar, short int* disp, int w, int h){

std_msgs::PointCloudFloat32 ros_cloud;
ros_cloud.set_chan_size(1);
ros_cloud.chan[0].name = "intensities";
int i,j;
float x,y,z;

//int w = Left->width;
//int h = Left->height;
//int wsi = Left->widthStep;
//unsigned char *dL = ((unsigned char *)(Left->imageData));

ros_cloud.set_pts_size(w*h);
ros_cloud.chan[0].set_vals_size(w*h);

for(j=0; j<h; j++)
	for(i=0; i<w; i++)
		if( disp[j*w+i] > 0) {
			z = (cpar.tx * cpar.feq) / ( (disp[j*w+i]*1.0) / SUBPIXEL) ;
			x = ((i - cpar.u0) * z) / cpar.fx;
			y = ((j - cpar.v0) * z) / cpar.fy;
			
			ros_cloud.pts[j*w+i].x = x/100; //from millimiters to decimiters
			ros_cloud.pts[j*w+i].y = y/100;
			ros_cloud.pts[j*w+i].z = z/100;

			ros_cloud.chan[0].vals[j*w+i] = 16*255;
			
		}

ros_cloud.header.frame_id = "FRAMEID_SMALLV";
publish("spacetime_stereo", ros_cloud);


//m_rosNode->publish("full_cloud",ros_cloud);

}



void spacetime_stereo(vector<IplImage*> left_frames, vector<IplImage*> right_frames, unsigned int nImages, short int* disp){

	par.radius = RADIUS;
	par.maxdisp = MAXDISP;
	
	int maxdisp = par.maxdisp;
	int r = par.radius;
	int w = left_frames[0]->width;
	int h = left_frames[0]->height;
	par.w = w;
	par.h = h;
	
	unsigned int framecount;
	int d,x,y;
	int den, alfa;
	int dbest=0;
	int costmin;

	int **mins =  (int **)calloc(w*h, sizeof(int *));
	for(d=0; d<w*h; d++)
		mins[d] =(int *)calloc(maxdisp, sizeof(int));
	par.mins = mins;

	disp = (short int *)malloc(w*h*sizeof(short int));
	par.disp = disp;
	
	//Needed for averaging the intensities of the several stereo reference image	
	unsigned char *L = (unsigned char *)malloc(w*h*sizeof(unsigned char));
	unsigned char *R = (unsigned char *)malloc(w*h*sizeof(unsigned char));
	//unsigned char *L, *R;
	
	//par.Lf = Leftf;
	//par.Rf = Rightf;
	par.L = L;
	par.R = R;
	
	IplImage *Leftg = cvCreateImage(cvSize(w, h), 8, 1);
	IplImage *Rightg = cvCreateImage(cvSize(w, h), 8, 1);
		
	#ifdef FILTERS
	int n = 2*r+1;
	int sqn = n*n;
	int rel_filter = sqn * par.rel_filter * par.nframes;
	int *min_scores = (int *)malloc(w * sizeof(int ));
	int sad_second_min;
	int da, db;
	#endif

	for(framecount = 0; framecount < nImages; framecount ++){

		printf("Processing frame %d .. \n", framecount);
		
		cvCvtColor(left_frames[framecount], Leftg, CV_BGR2GRAY);
		cvCvtColor(right_frames[framecount], Rightg, CV_BGR2GRAY);
		
		cv2array(Leftg, L);
		cv2array(Rightg, R);
		
		stereo_standard_sad(par);

	} //framecount

	for(y=r; y<h-r; y++){
		for(x=maxdisp+r+1; x<w-r; x++){
			
			costmin	= mins[y*w+x][0];
			dbest = 0;
		
			for(d=1; d<maxdisp; d++){
			if(mins[y*w+x][d] < costmin){
					costmin = mins[y*w+x][d];
					dbest = d;
			}}//d
				
			alfa = 0;
			if(dbest > 0){
				den = 2* (mins[y*w+x][dbest+1] + mins[y*w+x][dbest-1]-2*mins[y*w+x][dbest]);
				if(den!=0)
					alfa = (SUBPIXEL*(mins[y*w+x][dbest-1] - mins[y*w+x][dbest+1])) / den;
			}
			disp[y*w+x] = SUBPIXEL*dbest + alfa;
			
			#ifdef FILTERS
			//******* FILTERS ********
			//1) rel filter	
			if(costmin > rel_filter)
				disp[y*w + x] = -2;
	
			//( 2)needed for uniqueness constraint filter )
			min_scores[x] = costmin;
			
			//3) uniqueness filter
			
			sad_second_min = INT_MAX;
			for(d=0; d<dbest-2; d++){
				if(mins[y*w+x][d]<sad_second_min){
					sad_second_min = mins[y*w+x][d];
				}
			}
			for(d=dbest+3; d<maxdisp; d++){
				if(mins[y*w+x][d]<sad_second_min){
					sad_second_min = mins[y*w+x][d];
				}
			}	
			if( costmin*100  > par.ratio_filter*sad_second_min)
				disp[y*w + x] = -1;
			
			//4) Peak Filter
			da = (dbest>1) ? ( mins[y*w+x][dbest-2] - mins[y*w+x][dbest] ) : (mins[y*w+x][dbest+2] - mins[y*w+x][dbest]);
			db =  (dbest<maxdisp-2) ? (mins[y*w+x][dbest+2] - mins[y*w+x][dbest]) : (mins[y*w+x][dbest-2] - mins[y*w+x][dbest]);		
			if(da + db < par.peak_filter)
				disp[y*w + x] = -4;
			//******* FILTERS ********
			#endif
				//disp[y*w+x] = dbest;
		}//x
		
		#ifdef FILTERS
		if(par.unique == 1)
			uniqueness_constraint_reducedrange(par, min_scores, y);
		#endif
	}//y
	
	printf("DONE!\n");

	for(d=0; d<w*h;d++)
		free(mins[d]);	
	free(mins);

	free(L);
	free(R);
	cvReleaseImage(&Leftg);
	cvReleaseImage(&Rightg);
		
	#ifdef FILTERS
	free(min_scores);
	#endif
	
}







int main(int argc, char **argv){

	cpar.tx = 59.934176;
	cpar.u0 = 332.544501;
	cpar.v0 = 254.563215;
	cpar.dpx = 0.006;
	cpar.dpy = 0.006;
	cpar.fx = 945.598886;
	cpar.fy =  950.594644;
	cpar.feq =  (cpar.fx+cpar.fy) / 2;

	ros::init(argc, argv);
	SpacetimeStereoNode sts;
	sts.spin();

// 	static uint32_t count = 0;
// 	std::stringstream ss;
// 	ss << "spacetime_stereo" << count++;
// 	
// 	m_rosNode->init(argc, argv);	
// 	
// 	m_rosNode = new ros::node( ss.str() );
// 	m_rosNode->advertise<std_msgs::PointCloudFloat32>("full_cloud", 5);


return 0;

}






