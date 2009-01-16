/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** 

@mainpage

@htmlinclude manifest.html

This code computes stereo correspondence using the spacetime stereo technique. The input is given by the calibration parameters and a stream of stereo images that must be advertised under ROS (e.g. by means of the sensor_cart package). The output is a point cloud that can be visualized using, e.g.,  the ogre_visualizer package. 

At the very beginning of the file there are the parameters of the algorithm that can be modified by the user. In particular, these are:

 * FILTERS: if defined, three post-filters are used in order to discard wrong stereo correspondences.
 * RADIUS: this is the radius of the spatial window used to compute spacetime stereo. 
 * NIMAGES: this is the number of images of the processed stereo sequence. In other words, this is the temporal window used to compute spacetime stereo. 
 * MAXDISP, XOFFSET: these two parameters define the disparity range. In particular, the disparity range is defined as: [XOFFSET, XOFFSET + MAXDISP]

Advanced use: in some particular cases (e.g. smooth surfaces without big disparity jumps, such as faces) a better qualitative result can be achieved by switching off one of the three post-filters deployed. To do that, the parameter “par.unique” at line 420 can be set to 0. 

**/


//#############################################
//####### SPACE-TIME STEREO code    ###########
//####### author: Federico Tombari  ###########
//####### tombari@willowgarage.com  ###########
//#############################################

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
#include <std_msgs/PointCloud.h>
#include <std_msgs/ImageArray.h>
#include <std_msgs/Image.h>
#include "limits.h"
#include "image_utils/cv_bridge.h"
#include <vector>

#include <std_msgs/String.h>
#include <string>
#include <iostream>

#define SUBPIXEL 16
#define FILTERS

//Radius of spatial window
#define RADIUS 2
//Width of temporal window
#define NIMAGES 40

//Horopter (sensor range)
#define MAXDISP 80
#define XOFFSET 30

//#define DEMO_PR2

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

//calib_params cpar;


struct stereo_params{

unsigned char* L;	//original (raw) intensity image (Left) (array)
unsigned char* R;	//original (raw) intensity image (Right) (array)

short int* disp;	//disparity map

int w;
int h;

int maxdisp;
int xoffset;

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
bool wait;

void cv2array(IplImage *A, unsigned char *B){

int w = A->width;
int h = A->height;
int x,y;

for(y=0; y<h; y++){
for(x=0; x<w; x++){
	B[y*w+x] = ((unsigned char *)(A->imageData))[y*A->widthStep +x];
}}

}



void toggle_wait(int foo){
//printf("Togglin'  \n");
wait = !wait;

}


void uniqueness_constraint_reducedrange(stereo_params par, int* mins, int row){

int x, d, x_oth, d_oth;

int w = par.w;
short int* disp = par.disp;
int maxdisp = par.maxdisp;
int r = par.radius;

//Init array
int* unique;
unique = (int *)malloc(w*sizeof(int));
for(x=r; x<w-r; x++)
	unique[x] = -1;
	
for(x=maxdisp+r+1; x<w-r; x++){
	
	if(disp[row*w + x] >= 0){		//this is needed since previous filters may have already discarded this correspondence
		d = (disp[row*w + x]+8)/SUBPIXEL;
		if(d>maxdisp){
			//printf("%d %d\n", disp[row*w + x], d);
			d=maxdisp;
		}
		//d = disp[row*w + x];
		//UNIQUENESS CONSTRAINT
		
		if(unique[x-d] == -1)
			unique[x-d] = d;
		else{
			d_oth = unique[x-d];	
			x_oth = x - d + d_oth;
			if(mins[x_oth] > mins[x]){
 				unique[x-d] = d;
 				disp[row*w + x_oth] = -3;
 			}
			else{
				disp[row*w + x] = -3;
 			}
		}
	}
}//y

free(unique);
}





void spacetime_stereo(std::vector<IplImage*> left_frames, std::vector<IplImage*> right_frames, unsigned int nImages, short int* disp, string cal_string);
//void extract_cal_params(String cal_string, calib_params cpar);

// -- ROS Node class for getting Videre images.
class SpacetimeStereoNode : public ros::Node
{
	public:
	//IplImage *frame;
	std_msgs::ImageArray frame_msg;
	std_msgs::String cal_msg;
	bool builtBridge;
	CvBridge<std_msgs::Image> *left_bridge_in;
	CvBridge<std_msgs::Image> *right_bridge_in;
	//ros::thread::mutex frame_mutex_;

	bool IsCal;	
	//calib_params cpar;

	std::vector<IplImage*> left_frames;
	std::vector<IplImage*> right_frames;
	unsigned int nImages;
	short int* disp;
		
	#ifdef DEMO_PR2
	SpacetimeStereoNode() : ros::Node("world_3d_map"), builtBridge(false), nImages(NIMAGES)
	#else
	SpacetimeStereoNode() : ros::Node("spacetime_stereo_node"), builtBridge(false), nImages(NIMAGES)
	#endif

	{
		IsCal = 0;

		left_bridge_in = NULL;
		right_bridge_in = NULL;
		
		subscribe("videre/images", frame_msg, &SpacetimeStereoNode::processFrame, 1);
		subscribe("videre/cal_params", cal_msg, &SpacetimeStereoNode::processCal, 1);
		
		#ifdef DEMO_PR2
		advertise<std_msgs::PointCloud>("world_3d_map", 1);
		#else
		advertise<std_msgs::PointCloud>("spacetime_stereo", 5);
		#endif
	
	}
	
	~SpacetimeStereoNode()
	{
		delete right_bridge_in;
		delete left_bridge_in;
	}
	
	//void compute_point_cloud(calib_params cpar, short int* disp, int w, int h);
	void spacetime_stereo(std::vector<IplImage*> left_frames, std::vector<IplImage*> right_frames, unsigned int nImages, short int *disp, string cal_string);

	void processCal(){
		//printf("Got a cal message\n");
		IsCal = 1;
	}
	
	//Copies the image out of the frame_msg and into frame.
	void processFrame() 
	{
		//cvWaitKey(0);

		if(!builtBridge) {
			left_bridge_in = new CvBridge<std_msgs::Image>(&frame_msg.images[1], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
			right_bridge_in = new CvBridge<std_msgs::Image>(&frame_msg.images[0], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
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
			while(!IsCal && ok()){printf("; ");}

			system("killall -s USR1 lpg");
			
			string cal_string = cal_msg.data;

			int w = left_frames[0]->width;
			int h = left_frames[0]->height;
			printf("w: %d , h: %d \n", w, h);
			disp = (short int *)malloc(w*h*sizeof(short int));
			
			spacetime_stereo(left_frames, right_frames, nImages, disp, cal_string);
			//compute_point_cloud(cpar, disp, w, h);
			
			for(unsigned int i=0; i<nImages; i++){
				cvReleaseImage(&left_frames[i]);	
				cvReleaseImage(&right_frames[i]);
			}
			left_frames.clear();
			right_frames.clear();
			free(disp);
			
			//printf("processframe is complete!\n");
	
		}
	}  
};


void stereo_standard_sad(stereo_params par){

int maxdisp = par.maxdisp;
int xoffset = par.xoffset;

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
			V[i][d] += abs( L[j*w+i] - R[j*w+i-d-xoffset] );
		}
		//acc[d] += V[i][d];			
	}		
}

//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	for(d=0; d<maxdisp; d++){
		V[x+r][d] = 0;
		for(j=0; j<n; j++)
			V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d-xoffset] );
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
			V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d-xoffset] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d-xoffset] );
			acc[d] += V[i][d];			
		}	
	}
	
	//other positions
	lp = (unsigned char *) L + (y+r)*w + ind1;
	rp =  (unsigned char *) R + (y+r)*w + ind1 - xoffset;
	lpp = (unsigned char *) L + (y-r-1)*w + ind1;
	rpp = (unsigned char *) R + (y-r-1)*w + ind1 - xoffset;

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


void SpacetimeStereoNode::spacetime_stereo(std::vector<IplImage*> left_frames, std::vector<IplImage*> right_frames, unsigned int nImages, short int* disp, string cal_string){

	par.radius = RADIUS;
	par.maxdisp = MAXDISP;
	par.xoffset = XOFFSET;	
	
	int maxdisp = par.maxdisp;
	//int xoffset = par.xoffset;

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
	
	unsigned char *L = (unsigned char *)malloc(w*h*sizeof(unsigned char));
	unsigned char *R = (unsigned char *)malloc(w*h*sizeof(unsigned char));

	par.L = L;
	par.R = R;
	
	IplImage *Leftg = cvCreateImage(cvSize(w, h), 8, 1);
	IplImage *Rightg = cvCreateImage(cvSize(w, h), 8, 1);
		
	#ifdef FILTERS
	par.unique = 0;
	par.ratio_filter = 90;
	par.peak_filter = 120;

	//int n = 2*r+1;
	//int sqn = n*n;
	//int rel_filter = sqn * par.rel_filter * par.nframes;
	int *min_scores = (int *)malloc(w * sizeof(int ));
	int sad_second_min;
	int da, db;
	#endif

	printf("Channel n: %d %d\n", left_frames[0]->nChannels, right_frames[0]->nChannels);
	printf("Processing frame ");

	for(framecount = 0; framecount < nImages; framecount ++){

		printf("%d\r", framecount);
		fflush(stdout);

		cvCvtColor(left_frames[framecount], Leftg, CV_BGR2GRAY);
		cvCvtColor(right_frames[framecount], Rightg, CV_BGR2GRAY);
		//cvShowImage("Reference Image", left_frames[framecount]);
		//cvWaitKey(100);
		cv2array(Leftg, L);
		cv2array(Rightg, R);
		
		stereo_standard_sad(par);
		
	} //framecount
	printf("\n");

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
			if(dbest > 0 & dbest<maxdisp-1){
				den = 2* (mins[y*w+x][dbest+1] + mins[y*w+x][dbest-1]-2*mins[y*w+x][dbest]);
				if(den!=0)
					alfa = (SUBPIXEL*(mins[y*w+x][dbest-1] - mins[y*w+x][dbest+1])) / den;
			}
			disp[y*w+x] = SUBPIXEL*dbest + alfa;
			
			#ifdef FILTERS
			//******* FILTERS ********
			//1) rel filter	
			//if(costmin > rel_filter)
			//	disp[y*w + x] = -2;
	
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

	for(d=0; d<w*h;d++)
		free(mins[d]);	
	free(mins);

	free(L);
	free(R);
	cvReleaseImage(&Leftg);
	cvReleaseImage(&Rightg);
	cvDestroyWindow("Reference Image");
	#ifdef FILTERS
	free(min_scores);
	#endif
	printf("Finished computing stereo disparities \n");

//BEGIN COMPUTING POINT CLOUD
printf("Computing Point Cloud; w: %d h:%d ...", w, h);

calib_params cpar;

int position = cal_string.find("Tx");
string substring = cal_string.substr(position, cal_string.find("Ty") - position);
sscanf(substring.c_str(), "%*s %f", &cpar.tx);

position = cal_string.find("proj");
substring = cal_string.substr(position, cal_string.find("rect")-position);
sscanf(substring.c_str(), "%*s \n %f %*f %f %*f \n %*f %f %f", &cpar.fx, &cpar.u0, &cpar.fy, &cpar.v0);

cpar.feq =  (cpar.fx+cpar.fy) / 2;

printf("\nReading Parameters: tx: %f, u0: %f, v0: %f, fx:%f, fy:%f\n", cpar.tx, cpar.u0, cpar.v0, cpar.fx, cpar.fy);

std_msgs::PointCloud ros_cloud;

ros_cloud.set_chan_size(1);
ros_cloud.chan[0].name = "intensities";

int i,j;
float xf,yf,zf;

int point_count = 0, tot_points=0;
for(j=0; j<h; j++)
for(i=0; i<w; i++)
if( disp[j*w+i] > 0) 
	tot_points ++;

ros_cloud.set_pts_size(tot_points);
ros_cloud.chan[0].set_vals_size(tot_points);

for(j=0; j<h; j++)
for(i=0; i<w; i++)
//if( disp[j*w+i] > 0) {
#ifdef DEMO_PR2
if( disp[j*w+i] > 40*16 && disp[j*w+i] < 63*16) {
#else
if( disp[j*w+i] > 0) {
#endif
	zf = (abs(cpar.tx) * cpar.feq) / ( XOFFSET + ( (disp[j*w+i]*1.0) / SUBPIXEL) ) ;
	xf = ((i - cpar.u0) * zf) / cpar.fx;
	yf = ((j - cpar.v0) * zf) / cpar.fy;
	
	#ifdef DEMO_PR2
	ros_cloud.pts[point_count].x = -1.42 +zf/300;//xf/1000; //from millimiters to decimiters
	ros_cloud.pts[point_count].y = xf/300;//yf/1000; //Why does this need to be flipped?
	ros_cloud.pts[point_count].z = .75-yf/300;//zf/1000; //Why does this need to be flipped?
	#else
	ros_cloud.pts[point_count].x = xf/1000; //from millimiters to decimiters
	ros_cloud.pts[point_count].y = yf/1000; //Why does this need to be flipped?
	ros_cloud.pts[point_count].z = zf/1000; //Why does this need to be flipped?
	#endif

	ros_cloud.chan[0].vals[point_count] = 16*255;
	point_count++;
}

//printf("DON");

#ifdef DEMO_PR2
ros_cloud.header.frame_id = "FRAMEID_MAP";
publish("world_3d_map", ros_cloud);
#else
ros_cloud.header.frame_id = "FRAMEID_SMALLV";
publish("spacetime_stereo", ros_cloud);
#endif

signal(SIGUSR1, toggle_wait);

//#ifdef DISPLAY

IplImage *idisp = cvCreateImage(cvSize(w, h), 8, 3);
cvZero(idisp);

for(int y=RADIUS; y<h-RADIUS; y++){
for(int x=MAXDISP+RADIUS+1; x<w-RADIUS; x++){
	if(disp[y*w+x]-8 < 0){
		((unsigned char *)(idisp->imageData))[y*idisp->widthStep+x*3] = 255;
		((unsigned char *)(idisp->imageData))[y*idisp->widthStep+x*3+1] = 0;
		((unsigned char *)(idisp->imageData))[y*idisp->widthStep+x*3+2] = 0;
	}
	else{
		((unsigned char *)(idisp->imageData))[y*idisp->widthStep+x*3] = disp[y*w+x]/8;
		((unsigned char *)(idisp->imageData))[y*idisp->widthStep+x*3+1] = disp[y*w+x]/8;
		((unsigned char *)(idisp->imageData))[y*idisp->widthStep+x*3+2] = disp[y*w+x]/8;
	}
}}

printf("Done; press q to quit, space-key to continue.. \n");

cvShowImage("Disp Image", idisp);
//cvShowImage("Ref Image", left_frames[0]);
//cvShowImage("Tar Image", right_frames[0]);
cvReleaseImage(&idisp);

wait = 1;
int key = 'a';
while(wait){
	key = cvWaitKey(10);
	if(key == ' ')
		wait = 0;
	if (key=='q')
		exit(1);
}

system("killall -s USR1 lpg");

}




int main(int argc, char **argv){

//#ifdef DISPLAY
cvNamedWindow("Disp Image", 1);
//cvNamedWindow("Ref Image", 1);
//cvNamedWindow("Tar Image", 1);
//#endif


ros::init(argc, argv);

SpacetimeStereoNode sts;
sts.spin();

//#ifdef DISPLAY
cvDestroyWindow("Disp Image");
//cvDestroyWindow("Ref Image");
//vDestroyWindow("Tar Image");
//#endifc

return 0;

}






