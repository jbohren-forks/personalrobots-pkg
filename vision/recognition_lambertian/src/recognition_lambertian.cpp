/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Marius Muja

#include <vector>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <queue>


#include "opencv_latest/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "robot_msgs/PointStamped.h"
#include "robot_msgs/Door.h"
#include "visualization_msgs/Marker.h"

#include <string>

// transform library
#include <tf/transform_listener.h>

#include "topic_synchronizer/topic_synchronizer.h"

#include "CvStereoCamModel.h"

#include <boost/thread.hpp>

using namespace std;


void on_edges_low(int);
void on_edges_high(int);


#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)


typedef pair<int,int> coordinate_t;
typedef float orientation_t;;
typedef vector<coordinate_t> template_coords_t;
typedef vector<orientation_t> template_orientations_t;


class ChamferTemplate
{
public:
	template_coords_t coords;
	template_orientations_t orientations;
	CvSize size;
};

class ChamferMatch
{
public:
	CvPoint offset;
	float distance;
	const ChamferTemplate* tpl;
};

class ChamferMatching
{


	float min_scale;
	float max_scale;
	int count_scale;

	float truncate;

	vector<ChamferTemplate*> templates;

public:
	ChamferMatching() : min_scale(1), max_scale(1), count_scale(1), truncate(20)
	{

	}

	~ChamferMatching()
	{
		for (size_t i = 0; i<templates.size(); i++) {
			delete templates[i];
		}
	}

	void addTemplateFromImage(IplImage* templ)
	{
		ROS_INFO("Loading templates");
		for(int i = 0; i < count_scale; ++i) {
			float scale = min_scale + (max_scale - min_scale)*i/count_scale;
			int width = int(templ->width*scale);
			int height = int(templ->height*scale);

			printf("Level: %d, scale: %f, width: %d, height: %d\n", i, scale, width, height);

			// resize edge image to the required scale
			IplImage* templ_scale = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
			cvResize(templ, templ_scale, CV_INTER_NN);

			ChamferTemplate* cmt = new ChamferTemplate();
			extractTemplatePoints(templ_scale, *cmt);
			cmt->size = cvSize(width, height);

			templates.push_back(cmt);

			cvReleaseImage(&templ_scale);
		}
	}


	ChamferMatch matchEdgeImage(IplImage* edge_img)
	{
		ChamferMatch cm;

		cm.distance = 1e10; // very big distance
		IplImage* dist_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32F, 1);
		IplImage* orientation_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32F, 1);
		IplImage* annotated_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32S, 2);
		ROS_INFO("Computing distance transform");
		IplImage* edge_clone = cvCloneImage(edge_img);
		computeImageDirections(edge_clone, orientation_img );
		cvReleaseImage(&edge_clone);
		computeDistanceTransform(edge_img,dist_img, annotated_img, orientation_img, truncate);
		fillNonContourOrientations(annotated_img, orientation_img);

		ROS_INFO("Template matching");
		matchTemplates(dist_img, orientation_img, cm);

		ROS_INFO("Finishing");
		cvReleaseImage(&dist_img);
		cvReleaseImage(&orientation_img);
		cvReleaseImage(&annotated_img);

		return cm;
	}


	ChamferMatch matchImage(IplImage* img, int edge_threshold = 160)
	{
		IplImage *edge_img = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvCvtColor(img, edge_img, CV_RGB2GRAY);
		cvCanny(edge_img, edge_img, edge_threshold/2, edge_threshold);

		ChamferMatch cm = matchEdgeImage(edge_img);

		cvReleaseImage(&edge_img);
		return cm;
	}

private:


	bool findFirstContourPoint(IplImage* templ_img, coordinate_t& p)
	{
		unsigned char* ptr = (unsigned char*) templ_img->imageData;
		for (int y=0;y<templ_img->height;++y) {
			for (int x=0;x<templ_img->width;++x) {
				if (*(ptr+y*templ_img->widthStep+x)!=0) {
					p.first = x;
					p.second = y;
					return true;
				}
			}
		}
		return false;
	}


	/**
	 * Method that extracts a single continuous contour from an image given a starting point.
	 * When it extracts the contour it tries to maintain the same direction (at a T-join for example).
	 *
	 * @param templ_img
	 * @param coords
	 * @param crt
	 */
	void followContour(IplImage* templ_img, template_coords_t& coords, int direction = -1)
	{
		const int dir[][2] = { {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1}, {1,0}, {1,-1}, {0,-1} };
//		const int pos[] = { 0, 1, 2, 7, -1, 3, 6, 5, 4};
		coordinate_t next;
		coordinate_t next_temp;
		unsigned char* ptr;
		unsigned char* ptr_temp;

		assert (direction==-1 || !coords.empty());

		coordinate_t crt = coords.back();
//		printf("Enter followContour, point: (%d,%d)\n", crt.first, crt.second);

		// mark the current pixel as visited
		CV_PIXEL(unsigned char, templ_img, crt.first, crt.second)[0] = 0;
		if (direction==-1) {
//			printf("Initial point\n");
			for (int j = 0 ;j<7; ++j) {
				next.first = crt.first + dir[j][1];
				next.second = crt.second + dir[j][0];
				ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);
				if (*ptr!=0) {
//					*ptr = 0;
					coords.push_back(next);
					followContour(templ_img, coords,j);
					// try to continue contour in the other direction
//					printf("Reversing direction");
					reverse(coords.begin(), coords.end());
					followContour(templ_img, coords, (j+4)%8);
					break;
				}
			}
		}
		else {
			coordinate_t prev = coords.at(coords.size()-2);
//			printf("Prev point: (%d,%d)\n", prev.first, prev.second);
			int k = direction;
//			printf("Direction %d, offset (%d,%d)\n", k, dir[k][0], dir[k][1]);
			next.first = crt.first + dir[k][1];
			next.second = crt.second + dir[k][0];
			ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);

			if (*ptr!=0) {
//				*ptr = 0;
				next_temp.first = crt.first + dir[(k+7)%8][1];
				next_temp.second = crt.second + dir[(k+7)%8][0];
				ptr_temp = CV_PIXEL(unsigned char, templ_img,  next_temp.first, next_temp.second);
				if (*ptr_temp!=0) {
					*ptr_temp=0;
					coords.push_back(next_temp);
				}
				next_temp.first = crt.first + dir[(k+1)%8][1];
				next_temp.second = crt.second + dir[(k+1)%8][0];
				ptr_temp = CV_PIXEL(unsigned char, templ_img,  next_temp.first, next_temp.second);
				if (*ptr_temp!=0) {
					*ptr_temp=0;
					coords.push_back(next_temp);
				}
				coords.push_back(next);
				followContour(templ_img, coords, k);
			} else {
				int p = k;
				int n = k;

				for (int j = 0 ;j<3; ++j) {
					p = (p + 7) % 8;
					n = (n + 1) % 8;
					next.first = crt.first + dir[p][1];
					next.second = crt.second + dir[p][0];
					ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);
					if (*ptr!=0) {
//						*ptr = 0;
						next_temp.first = crt.first + dir[(p+7)%8][1];
						next_temp.second = crt.second + dir[(p+7)%8][0];
						ptr_temp = CV_PIXEL(unsigned char, templ_img,  next_temp.first, next_temp.second);
						if (*ptr_temp!=0) {
							*ptr_temp=0;
							coords.push_back(next_temp);
						}
						coords.push_back(next);
						followContour(templ_img, coords, p);
						break;
					}
					next.first = crt.first + dir[n][1];
					next.second = crt.second + dir[n][0];
					ptr = CV_PIXEL(unsigned char, templ_img, next.first, next.second);
					if (*ptr!=0) {
//						*ptr = 0;
						next_temp.first = crt.first + dir[(n+1)%8][1];
						next_temp.second = crt.second + dir[(n+1)%8][0];
						ptr_temp = CV_PIXEL(unsigned char, templ_img,  next_temp.first, next_temp.second);
						if (*ptr_temp!=0) {
							*ptr_temp=0;
							coords.push_back(next_temp);
						}
						coords.push_back(next);
						followContour(templ_img, coords, n);
						break;
					}
				}
			}
		}
	}

	bool findContour(IplImage* templ_img, template_coords_t& coords)
	{
		coordinate_t start_point;

		bool found = findFirstContourPoint(templ_img,start_point);
		if (found) {
			coords.push_back(start_point);
			followContour(templ_img, coords);
			return true;
		}

		return false;
	}


	float getAngle(coordinate_t a, coordinate_t b, int& dx, int& dy)
	{
		dx = b.first-a.first;
		dy = -(b.second-a.second);  // in image coordinated Y axis points downward
		float angle = atan2(dy,dx);

		if (angle<0) {
			angle+=M_PI;
		}

		return angle;
	}

	float r2d(float rad)
	{
		return 180*rad/M_PI;
	}

	void findContourOrientations(const template_coords_t& coords, template_orientations_t& orientations)
	{
		const int M = 5;
		int coords_size = coords.size();

		vector<float> angles(2*M);
		orientations.insert(orientations.begin(), coords_size, float(-3*M_PI)); // mark as invalid in the beginning

		if (coords_size<2*M+1) {  // if contour not long enough to estimate orientations, abort
			return;
		}

		for (int i=M;i<coords_size-M;++i) {
			coordinate_t crt = coords[i];
			coordinate_t other;
			int k = 0;
			int dx, dy;
			// compute previous M angles
			for (int j=M;j>0;--j) {
				other = coords[i-j];
				angles[k++] = getAngle(other,crt, dx, dy);
			}
			// compute next M angles
			for (int j=1;j<=M;++j) {
				other = coords[i+j];
				angles[k++] = getAngle(crt, other, dx, dy);
			}

			// sort angles
			sort(angles.begin(), angles.end());

			// average them to compute tangent
			orientations[i] = (angles[M-1]+angles[M])/2;
		}
	}


	void drawContour(IplImage* templ_color, const template_coords_t& coords, const template_orientations_t& orientations)
	{
//		IplImage* templ_color = cvCreateImage(cvSize(templ_img->width, templ_img->height), IPL_DEPTH_8U, 3);
//		cvCvtColor(templ_img, templ_color, CV_GRAY2RGB);


		for (size_t i=0;i<coords.size();++i) {

			int x = coords[i].first;
    		int y = coords[i].second;
    		CV_PIXEL(unsigned char, templ_color,x,y)[1] = 255;

//    		if (x==101 && y==55)
    		if (i%3==0) {
    			if (orientations[i] < -M_PI) {
    				continue;
    			}
    			CvPoint p1;
    			p1.x = x;
    			p1.y = y;
    			CvPoint p2;
    			p2.x = x + 10*sin(orientations[i]);
    			p2.y = y + 10*cos(orientations[i]);

    			cvLine(templ_color, p1,p2, CV_RGB(255,0,0));
    		}
		}

		cvNamedWindow("templ",1);
		cvShowImage("templ",templ_color);

//		cvReleaseImage(&templ_color);
//		cvWaitKey(0);
	}


	void extractTemplatePoints(IplImage* templ_img, ChamferTemplate& ct)
    {
		IplImage* templ_color = cvCreateImage(cvSize(templ_img->width, templ_img->height), IPL_DEPTH_8U, 3);
		cvCvtColor(templ_img, templ_color, CV_GRAY2RGB);

		template_coords_t coords;
		template_orientations_t orientations;

		while (findContour(templ_img, coords)) {
			findContourOrientations(coords, orientations);

			drawContour(templ_color, coords, orientations);

			ct.coords.insert(ct.coords.end(), coords.begin(), coords.end());
			ct.orientations.insert(ct.orientations.end(), orientations.begin(), orientations.end());
			coords.clear();
			orientations.clear();
		}


//		cvNamedWindow("templ",1);
//		cvShowImage("templ",templ_color);

		cvReleaseImage(&templ_color);
    }



//    /**
//     * @param edges_img - input images
//     * @param dist_img - output distance image (IPL_DEPTH_32F)
//     */
//    void computeDistanceTransform_(IplImage* edges_img, IplImage* dist_img, float truncate)
//    {
//    	cvNot(edges_img, edges_img);
//    	cvDistTransform(edges_img, dist_img);
//    	cvNot(edges_img, edges_img);
//
//    	if (truncate>0) {
//    		cvMinS(dist_img, truncate, dist_img);
//    	}
//    }


    /**
     * Alternative version of computeDistanceTransform, will probably be used to compute distance
     * transform annotated with edge orientation.
     */
    void computeDistanceTransform(IplImage* edges_img, IplImage* dist_img, IplImage* annotate_img, IplImage* orientation_img, float truncate = -1, float a = 1.0, float b = 0.5 )
    {
    	int d[][2] = { {-1,-1}, { 0,-1}, { 1,-1},
					  {-1,0},          { 1,0},
					  {-1,1}, { 0,1}, { 1,1} };

    	ROS_INFO("Computing distance transform");

    	CvSize s = cvGetSize(edges_img);
    	int w = s.width;
    	int h = s.height;
    	// set distance to the edge pixels to 0 and put them in the queue
    	queue<pair<int,int> > q;
    	for (int y=0;y<h;++y) {
    		for (int x=0;x<w;++x) {

    			unsigned char edge_val = CV_PIXEL(unsigned char, edges_img, x,y)[0];
    			float orientation_val =  CV_PIXEL(float, orientation_img, x,y)[0];

    			if ( (edge_val!=0) && !(orientation_val<-M_PI) ) {
    				q.push(make_pair(x,y));
        			CV_PIXEL(float, dist_img, x, y)[0] = 0;

        			if (annotate_img!=NULL) {
        				int *aptr = CV_PIXEL(int,annotate_img,x,y);
        				aptr[0] = x;
        				aptr[1] = y;
        			}
    			}
    			else {
    				CV_PIXEL(float, dist_img, x, y)[0] = -1;
    			}
    		}
    	}

    	// breadth first computation of distance transform
    	pair<int,int> crt;
    	while (!q.empty()) {
    		crt = q.front();
    		q.pop();

    		int x = crt.first;
    		int y = crt.second;
    		float dist_orig = CV_PIXEL(float, dist_img, x, y)[0];
    		float dist;

    		for (size_t i=0;i<sizeof(d)/sizeof(d[0]);++i) {
    			int nx = x + d[i][0];
    			int ny = y + d[i][1];

    			if (nx<0 || ny<0 || nx>=w || ny>=h) continue;

    			if (abs(d[i][0]+d[i][1])==1) {
    				dist = dist_orig+a;
    			}
    			else {
    				dist = dist_orig+b;
    			}

    			float* dt = CV_PIXEL(float, dist_img, nx, ny);

    			if (*dt==-1 || *dt>dist) {
    				*dt = dist;
    				q.push(make_pair(nx,ny));

    				if (annotate_img!=NULL) {
    					int *aptr = CV_PIXEL(int,annotate_img,nx,ny);
    					int *optr = CV_PIXEL(int,annotate_img,x,y);
    					aptr[0] = optr[0];
    					aptr[1] = optr[1];

    				}


    			}
    		}

    	}

    	// truncate dt
    	if (truncate>0) {
    		cvMinS(dist_img, truncate, dist_img);
    	}
    }


    void computeImageDirections(IplImage* edge_img, IplImage* orientation_img)
    {
    	IplImage* img_color = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_8U, 3);
    	cvCvtColor(edge_img, img_color, CV_GRAY2RGB);

    	template_coords_t coords;
    	template_orientations_t orientations;

    	while (findContour(edge_img, coords)) {
    		findContourOrientations(coords, orientations);

    		drawContour(img_color, coords, orientations);


    		// set orientation pixel in orientation image
    		for (size_t i = 0; i<coords.size();++i) {
    			int x = coords[i].first;
    			int y = coords[i].second;
    			CV_PIXEL(float, orientation_img, x, y)[0] = orientations[i];
    		}

    		coords.clear();
    		orientations.clear();
    	}

    	cvReleaseImage(&img_color);
    }


    void fillNonContourOrientations(IplImage* annotated_img, IplImage* orientation_img)
    {
    	int width = annotated_img->width;
    	int height = annotated_img->height;

    	assert(orientation_img->width==width && orientation_img->height==height);

    	for (int y=0;y<height;++y) {
    		for (int x=0;x<width;++x) {
    			int* ptr = CV_PIXEL(int, annotated_img, x,y);
    			int xorig = ptr[0];
    			int yorig = ptr[1];

    			if (x!=xorig || y!=yorig) {
    				float val = CV_PIXEL(float,orientation_img,xorig,yorig)[0];
    				CV_PIXEL(float,orientation_img,x,y)[0] = val;
    			}
    		}
    	}
    }


    float orientation_diff(float o1, float o2)
    {
    	return fabs(o1-o2);
    }

    float localChamferDistance(IplImage* dist_img, IplImage* orientation_img, const vector<int>& templ_addr, const vector<float>& templ_orientations, CvPoint offset, float alpha = 0.7)
    {
    	int x = offset.x;
    	int y = offset.y;
    	float sum_distance = 0;
    	float sum_orientation = 0;

    	float beta = 1-alpha;

    	float* ptr = (float*) dist_img->imageData;
    	float* optr = (float*) orientation_img->imageData;
    	ptr += (y*dist_img->width+x);
    	optr += (y*orientation_img->width+x);
    	for (size_t i=0;i<templ_addr.size();++i) {
    		sum_distance += *(ptr+templ_addr[i]);
    	}
    	for (size_t i=0;i<templ_addr.size();++i) {
    		sum_orientation += orientation_diff(templ_orientations[i], *(optr+templ_addr[i]));
    	}
    	return (beta*sum_distance/truncate+alpha*sum_orientation/M_PI)/templ_addr.size();
    }



    void matchTemplate(IplImage* dist_img, IplImage* orientation_img, const ChamferTemplate& tpl, ChamferMatch& cm)
    {
    	int width = dist_img->width;

    	const template_coords_t& coords = tpl.coords;
    	// compute template address offsets
    	vector<int> templ_addr;
    	templ_addr.clear();
    	for (size_t i= 0; i<coords.size();++i) {
    		templ_addr.push_back(coords[i].second*width+coords[i].first);
    	}

    	// do sliding window
    	for (int y=0;y<dist_img->height - tpl.size.height; y+=2) {
    		for (int x=0;x<dist_img->width - tpl.size.width; x+=2) {
 				CvPoint test_offset;
 				test_offset.x = x;
 				test_offset.y = y;
 				float test_dist = localChamferDistance(dist_img, orientation_img, templ_addr, tpl.orientations, test_offset);

 				if (test_dist<cm.distance) {
 					cm.distance = test_dist;
 					cm.offset = test_offset;
 					cm.tpl = &tpl;
 				}
    		}
    	}
    }


    void matchTemplates(IplImage* dist_img, IplImage* orientation_img, ChamferMatch& cm)
    {
 	   for(size_t i = 0; i < templates.size(); i++) {
 		   matchTemplate(dist_img, orientation_img, *templates[i],cm);
 	   }
    }



};






class RecognitionLambertian : public ros::Node
{
public:


	image_msgs::Image limage;
	image_msgs::Image rimage;
	image_msgs::Image dimage;
	image_msgs::StereoInfo stinfo;
	image_msgs::DisparityInfo dispinfo;
	image_msgs::CamInfo rcinfo;
	image_msgs::CvBridge lbridge;
	image_msgs::CvBridge rbridge;
	image_msgs::CvBridge dbridge;

	robot_msgs::PointCloud cloud_fetch;
	robot_msgs::PointCloud cloud;

	IplImage* left;
	IplImage* right;
	IplImage* disp;
	IplImage* disp_clone;

	TopicSynchronizer<RecognitionLambertian> sync;

	boost::mutex cv_mutex;
	boost::condition_variable images_ready;

	tf::TransformListener *tf_;


	// minimum height to look at (in base_link frame)
	double min_height;
	// maximum height to look at (in base_link frame)
	double max_height;
	// no. of frames to detect handle in
	int frames_no;
	// display stereo images ?
	bool display;

	int edges_low;
	int edges_high;


	ChamferMatching* cm;

    RecognitionLambertian()
    :ros::Node("stereo_view"), left(NULL), right(NULL), disp(NULL), disp_clone(NULL), sync(this, &RecognitionLambertian::image_cb_all, ros::Duration().fromSec(0.1), &RecognitionLambertian::image_cb_timeout)
    {
        tf_ = new tf::TransformListener(*this);
        // define node parameters


        param("~min_height", min_height, 0.7);
        param("~max_height", max_height, 1.0);
        param("~frames_no", frames_no, 7);


        param("~display", display, false);
        stringstream ss;
        ss << getenv("ROS_ROOT") << "/../ros-pkg/vision/recognition_lambertian/data/";
        string path = ss.str();
        string template_path;
        param<string>("template_path", template_path, path + "template.png");

        edges_low = 50;
        edges_high = 170;

        if(display){
            cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
            cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
//            cvNamedWindow("disparity_original", CV_WINDOW_AUTOSIZE);
        	cvNamedWindow("edges",1);
        	cvCreateTrackbar("edges_low","edges",&edges_low, 500, &on_edges_low);
        	cvCreateTrackbar("edges_high","edges",&edges_high, 500, &on_edges_high);
        }


//        advertise<robot_msgs::PointStamped>("handle_detector/handle_location", 1);
        advertise<visualization_msgs::Marker>("visualization_marker", 1);

        subscribeStereoData();

        cm = new ChamferMatching();

        loadTemplate(template_path);
    }

    ~RecognitionLambertian()
    {
        if(left){
            cvReleaseImage(&left);
        }
        if(right){
            cvReleaseImage(&right);
        }
        if(disp){
            cvReleaseImage(&disp);
        }

        delete cm;

        unsubscribeStereoData();
    }

private:

    void subscribeStereoData()
    {

    	sync.reset();
        std::list<std::string> left_list;
        left_list.push_back(std::string("stereo/left/image_rect_color"));
        left_list.push_back(std::string("stereo/left/image_rect"));
        sync.subscribe(left_list, limage, 1);

        std::list<std::string> right_list;
        right_list.push_back(std::string("stereo/right/image_rect_color"));
        right_list.push_back(std::string("stereo/right/image_rect"));
        sync.subscribe(right_list, rimage, 1);

        sync.subscribe("stereo/disparity", dimage, 1);
//        sync.subscribe("stereo/stereo_info", stinfo, 1);
//        sync.subscribe("stereo/disparity_info", dispinfo, 1);
//        sync.subscribe("stereo/right/cam_info", rcinfo, 1);
        sync.subscribe("stereo/cloud", cloud_fetch, 1);
        sync.ready();
//        sleep(1);
    }

    void unsubscribeStereoData()
    {
        unsubscribe("stereo/left/image_rect_color");
        unsubscribe("stereo/left/image_rect");
        unsubscribe("stereo/right/image_rect_color");
        unsubscribe("stereo/right/image_rect");
        unsubscribe("stereo/disparity");
//        unsubscribe("stereo/stereo_info");
//        unsubscribe("stereo/disparity_info");
//        unsubscribe("stereo/right/cam_info");
        unsubscribe("stereo/cloud");
    }


    void loadTemplate(string path)
    {
    	IplImage* templ = cvLoadImage(path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);

        cm->addTemplateFromImage(templ);

    	cvReleaseImage(&templ);
	}



    void showMatch(IplImage* img, const ChamferMatch& match)
    {
    	const template_coords_t& templ_coords = match.tpl->coords;
    	for (size_t i=0;i<templ_coords.size();++i) {
    		int x = match.offset.x + templ_coords[i].first;
    		int y = match.offset.y + templ_coords[i].second;
    		CV_PIXEL(unsigned char, img,x,y)[1] = 255;
    	}
    }


    /**
     * \brief Finds edges in an image
     * @param img
     */
    void doChamferMatching(IplImage *img)
    {
    	// edge detection
        IplImage *gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
        cvCvtColor(img, gray, CV_RGB2GRAY);
        cvCanny(gray, gray, edges_high/2, edges_high);

        if (display) {
        	cvShowImage("edges", gray);
        }


        ChamferMatch match = cm->matchEdgeImage(gray);

        IplImage* left_clone = cvCloneImage(left);
        showMatch(left,match);
        if(display){
        	// show filtered disparity
        	cvShowImage("disparity", disp);
        	// show left image
        	cvShowImage("left", left);
        	cvShowImage("right", right);
        }
        cvCopy(left_clone, left);
        cvReleaseImage(&left_clone);

        cvReleaseImage(&gray);
    }





    void runRecognitionLambertian()
    {
        // acquire cv_mutex lock
//        boost::unique_lock<boost::mutex> images_lock(cv_mutex);

        // goes to sleep until some images arrive
//        images_ready.wait(images_lock);
//        printf("Woke up, processing images\n");


        // do useful stuff
    	doChamferMatching(left);

    }





    /**
     * \brief Filters a cloud point, retains only points coming from a specific region in the disparity image
     *
     * @param rect Region in disparity image
     * @return Filtered point cloud
     */
    robot_msgs::PointCloud filterPointCloud(const CvRect & rect)
    {
        robot_msgs::PointCloud result;
        result.header.frame_id = cloud.header.frame_id;
        result.header.stamp = cloud.header.stamp;
        int xchan = -1;
        int ychan = -1;
        for(size_t i = 0;i < cloud.chan.size();++i){
            if(cloud.chan[i].name == "x"){
                xchan = i;
            }
            if(cloud.chan[i].name == "y"){
                ychan = i;
            }
        }

        if(xchan != -1 && ychan != -1){
            for(size_t i = 0;i < cloud.pts.size();++i){
                int x = (int)(cloud.chan[xchan].vals[i]);
                int y = (int)(cloud.chan[ychan].vals[i]);
                if(x >= rect.x && x < rect.x + rect.width && y >= rect.y && y < rect.y + rect.height){
                    result.pts.push_back(cloud.pts[i]);
                }
            }

        }

        return result;
    }


    /**
     * Callback from topic synchronizer, timeout
     * @param t
     */
    void image_cb_timeout(ros::Time t)
    {
        if(limage.header.stamp != t) {
            printf("Timed out waiting for left image\n");
        }

        if(dimage.header.stamp != t) {
            printf("Timed out waiting for disparity image\n");
        }

//        if(stinfo.header.stamp != t) {
//            printf("Timed out waiting for stereo info\n");
//        }

        if(cloud_fetch.header.stamp != t) {
        	printf("Timed out waiting for point cloud\n");
        }
    }


    /**
     * Callback from topic synchronizer, images ready to be consumed
     * @param t
     */
    void image_cb_all(ros::Time t)
    {
        // obtain lock on vision data
        boost::lock_guard<boost::mutex> lock(cv_mutex);

        if(lbridge.fromImage(limage, "bgr")){
            if(left != NULL)
                cvReleaseImage(&left);

            left = cvCloneImage(lbridge.toIpl());
        }
        if(rbridge.fromImage(rimage, "bgr")){
            if(right != NULL)
                cvReleaseImage(&right);

            right = cvCloneImage(rbridge.toIpl());
        }
        if(dbridge.fromImage(dimage)){
            if(disp != NULL)
                cvReleaseImage(&disp);

//            disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
            disp = cvCloneImage(dbridge.toIpl());
//            cvCvtScale(dbridge.toIpl(), disp, 4.0 / dispinfo.dpp);
        }

        cloud = cloud_fetch;

//        images_ready.notify_all();
        runRecognitionLambertian();
    }



public:
	/**
	 * Needed for OpenCV event loop, to show images
	 * @return
	 */
	/**
	 * Needed for OpenCV event loop, to show images
	 * @return
	 */
	bool spin()
	{
		while (ok())
		{
			cv_mutex.lock();
			int key = cvWaitKey(3)&0x00FF;
			if(key == 27) //ESC
				break;

			cv_mutex.unlock();
			usleep(10000);
		}

		return true;
	}

	void triggerEdgeDetection()
	{
		doChamferMatching(left);
	}
};

RecognitionLambertian* node;

void on_edges_low(int value)
{
	node->edges_low = value;
	node->triggerEdgeDetection();
}

void on_edges_high(int value)
{
	node->edges_high = value;
	node->triggerEdgeDetection();
}


int main(int argc, char **argv)
{
	for(int i = 0; i<argc; ++i)
		cout << "(" << i << "): " << argv[i] << endl;

	ros::init(argc, argv);
	node = new RecognitionLambertian();
	node->spin();

	delete node;

	return 0;
}

