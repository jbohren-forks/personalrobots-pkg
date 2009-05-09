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

#include <queue>
#include <algorithm>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


#include <recognition_lambertian/chamfer_matching.h>



#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)




/**
 * Finds a point in the image from which to start contour following.
 * @param templ_img
 * @param p
 * @return
 */
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
 * @param templ_
 * @param coords
 * @param crt
 */
void followContour(IplImage* templ_img, template_coords_t& coords, int direction = -1)
{
	const int dir[][2] = { {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1}, {1,0}, {1,-1}, {0,-1} };
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

/**
 * Finds a contour in an edge image. The original image is altered by removing the found contour.
 * @param templ_img Edge image
 * @param coords Coordinates forming the contour.
 * @return True while a contour is still found in the image.
 */
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

/**
 * Computes the angle of a line segment.
 *
 * @param a One end of te line segment
 * @param b The other end.
 * @param dx
 * @param dy
 * @return Angle in radians.
 */

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



/**
* Computes contour points orientations using the approach from:
*
* Matas, Shao and Kittler - Estimation of Curvature and Tangent Direction by
* Median Filtered Differencing
*
* @param coords Contour points
* @param orientations Contour points orientations
*/
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



ChamferTemplate::ChamferTemplate(IplImage* edge_image)
{
	size = cvGetSize(edge_image);

	template_coords_t local_coords;
	template_orientations_t local_orientations;

	while (findContour(edge_image, local_coords)) {
		findContourOrientations(local_coords, local_orientations);

		coords.insert(coords.end(), local_coords.begin(), local_coords.end());
		orientations.insert(orientations.end(), local_orientations.begin(), local_orientations.end());
		local_coords.clear();
		local_orientations.clear();
	}

}


/**
 * Resizes a template
 *
 * @param scale Scale to be resized to
 */
void ChamferTemplate::rescale(float scale)
{
	if (scale==1) return;

	size.width = int(size.width*scale+0.5);
	size.height = int(size.height*scale+0.5);

	for (size_t i=0;i<coords.size();++i) {
		coords[i].first = int(coords[i].first*scale+0.5);
		coords[i].second = int(coords[i].second*scale+0.5);
	}

}



void ChamferTemplate::show() const
{
	IplImage* templ_color = cvCreateImage(size, IPL_DEPTH_8U, 3);

	for (size_t i=0;i<coords.size();++i) {

		int x = coords[i].first;
		int y = coords[i].second;
		CV_PIXEL(unsigned char, templ_color,x,y)[1] = 255;

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

	cvReleaseImage(&templ_color);
//		cvWaitKey(0);
}


// ----------------- ChamferMatching ---------------------


/**
 * Add a template to the detector from an edge image.
 * @param templ An edge image
 */
void ChamferMatching::addTemplateFromImage(IplImage* templ)
{
	ChamferTemplate* cmt = new ChamferTemplate(templ);

	for(int i = 0; i < count_scale; ++i) {
		float scale = min_scale + (max_scale - min_scale)*i/count_scale;

		ChamferTemplate* cmt_resized = new ChamferTemplate(*cmt);
		cmt_resized->rescale(scale);

		templates.push_back(cmt_resized);
		cmt_resized->show();

	}

	cmt->show();
}

/**
 * Run matching usin an edge image.
 * @param edge_img Edge image
 * @return a match object
 */
ChamferMatch ChamferMatching::matchEdgeImage(IplImage* edge_img)
{
	ChamferMatch cm;

	IplImage* dist_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32F, 1);
	IplImage* orientation_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32F, 1);
	IplImage* annotated_img = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_32S, 2);

	// Computing distance transform
	IplImage* edge_clone = cvCloneImage(edge_img);
	computeEdgeOrientations(edge_clone, orientation_img );
	cvReleaseImage(&edge_clone);
	computeDistanceTransform(edge_img,dist_img, annotated_img, orientation_img, truncate);
	fillNonContourOrientations(annotated_img, orientation_img);

	// Template matching
	matchTemplates(dist_img, orientation_img, cm);

	cvReleaseImage(&dist_img);
	cvReleaseImage(&orientation_img);
	cvReleaseImage(&annotated_img);

	return cm;
}

/**
 * Run matching using a regular image.
 *
 * Will run Canny edge detection and then the edge matching.
 * @param img
 * @param high_threshold The high edge threshold to use in Canny edge detector.
 * @param low_threshold The low edge threshold to use in Canny edge detector. If default (-1) is used it's computed as high_threshold/2.
 * @return a match object
 */
ChamferMatch ChamferMatching::matchImage(IplImage* img, int high_threshold, int low_threshold)
{
	if (low_threshold==-1) {
		low_threshold = high_threshold/2;
	}
	IplImage *edge_img = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	cvCvtColor(img, edge_img, CV_RGB2GRAY);
	cvCanny(edge_img, edge_img, low_threshold, high_threshold);

	ChamferMatch cm = matchEdgeImage(edge_img);

	cvReleaseImage(&edge_img);
	return cm;
}



/**
 * Alternative version of computeDistanceTransform, will probably be used to compute distance
 * transform annotated with edge orientation.
 */
void ChamferMatching::computeDistanceTransform(IplImage* edges_img, IplImage* dist_img, IplImage* annotate_img, IplImage* orientation_img, float truncate, float a, float b)
{
	int d[][2] = { {-1,-1}, { 0,-1}, { 1,-1},
				  {-1,0},          { 1,0},
				  {-1,1}, { 0,1}, { 1,1} };


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


void ChamferMatching::computeEdgeOrientations(IplImage* edge_img, IplImage* orientation_img)
{
	IplImage* img_color = cvCreateImage(cvSize(edge_img->width, edge_img->height), IPL_DEPTH_8U, 3);
	cvCvtColor(edge_img, img_color, CV_GRAY2RGB);

	template_coords_t coords;
	template_orientations_t orientations;

	while (findContour(edge_img, coords)) {
		findContourOrientations(coords, orientations);

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


void ChamferMatching::fillNonContourOrientations(IplImage* annotated_img, IplImage* orientation_img)
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


static float orientation_diff(float o1, float o2)
{
	return fabs(o1-o2);
}

float ChamferMatching::localChamferDistance(IplImage* dist_img, IplImage* orientation_img, const vector<int>& templ_addr, const vector<float>& templ_orientations, CvPoint offset, float alpha)
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



void ChamferMatching::matchTemplate(IplImage* dist_img, IplImage* orientation_img, const ChamferTemplate& tpl, ChamferMatch& cm)
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
				CvPoint offset;
				offset.x = x;
				offset.y = y;
				float cost = localChamferDistance(dist_img, orientation_img, templ_addr, tpl.orientations, offset);

				cm.addMatch(cost, offset, tpl);

		}
	}

}


void ChamferMatching::matchTemplates(IplImage* dist_img, IplImage* orientation_img, ChamferMatch& cm)
{
	   for(size_t i = 0; i < templates.size(); i++) {
		   matchTemplate(dist_img, orientation_img, *templates[i],cm);
	   }
}



void ChamferMatch::addMatch(float cost, CvPoint offset, const ChamferTemplate& tpl)
{
	if (cost<0.5) {
		ChamferMatchInstance cmi;
		cmi.cost = cost;
		cmi.offset = offset;
		cmi.tpl = &tpl;
		matches.push_back(cmi);
	}
}


template<typename T>
class CostComparator
{
public:
	bool operator()(const T& a, const T& b)
	{
		return a.cost<b.cost;
	}
};


struct MatchCenter {
	int x;
	int y;
	int count;
};


//void ChamferMatch::getMatches()
//{
//	vector<MatchCenter> centers;
//
//	for (size_t i=0;i<matches.size();++i) {
//		for (size_t j=0;j<matches.size();++j) {
//
//		}
//	}
//}

void ChamferMatch::show(IplImage* img)
{
	CostComparator<ChamferMatchInstance> less;
	sort(matches.begin(), matches.end(), less);

	for (size_t i=0;i<2;++i) {
		ChamferMatchInstance match = matches[i];
		const template_coords_t& templ_coords = match.tpl->coords;
		for (size_t i=0;i<templ_coords.size();++i) {
			int x = match.offset.x + templ_coords[i].first;
			int y = match.offset.y + templ_coords[i].second;
			CV_PIXEL(unsigned char, img,x,y)[1] = 255;
		}
	}
}
