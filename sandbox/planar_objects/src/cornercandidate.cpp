/*
 * cornercandidate.cpp
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#include "cornercandidate.h"

using namespace std;

namespace planar_objects {

void CornerCandidate::updatePoints3d()
{
  points3d[0] = tf * btVector3(0, 0, 0);
  points3d[1] = tf * btVector3(w, 0, 0);
  points3d[2] = tf * btVector3(w, h, 0);
  points3d[3] = tf * btVector3(0, h, 0);

  //  for(int i=0;i<4;i++) {
  //    cout << "X="<<points3d[i].x()<<" "<< "Y="<<points3d[i].y()<<" "<< "Z="<<points3d[i].z()<<" "<<endl;
  //  }
}

void CornerCandidate::updatePoints2d()
{
  for (int i = 0; i < 4; i++)
  {
    points2d[i].x = (P[0] * points3d[i].x() + P[1] * points3d[i].y() + P[2] * points3d[i].z() + P[3]) / (P[8]
        * points3d[i].x() + P[9] * points3d[i].y() + P[10] * points3d[i].z() + P[11]);
    points2d[i].y = (P[4] * points3d[i].x() + P[5] * points3d[i].y() + P[6] * points3d[i].z() + P[7]) / (P[8]
        * points3d[i].x() + P[9] * points3d[i].y() + P[10] * points3d[i].z() + P[11]);

  }
}

void CornerCandidate::optimizePhi(IplImage* distImage, double a, double b, int steps)
{
  double best_d = DBL_MAX;
  int best_i = 0;
  double d;

  btTransform org_tf = tf;
  btTransform wh = btTransform(btQuaternion::getIdentity(), btVector3(w/2, h/2, 0));
  btTransform wh_inv = wh.inverse();
  for (int i = 0; i < steps; i++)
  {
    tf = org_tf * wh*btTransform(btQuaternion(btVector3(0, 0, 1), a + ((b - a) * (i)) / (double)steps))*wh_inv;
    d = computeDistance2(distImage);
//    cout << "delta_phi="<<((a + ((b - a) * (i)) / (double)steps)/M_PI*180.0)<<" d="<<d<< endl;
    if (d < best_d)
    {
      best_d = d;
      best_i = i;
    }
  }
//  cout << "best delta_phi="<<((a + ((b - a) * (best_i)) / (double)steps)/M_PI*180.0)<< endl;
  tf = org_tf * wh*btTransform(btQuaternion(btVector3(0, 0, 1), a + ((b - a) * (best_i)) / (double)steps))*wh_inv;
}

double CornerCandidate::computeSupport2d(IplImage* pixOccupied, IplImage* pixDebug)
{
  std::map<int, std::map<int, bool> > rect;

  for (int i = 0; i < 4; i++)
  {
	if(points2d[i].x <0 || points2d[i].x >=pixOccupied->width
		||points2d[i].y <0 || points2d[i].y >=pixOccupied->height)
		return(0.00);

	CvLineIterator iterator;

    int count = cvInitLineIterator(pixOccupied, points2d[i], points2d[(i + 1) % 4], &iterator, 8, 0);
    for (int j = 0; j < count; j++)
    {
      int y = (iterator.ptr - (uchar*)pixOccupied->imageData) / pixOccupied->widthStep;
      int x = ((iterator.ptr - (uchar*)pixOccupied->imageData) % pixOccupied->widthStep) / pixOccupied->nChannels;
      rect[x][y] = true;
      CV_NEXT_LINE_POINT(iterator);
    }
    if (pixDebug != NULL)
    cvLine( pixDebug, points2d[i],points2d[(i + 1) % 4], CV_RGB(0,255,0), 1, 8);
  }

  int count = 0;
  int sum = 0;
  std::map<int, std::map<int, bool> >::iterator i;
  for (i = rect.begin(); i != rect.end(); i++)
  {
    int x = i->first;
    int y1 = i->second.begin()->first;
    int y2 = i->second.rbegin()->first;
    for (int y = y1; y <= y2; y++)
    {
      if (pixOccupied->imageData[y * pixOccupied->widthStep + x * pixOccupied->nChannels] != 0)
        sum++;
      if (pixDebug != NULL)
        ((uchar*)pixDebug->imageData)[y * pixDebug->widthStep + x * pixDebug->nChannels+1] =
            MIN(255,((uchar*)pixDebug->imageData)[y *
pixDebug->widthStep
+ x *
pixDebug->nChannels+1] + 64);
    }
    count += y2 - y1 + 1;
  }
  return (sum / (double)count);
}

double CornerCandidate::computeSupport3d(const sensor_msgs::PointCloud& cloud, std::vector<int> & plane_indices)
{
  int count = MAX(1,plane_indices.size());
  int sum = 0;

  btTransform tf_inv = tf.inverse();
  for (size_t i = 0; i < plane_indices.size(); i++)
  {
    geometry_msgs::Point32 point = cloud.points[plane_indices[i]];
    btVector3 vec = tf_inv * btVector3(point.x, point.y, point.z);
    if (vec.x() >= 0 && vec.x() <= w && vec.y() >= 0 && vec.y() <= h)
    {
      sum++;
    }
  }
  //cout << "sum="<<sum<<" count="<<count<<endl;
  return (sum / (double)count);
}


void CornerCandidate::optimizeWidth2(IplImage* distImage, double a, double b, int steps,bool other)
{
  double best_d = DBL_MAX;
  double best_w = w;
  double d;
  double start = w;
  btTransform org_tf = tf;
  btTransform best_tf = tf;
//  cout << "starting w="<< best_w << endl;
  for (int i = 0; i < steps; i++)
  {
    w = MIN(rect_max_size,MAX( rect_min_size,start + a + ((b-a)*(i))/(double)steps));

    if(other)
      tf = org_tf * btTransform(btQuaternion::getIdentity(), btVector3( start - w, 0, 0));


    d = computeDistance2(distImage);
//    cout << "i="<<i<<" steps="<<steps<< " w="<<w<<" d="<<d << endl;
    if (d < best_d)
    {
      best_d = d;
      best_w = w;
      best_tf = tf;
    }
//    for(int j=0;j<4;j++)
//      cout << "["<<points2d[j].x <<","<<points2d[j].y<<"] ";
//    cout << endl;
  }
//  cout << "found w="<< best_w << endl;

  w = best_w;
  tf = best_tf;
}


void CornerCandidate::optimizeHeight2(IplImage* distImage, double a, double b, int steps,bool other)
{
  double best_d = DBL_MAX;
  double best_h = h;
  double d;
  double start = h;
  btTransform org_tf = tf;
  btTransform best_tf = tf;
  for (int i = 0; i < steps; i++)
  {
    h = MIN(rect_max_size,MAX( rect_min_size,start + a + ((b-a)*(i))/(double)steps));
    if(other)
      tf = org_tf * btTransform(btQuaternion::getIdentity(), btVector3(0,start-h, 0));
    d = computeDistance2(distImage);
    if (d < best_d)
    {
      best_d = d;
      best_h = h;
      best_tf = tf;
    }
  }
  h = best_h;
  tf = best_tf;
}

double CornerCandidate::computeDistance2(IplImage* pixOccupied)
{
  updatePoints3d();
  updatePoints2d();
  std::map<int, std::map<int, bool> > rect;

  for (int i = 0; i < 4; i++)
  {
	if(points2d[i].x <0 || points2d[i].x >=pixOccupied->width
			||points2d[i].y <0 || points2d[i].y >=pixOccupied->height)
		return(DBL_MAX);

    CvLineIterator iterator;

    int count = cvInitLineIterator(pixOccupied, points2d[i], points2d[(i + 1) % 4], &iterator, 8, 0);
    for (int j = 0; j < count; j++)
    {
      int y = (iterator.ptr - (uchar*)pixOccupied->imageData) / pixOccupied->widthStep;
      int x = ((iterator.ptr - (uchar*)pixOccupied->imageData) % pixOccupied->widthStep) / pixOccupied->nChannels;
      rect[x][y] = true;
      CV_NEXT_LINE_POINT(iterator);
    }
  }

  double count = 0;
  double sum = 0;
  std::map<int, std::map<int, bool> >::iterator i;
  for (i = rect.begin(); i != rect.end(); i++)
  {
    int x = i->first;
    int y1 = i->second.begin()->first;
    int y2 = i->second.rbegin()->first;
    for (int y = y1; y <= y2; y++)
    {
//      if (pixOccupied->imageData[y * pixOccupied->widthStep + x * pixOccupied->nChannels] != 0)
//        sum++;
    	sum = sum + ((uchar*)pixOccupied->imageData)[y * pixOccupied->widthStep + x * pixOccupied->nChannels]/255.0;
    }
    count = count + y2 - y1 + 1;
  }
//  cout << "sum = "<<sum<<" count="<<count<<endl;
  return -pow((double)sum,1.2)/(double)count;
}


}
