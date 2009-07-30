/*
 * cornercandidate.cpp
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#include "cornercandidate.h"

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

double CornerCandidate::computeDistance(IplImage* distImage)
{
  updatePoints3d();
  updatePoints2d();

  bool invalidPoints = false;
  for (int i = 0; i < 4; i++)
  {
    if (points2d[i].x < 0)
      invalidPoints = true;
    if (points2d[i].y < 0)
      invalidPoints = true;
    if (points2d[i].x >= distImage->width)
      invalidPoints = true;
    if (points2d[i].y >= distImage->height)
      invalidPoints = true;
  }
  if (invalidPoints)
    return DBL_MAX;

  int global_sum = 0;
  int global_count = 0;
  for (int i = 0; i < 4; i++)
  {
    CvLineIterator iterator;

    int sum = 0;
    int count = cvInitLineIterator(distImage, points2d[i], points2d[(i + 1) % 4], &iterator, 8, 0);
    for (int j = 0; j < count; j++)
    {
      sum += iterator.ptr[0];
      CV_NEXT_LINE_POINT(iterator);
    }
    global_sum += sum;
    global_count += count;
  }
  //  cout <<"global_sum="<<global_sum<<" global_count="<<global_count<<endl;
  return (global_sum / (double)global_count);
}

void CornerCandidate::optimizeWidth(IplImage* distImage, double a, double b, int steps)
{
  double best_d = DBL_MAX;
  double best_w = w;
  double d;
  double start = w;
  for (int i = 0; i < steps; i++)
  {
    w = MIN(rect_max_size,MAX( rect_min_size,start + a + ((b-a)*(i))/(double)steps));
    d = computeDistance(distImage);
    //    cout << "i="<<i<<" steps="<<steps<< " w="<<w<<" d="<<d << endl;
    if (d < best_d)
    {
      best_d = d;
      best_w = w;
    }
  }
  //  cout << "found w="<< best_w << endl;

  w = best_w;
}

void CornerCandidate::optimizeHeight(IplImage* distImage, double a, double b, int steps)
{
  double best_d = DBL_MAX;
  double best_h = h;
  double d;
  double start = h;
  for (int i = 0; i < steps; i++)
  {
    h = MIN(rect_max_size,MAX( rect_min_size,start + a + ((b-a)*(i))/(double)steps));
    d = computeDistance(distImage);
    if (d < best_d)
    {
      best_d = d;
      best_h = h;
    }
  }
  h = best_h;
}

void CornerCandidate::optimizePhi(IplImage* distImage, double a, double b, int steps)
{
  double best_d = DBL_MAX;
  int best_i = 0;
  double d;

  btTransform org_tf = tf;

  for (int i = 0; i < steps; i++)
  {
    tf = org_tf * btTransform(btQuaternion(btVector3(0, 0, 1), a + ((b - a) * (i)) / (double)steps));
    d = computeDistance(distImage);
    if (d < best_d)
    {
      best_d = d;
      best_i = i;
    }
  }
  tf = org_tf * btTransform(btQuaternion(btVector3(0, 0, 1), a + ((b - a) * (best_i)) / (double)steps));
}

void CornerCandidate::optimizeX(IplImage* distImage, double a, double b, int steps)
{
  double best_d = DBL_MAX;
  int best_i = 0;
  double d;

  btTransform org_tf = tf;

  for (int i = 0; i < steps; i++)
  {
    tf = org_tf * btTransform(btQuaternion::getIdentity(), btVector3(a + ((b - a) * (i)) / (double)steps, 0, 0));
    d = computeDistance(distImage);
    //    cout << "i="<<i<<" steps="<<steps<< " x+="<<a + (((b-a)*(i))/(double)steps)<<" d="<<d
    //    <<  " x="<<tf.getOrigin().x()
    //<<  " y="<<tf.getOrigin().y()
    //<<  " z="<<tf.getOrigin().z()
    //    <<endl;
    if (d < best_d)
    {
      best_d = d;
      best_i = i;
    }
  }
  tf = org_tf * btTransform(btQuaternion::getIdentity(), btVector3(a + ((b - a) * (best_i)) / (double)steps, 0, 0));
}

void CornerCandidate::optimizeY(IplImage* distImage, double a, double b, int steps)
{
  double best_d = DBL_MAX;
  int best_i = 0;
  double d;

  btTransform org_tf = tf;

  for (int i = 0; i < steps; i++)
  {
    tf = org_tf * btTransform(btQuaternion::getIdentity(), btVector3(0, a + ((b - a) * (i)) / (double)steps, 0));
    d = computeDistance(distImage);
    if (d < best_d)
    {
      best_d = d;
      best_i = i;
    }
  }
  tf = org_tf * btTransform(btQuaternion::getIdentity(), btVector3(0, a + ((b - a) * (best_i)) / (double)steps, 0));
}

double CornerCandidate::computeSupport2d(IplImage* pixOccupied, IplImage* pixDebug)
{
  std::map<int, std::map<int, bool> > rect;

  for (int i = 0; i < 4; i++)
  {
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
        ((uchar*)pixDebug->imageData)[y * pixDebug->widthStep + x * pixDebug->nChannels] = MIN(255,((uchar*)pixDebug->imageData)[y * 
pixDebug->widthStep 
+ x * 
pixDebug->nChannels] + 64);
    }
    count += y2 - y1 + 1;
  }
  return (sum / (double)count);
}

double CornerCandidate::computeSupport3d(const robot_msgs::PointCloud& cloud, std::vector<int> & plane_indices)
{
  int count = MAX(1,plane_indices.size());
  int sum = 0;

  btTransform tf_inv = tf.inverse();
  for (size_t i = 0; i < plane_indices.size(); i++)
  {
    robot_msgs::Point32 point = cloud.pts[plane_indices[i]];
    btVector3 vec = tf_inv * btVector3(point.x, point.y, point.z);
    if (vec.x() >= 0 && vec.x() <= w && vec.y() >= 0 && vec.y() <= h)
    {
      sum++;
    }
  }
  //cout << "sum="<<sum<<" count="<<count<<endl;
  return (sum / (double)count);
}

}
