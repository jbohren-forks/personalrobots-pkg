/*********************************************************************
 * meanshift_clustering.cpp
 *
 * Adapted from code by: Piotr Dollara nd Sameer Agarwal.
 *
 * Piotr's Image&Video Toolbox      Version NEW
 * Copyright 2009 Piotr Dollar.  [pdollar-at-caltech.edu]
 * Please email me if you find bugs, or have suggestions or questions!
 * Licensed under the Lesser GPL [see 3rdparty_LGPL/]
 *********************************************************************/
#include <point_cloud_clustering/meanshift_clustering.h>

/*********************************************************************
 * Calculates mean of all the points in data that lie on a sphere of
 * radius^2==radius2 centered on [1xp] vector x. data is [nxp]. mean
 * contains [1xp] result and return is number of points used for calc.
 *********************************************************************/
int meanVec(const robot_msgs::PointCloud& pt_cloud,
            cloud_kdtree::KdTree& pt_cloud_kdtree,
            const double* query_ptr,
            const double radius,
            double* mean)
{
  mean[0] = 0.0;
  mean[1] = 0.0;
  mean[2] = 0.0;

  robot_msgs::Point32 query_pt;
  query_pt.x = query_ptr[0];
  query_pt.y = query_ptr[1];
  query_pt.z = query_ptr[2];

  vector<int> neighbor_indices;
  vector<float> neighbor_distances;
  if (pt_cloud_kdtree.radiusSearch(query_pt, radius, neighbor_indices, neighbor_distances))
  {
    unsigned int nbr_neighbors = neighbor_indices.size();
    for (unsigned int i = 0 ; i < nbr_neighbors ; i++)
    {
      mean[0] += static_cast<double> (pt_cloud.pts[neighbor_indices[i]].x);
      mean[1] += static_cast<double> (pt_cloud.pts[neighbor_indices[i]].y);
      mean[2] += static_cast<double> (pt_cloud.pts[neighbor_indices[i]].z);
    }

    mean[0] /= static_cast<double> (nbr_neighbors);
    mean[1] /= static_cast<double> (nbr_neighbors);
    mean[2] /= static_cast<double> (nbr_neighbors);
  }

  return neighbor_indices.size();
}

/* Squared euclidean distance between two vectors. */
double dist(double *A, double *B, int n)
{
  double d = 0.0;
  int i;
  for (i = 0; i < n ; i++)
    d += (A[i] - B[i]) * (A[i] - B[i]);
  return d;
}

/*********************************************************************
 * data        - p x n column matrix of data points
 * p         - dimension of data points
 * n         - number of data points
 * radius      - radius of search windo
 * rate        - gradient descent proportionality factor
 * maxIter     - max allowed number of iterations
 * blur        - specifies algorithm mode
 * labels      - labels for each cluster
 * means       - output (final clusters)
 *********************************************************************/
void point_cloud_clustering::pcMeanshift(const robot_msgs::PointCloud& pt_cloud,
                                         cloud_kdtree::KdTree& pt_cloud_kdtree,
                                         double radius,
                                         double rate,
                                         int maxIter,
                                         map<unsigned int, vector<int> >& cluster_pt_indices)
{
  cluster_pt_indices.clear();
  int p = 3;
  int n = pt_cloud.pts.size();

  double radius2; /* radius^2 */
  int iter; /* number of iterations */
  double *mean; /* mean vector */
  int i, j, o, m; /* looping and temporary variables */
  int delta = 1; /* indicator if change occurred between iterations */
  int *deltas; /* indicator if change occurred between iterations per point */
  double *meansCur; /* calculated means for current iter */
  double *meansNxt; /* calculated means for next iter */
  int *consolidated; /* Needed in the assignment of cluster labels */
  int nLabels = 1; /* Needed in the assignment of cluster labels */

  /* initialization */
  meansCur = (double*) malloc(sizeof(double) * p * n);
  meansNxt = (double*) malloc(sizeof(double) * p * n);
  mean = (double*) malloc(sizeof(double) * p);
  consolidated = (int*) malloc(sizeof(int) * n);
  deltas = (int*) malloc(sizeof(int) * n);
  for (i = 0; i < n ; i++)
    deltas[i] = 1;
  radius2 = radius * radius;

  double data1[p * n];
  for (int i = 0 ; i < n ; i++)
  {
    data1[i * p + 0] = pt_cloud.pts[i].x;
    data1[i * p + 1] = pt_cloud.pts[i].y;
    data1[i * p + 2] = pt_cloud.pts[i].z;
  }

  meansCur = (double*) memcpy(meansCur, data1, p * n * sizeof(double));

  /* main loop */
  for (iter = 0; iter < maxIter ; iter++)
  {
    delta = 0;
    for (i = 0; i < n ; i++)
    {
      if (i % 1000 == 0)
        ROS_INFO("i %d",i);
      if (deltas[i])
      {
        /* shift meansNxt in direction of mean (if m>0) */
        o = i * p;
        //m = meanVec(meansCur + o, data1, p, n, radius2, mean);
        m = meanVec(pt_cloud, pt_cloud_kdtree, meansCur + o, radius, mean);
        if (m)
        {
          for (j = 0; j < p ; j++)
            meansNxt[o + j] = (1 - rate) * meansCur[o + j] + rate * mean[j];
          if (dist(meansNxt + o, meansCur + o, p) > 0.001) delta = 1;
          else
            deltas[i] = 0;
        }
        else
        {
          for (j = 0; j < p ; j++)
            meansNxt[o + j] = meansCur[o + j];
          deltas[i] = 0;
        }
      }
    }
    memcpy(meansCur, meansNxt, p * n * sizeof(double));
    if (!delta) break;
  }

  /* Consolidate: assign all points that are within radius2 to same cluster. */
  for (i = 0; i < n ; i++)
  {
    consolidated[i] = 0;
    //labels[i] = 0;
  }
  for (i = 0; i < n ; i++)
    if (!consolidated[i])
    {
      cluster_pt_indices[nLabels] = vector<int> ();
      for (j = 0; j < n ; j++)
        if (!consolidated[j])
        {
          if (dist(meansCur + i * p, meansCur + j * p, p) < radius2)
          {
            cluster_pt_indices[nLabels].push_back(j);
            //labels[j] = nLabels;
            consolidated[j] = 1;
          }
        }
      nLabels++;
    }
  nLabels--;

  /* free memory */
  free(meansNxt);
  free(meansCur);
  free(mean);
  free(consolidated);
  free(deltas);
}
