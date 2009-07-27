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
 * radius centered on [1xp] vector query_ptr. mean
 * contains [1xp] result and return is number of points used for calc.
 *********************************************************************/
int meanVec(const robot_msgs::PointCloud& pt_cloud, cloud_kdtree::KdTree& pt_cloud_kdtree, const set<
    unsigned int>& indices_to_cluster, const double* query_ptr, const double radius, double* mean)
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

  pt_cloud_kdtree.radiusSearch(query_pt, radius, neighbor_indices, neighbor_distances);
  unsigned int nbr_neighbors = neighbor_indices.size();
  unsigned int nbr_valid = nbr_neighbors;
  for (unsigned int i = 0 ; i < nbr_neighbors ; i++)
  {
    if (indices_to_cluster.count(neighbor_indices[i]) == 0)
    {
      nbr_valid--;
      continue;
    }

    mean[0] += static_cast<double> (pt_cloud.pts[neighbor_indices[i]].x);
    mean[1] += static_cast<double> (pt_cloud.pts[neighbor_indices[i]].y);
    mean[2] += static_cast<double> (pt_cloud.pts[neighbor_indices[i]].z);
  }

  if (nbr_valid > 0)
  {
    mean[0] /= static_cast<double> (nbr_neighbors);
    mean[1] /= static_cast<double> (nbr_neighbors);
    mean[2] /= static_cast<double> (nbr_neighbors);
  }

  return nbr_valid;
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
int point_cloud_clustering::MeanShift::cluster(const robot_msgs::PointCloud& pt_cloud,
                                               cloud_kdtree::KdTree& pt_cloud_kdtree,
                                               const set<unsigned int>& indices_to_cluster,
                                               map<unsigned int, vector<int> >& created_clusters,
                                               map<unsigned int, vector<float> >* cluster_centroids)
{
  double bandwidth2; /* bandwidth_^2 */
  unsigned int iter; /* number of iterations */
  double *mean; /* mean vector */
  int i, j, o, m; /* looping and temporary variables */
  int delta = 1; /* indicator if change occurred between iterations */
  int *deltas; /* indicator if change occurred between iterations per point */
  double *meansCur; /* calculated means for current iter */
  double *meansNxt; /* calculated means for next iter */
  int *consolidated; /* Needed in the assignment of cluster labels */
  int nLabels = 1; /* Needed in the assignment of cluster labels */

  int p = 3; // xyz dimension
  int n = indices_to_cluster.size();
  if (n == 0)
  {
    return -1;
  }

  double data1[p * n];
  unsigned int sample_idx = 0;
  set<unsigned int>::const_iterator iter_indices_to_cluster;
  for (iter_indices_to_cluster = indices_to_cluster.begin(); iter_indices_to_cluster
      != indices_to_cluster.end() ; iter_indices_to_cluster++)
  {
    unsigned int curr_pt_index = *iter_indices_to_cluster;
    if (curr_pt_index > pt_cloud.pts.size())
    {
      return -1;
    }
    data1[sample_idx * 3 + 0] = pt_cloud.pts[curr_pt_index].x;
    data1[sample_idx * 3 + 1] = pt_cloud.pts[curr_pt_index].y;
    data1[sample_idx * 3 + 2] = pt_cloud.pts[curr_pt_index].z;

    sample_idx++;
  }

  /* initialization */
  meansCur = (double*) malloc(sizeof(double) * p * n);
  meansNxt = (double*) malloc(sizeof(double) * p * n);
  mean = (double*) malloc(sizeof(double) * p);
  consolidated = (int*) malloc(sizeof(int) * n);
  deltas = (int*) malloc(sizeof(int) * n);
  for (i = 0; i < n ; i++)
    deltas[i] = 1;
  bandwidth2 = bandwidth_ * bandwidth_;

  meansCur = (double*) memcpy(meansCur, data1, p * n * sizeof(double));

  /* main loop */
  for (iter = 0; iter < max_iter_ ; iter++)
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
        m = meanVec(pt_cloud, pt_cloud_kdtree, indices_to_cluster, meansCur + o, bandwidth_, mean);
        if (m)
        {
          for (j = 0; j < p ; j++)
            meansNxt[o + j] = (1 - rate_) * meansCur[o + j] + rate_ * mean[j];
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
  created_clusters.clear();
  for (i = 0; i < n ; i++)
  {
    consolidated[i] = 0;
    //labels[i] = 0;
  }
  for (i = 0; i < n ; i++)
    if (!consolidated[i])
    {
      created_clusters[nLabels] = vector<int> ();

      j = 0;
      for (iter_indices_to_cluster = indices_to_cluster.begin(); iter_indices_to_cluster
          != indices_to_cluster.end() ; iter_indices_to_cluster++)
      {
        if (!consolidated[j])
        {
          if (dist(meansCur + i * p, meansCur + j * p, p) < bandwidth2)
          {
            created_clusters[nLabels].push_back(*iter_indices_to_cluster);
            //labels[j] = nLabels;
            consolidated[j] = 1;
          }
        }

        j++;
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
  return 0;
}
