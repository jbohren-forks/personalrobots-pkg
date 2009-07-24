/*********************************************************************
 * meanShift1.c
 *
 * See meanShift.m. Adapted from code by: Sameer Agarwal.
 *
 * Piotr's Image&Video Toolbox      Version NEW
 * Copyright 2009 Piotr Dollar.  [pdollar-at-caltech.edu]
 * Please email me if you find bugs, or have suggestions or questions!
 * Licensed under the Lesser GPL [see external/lgpl.txt]
 *********************************************************************/
#include <image_segmentation/meanshift.h>

/*********************************************************************
 * Calculates mean of all the points in data that lie on a sphere of
 * radius^2==radius2 centered on [1xp] vector x. data is [nxp]. mean
 * contains [1xp] result and return is number of points used for calc.
 *********************************************************************/
int meanVec(double *x, double *data, int p, int n, double radius2, double *mean)
{
  int i, j;
  double dist;
  int cnt = 0, m = 0;
  for (j = 0; j < p ; j++)
    mean[j] = 0;
  for (i = 0; i < n ; i++)
  {
    dist = 0.0;
    for (j = 0; j < p ; j++)
    {
      dist += (x[j] - data[cnt]) * (x[j] - data[cnt]);
      cnt++;
    }
    if (dist < radius2)
    {
      cnt -= p;
      m++;
      for (j = 0; j < p ; j++)
        mean[j] += data[cnt++];
    }
  }
  if (m) for (j = 0; j < p ; j++)
    mean[j] /= m;
  return m;
}

int meanVec(ANNkd_tree& ann_kd_tree, ANNpointArray& data, double* query_ptr, double radius, double* mean)
{
  // p = dimension
  int p = ann_kd_tree.theDim();
  memset(mean, 0, sizeof(double) * p);

  ANNpoint query = annAllocPt(p);
  for (int i = 0 ; i < p ; i++)
  {
    query[i] = query_ptr[i];
  }

  // compute number of points within radius
  double epsilon = 1e-5;
  int neighbors_in_radius = ann_kd_tree.annkFRSearch(query, radius, 0, NULL, NULL, epsilon);
  ANNidxArray k_indices = new ANNidx[neighbors_in_radius];
  ANNdistArray k_distances = new ANNdist[neighbors_in_radius];

  // calculate mean of points in radius
  if (neighbors_in_radius != 0)
  {
    //ann_kd_tree.annkFRSearch(query, radius, neighbors_in_radius, &k_indices[0], &k_distances[0], epsilon);
    ann_kd_tree.annkFRSearch(query, radius, neighbors_in_radius, k_indices, k_distances, epsilon);

    // accumulate sum
    for (int i = 0 ; i < neighbors_in_radius ; i++)
    {
      for (int j = 0 ; j < p ; j++)
      {
        mean[j] += data[k_indices[i]][j];
      }
    }

    // normalize
    for (int i = 0 ; i < p ; i++)
    {
      mean[i] /= static_cast<double> (neighbors_in_radius);
    }
  }

  delete[] k_indices;
  delete[] k_distances;
  annDeallocPt(query);

  return neighbors_in_radius;
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
 * data				- p x n column matrix of data points
 * p					- dimension of data points
 * n					- number of data points
 * radius			- radius of search windo
 * rate				- gradient descent proportionality factor
 * maxIter			- max allowed number of iterations
 * blur				- specifies algorithm mode
 * labels			- labels for each cluster
 * means				- output (final clusters)
 *********************************************************************/

void meanshift::meanShift(double *data,
                          int p,
                          int n,
                          double radius,
                          double rate,
                          int maxIter,
                          bool blur,
                          double *labels,
                          double *means)
{
  // use blur = false
  // ------- kd tree ---------
  ANNpointArray ann_data;
  ann_data = annAllocPts(n, p);
  for (int dimension_idx = 0 ; dimension_idx < p ; dimension_idx++)
  {
    for (int sample_idx = 0 ; sample_idx < n ; sample_idx++)
    {
      ann_data[sample_idx][dimension_idx] = data[dimension_idx * n + sample_idx];
    }
  }
  int bucket_size = std::min(30, n); // default bucket size value
  ANNkd_tree ann_kd_tree(ann_data, n, p, bucket_size);

  // ------- kd tree ---------

  double radius2; /* radius^2 */
  int iter; /* number of iterations */
  double *mean; /* mean vector */
  int i, j, o, m; /* looping and temporary variables */
  int delta = 1; /* indicator if change occurred between iterations */
  int *deltas; /* indicator if change occurred between iterations per point */
  double *meansCur; /* calculated means for current iter */
  double *meansNxt; /* calculated means for next iter */
  double *data1; /* If blur data1 points to meansCur else it points to data */
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
  meansCur = (double*) memcpy(meansCur, data, p * n * sizeof(double));
  if (blur) data1 = meansCur;
  else
    data1 = data;

  /* main loop */
  for (iter = 0; iter < maxIter ; iter++)
  {
    delta = 0;
    for (i = 0; i < n ; i++)
    {
      if (deltas[i])
      {
        /* shift meansNxt in direction of mean (if m>0) */
        o = i * p;
        //m = meanVec(meansCur + o, data1, p, n, radius2, mean);
        m = meanVec(ann_kd_tree, ann_data, meansCur + o, radius, mean);
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
    std::cout << "iteration: " << iter << " / " << maxIter << std::endl;
    memcpy(meansCur, meansNxt, p * n * sizeof(double));
    if (!delta) break;
  }

  /* Consolidate: assign all points that are within radius2 to same cluster. */
  for (i = 0; i < n ; i++)
  {
    consolidated[i] = 0;
    labels[i] = 0;
  }
  for (i = 0; i < n ; i++)
    if (!consolidated[i])
    {
      for (j = 0; j < n ; j++)
        if (!consolidated[j])
        {
          if (dist(meansCur + i * p, meansCur + j * p, p) < radius2)
          {
            labels[j] = nLabels;
            consolidated[j] = 1;
          }
        }
      nLabels++;
    }
  nLabels--;
  memcpy(means, meansCur, p * n * sizeof(double));

  /* free memory */
  free(meansNxt);
  free(meansCur);
  free(mean);
  free(consolidated);
  free(deltas);
}
