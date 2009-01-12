// This file contains the templated version from mpi_kmeans. The original 
// C-code is available at http://www.mloss.org/software/view/48/ and has been
// written by Peter Gehler from Max Planck Institut (MPI), Germany.
// 
// The underlying algorithm is based on "Using the Triangle Inequality to 
// Accelerate k-Means" by Charles Elkan. This work could not reduce the 
// theoretical complexity but in practice the proportionality constant is
// considerably smaller compared to the fastetst algorithms to date.
//
// Additionally, the k-means++ initialization scheme has been added, details
// are given in "k-means++: The Advantages of Careful Seeding" by Arthur et al.
//
// Dec 2008, calonder@willowgarage.com

#pragma once

#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <cassert>
#include <limits>
#include <algorithm>

namespace features {

template< typename T > struct DistanceFltTTrait { typedef float type; };
template<> struct DistanceFltTTrait<float>  { typedef float type; };
template<> struct DistanceFltTTrait<double> { typedef double type; };

typedef unsigned int uint;

// from original implementation: PREC <-> {PointDataT,DistanceFltT}, BoundFltT <-> BOUND_PREC
template < typename PointDataT >
class FastKMeans
{
public:
   typedef typename DistanceFltTTrait<PointDataT>::type DistanceFltT;
   typedef float BoundFltT;
   typedef enum { IM_RANDOM, IM_USE_GIVEN_CENTROIDS, IM_KMEANSPP } InitMethod;      
   
   FastKMeans();
   
   // default is IM_RANDOM
   void setInitMethod(const InitMethod &m) { init_method_ = m; }

   inline
   double run(PointDataT* features,    // array of pointers to data vectors (num_pts x dim-matrix)
              uint size,               // number of data vectors
              uint dimension,          // dimension of data vectors
              uint k,                  // number of clusters
              uint* membership,        // returns assigned cluster id of each data vector (num_pts-vector)
              PointDataT* centroids,   // cluster centers (k x dim-matrix)
              uint max_it = 100,       // max number of iterations
              uint num_restarts = 1    // number of restarts (>1 makes sense for init_method == IM_RANDOM only)
             );
   
protected:
   void  kmeans_error(char *msg);
   int   comp_randperm(const void * a, const void * b);
   void  randperm(uint *order, uint npoints);
   DistanceFltT  compute_distance(const PointDataT *vec1, const PointDataT *vec2, const uint dim);
   DistanceFltT  compute_sserror(const PointDataT *CX, const PointDataT *X, const uint *c,uint dim, uint npts);
   void  remove_point_from_cluster(uint cluster_ind, PointDataT *CX, const PointDataT *px, uint *nr_points, uint dim);
   void  add_point_to_cluster(uint cluster_ind, PointDataT *CX, const PointDataT *px, uint *nr_points, uint dim);
   bool  remove_identical_clusters(PointDataT *CX, BoundFltT *cluster_distance, const PointDataT *X, uint *cluster_count, uint *c, uint dim, uint nclus, uint npts);
   void  compute_cluster_distances(BoundFltT *dist, BoundFltT *s, const PointDataT *CX, uint dim,uint nclus, const bool *cluster_changed);
   uint  init_point_to_cluster(uint point_ind, const PointDataT *px, const PointDataT *CX, uint dim,uint nclus, DistanceFltT *mindist, BoundFltT *low_b, const BoundFltT *cl_dist);
   uint  assign_point_to_cluster_ordinary(const PointDataT *px, const PointDataT *CX, uint dim,uint nclus);
   uint  assign_point_to_cluster(uint point_ind, const PointDataT *px, const PointDataT *CX, uint dim,uint nclus, uint old_assignment, DistanceFltT *mindist, BoundFltT *s, BoundFltT *cl_dist, BoundFltT *low_b);
   DistanceFltT kmeans_run(PointDataT *CX,const PointDataT *X,uint *c,uint dim,uint npts,uint nclus,uint maxiter);
   DistanceFltT kmeans(PointDataT *CX,PointDataT *X,uint *assignment,uint dim,uint npts,uint nclus,uint maxiter, uint restarts);
   static double rand_double(double high=1., double low=0.);
   static int rand_int(int high=RAND_MAX, int low=0);   
   static double squared_dist(DistanceFltT* a, DistanceFltT* b, int length);

   static const double BOUND_EPS = 1e-6f;
   const double BOUND_PREC_MAX;
   const double PREC_MAX;
   InitMethod init_method_;
};

template < typename PointDataT >
FastKMeans<PointDataT>::FastKMeans() 
  : BOUND_PREC_MAX(std::numeric_limits<BoundFltT>::max()), PREC_MAX(std::numeric_limits<PointDataT>::max()),
    init_method_(IM_RANDOM)
{ }


template < typename PointDataT >
inline
double FastKMeans<PointDataT>::run(PointDataT* features, uint size, uint dimension, uint k, uint* membership, 
                                   PointDataT* centroids, uint max_it, uint num_restarts)
{
   return kmeans(centroids, features, membership, dimension, size, k, max_it, num_restarts);
}


template < typename PointDataT >
void FastKMeans<PointDataT>::kmeans_error(char *msg)
{
	printf("%s",msg);
	exit(-1);
}

static int comp_randperm(const void * a, const void * b)
{
	return ((int)( *(double*)a - *(double*)b ));
}
static int (*pcmp)(const void*,const void*) = comp_randperm;

template < typename PointDataT >
void FastKMeans<PointDataT>::randperm(uint *order, uint npoints)
{
	double *r = (double*)malloc(2*npoints*sizeof(double));
	for (uint i=0; i<2*npoints; i++,i++)
	{
		r[i] = rand();
		r[i+1] = i/2;
	}	
	qsort(r, npoints, 2*sizeof(double), pcmp);

	for (uint i=1; i<2*npoints; i++,i++)
		order[i/2] = (uint)r[i];

	free(r);
}

template < typename PointDataT >
typename FastKMeans<PointDataT>::DistanceFltT FastKMeans<PointDataT>::compute_distance(const PointDataT *vec1, const PointDataT *vec2, const uint dim)
{
	DistanceFltT d = 0.0;
	for ( uint k=0 ; k<dim ; k++ )
	{
		DistanceFltT df = (vec1[k]-vec2[k]);
		d += df*df;
	}
	assert(d>=0.0);
	d = sqrt(d);

	return d;
}

template < typename PointDataT >
typename FastKMeans<PointDataT>::DistanceFltT FastKMeans<PointDataT>::compute_sserror(const PointDataT *CX, const PointDataT *X, const uint *c,uint dim, uint npts)
{
	DistanceFltT sse = 0.0;
	const PointDataT *px = X;
	for ( uint i=0 ; i<npts ; i++,px+=dim)
	{
		const PointDataT *pcx = CX+c[i]*dim;
		DistanceFltT d = compute_distance(px,pcx,dim);
		sse += d*d;
	}
	assert(sse>=0.0);
	return(sse);
}

template < typename PointDataT >
void FastKMeans<PointDataT>::remove_point_from_cluster(uint cluster_ind, PointDataT *CX, const PointDataT *px, uint *nr_points, uint dim)
{
	PointDataT *pcx = CX + cluster_ind*dim;

	/* empty cluster after or before removal */
	if (nr_points[cluster_ind]<2)
	{
		for ( uint k=0 ; k<dim ; k++ )
			pcx[k] = 0.0;
		nr_points[cluster_ind]=0;
	}
	else
	{
		double nr_old,nr_new; 
		nr_old = (double)nr_points[cluster_ind];
		(nr_points[cluster_ind])--;
		nr_new = (double)nr_points[cluster_ind];

		for ( uint k=0 ; k<dim ; k++ )
			pcx[k] = (PointDataT)((nr_old*pcx[k] - px[k])/nr_new);
	}
}

template < typename PointDataT >
void FastKMeans<PointDataT>::add_point_to_cluster(uint cluster_ind, PointDataT *CX, const PointDataT *px, uint *nr_points, uint dim)
{

	PointDataT *pcx = CX + cluster_ind*dim;

	/* first point in cluster */
	if (nr_points[cluster_ind]==0)
	{		
		(nr_points[cluster_ind])++;
		for ( uint k=0 ; k<dim ; k++ )
			pcx[k] = px[k];
	}
	else
	{
		double nr_old = (double)(nr_points[cluster_ind]);
		(nr_points[cluster_ind])++;
		double nr_new = (double)(nr_points[cluster_ind]);
		for ( uint k=0 ; k<dim ; k++ )
			pcx[k] = (PointDataT)((nr_old*pcx[k]+px[k])/nr_new);
	}
}

template < typename PointDataT >
bool FastKMeans<PointDataT>::remove_identical_clusters(PointDataT *CX, BoundFltT *cluster_distance, const PointDataT *X, uint *cluster_count, uint *c, uint dim, uint nclus, uint npts)
{
	bool stat = false;
	for ( uint i=0 ; i<(nclus-1) ; i++ )
	{
		for ( uint j=i+1 ; j<nclus ; j++ )
		{
			if (cluster_distance[i*nclus+j] <= BOUND_EPS)
			{
				stat = true;
				/* assign the points from j to i */
				const PointDataT *px = X;
				for ( uint n=0 ; n<npts ; n++,px+=dim )
				{
					if (c[n] != j) continue;
					remove_point_from_cluster(c[n],CX,px,cluster_count,dim);
					c[n] = i;
					add_point_to_cluster(c[i],CX,px,cluster_count,dim);
				}
			}
		}
	}
	return(stat);
}

template < typename PointDataT >
void FastKMeans<PointDataT>::compute_cluster_distances(BoundFltT *dist, BoundFltT *s, const PointDataT *CX, uint dim,uint nclus, const bool *cluster_changed)
{
	for ( uint j=0 ; j<nclus ; j++ )
		s[j] = BOUND_PREC_MAX;

	const PointDataT *pcx = CX;
	for ( uint i=0 ; i<nclus-1 ; i++,pcx+=dim)
	{
		const PointDataT *pcxp = CX + (i+1)*dim;
		uint cnt=i*nclus+i+1;
		for ( uint j=i+1 ; j<nclus; j++,cnt++,pcxp+=dim )
		{
			if (cluster_changed[i] || cluster_changed[j])
			{
				dist[cnt] = (BoundFltT)(0.5 * compute_distance(pcx,pcxp,dim));
				dist[j*nclus+i] = dist[cnt];

				if (dist[cnt] < s[i])
					s[i] = dist[cnt];

				if (dist[cnt] < s[j])
					s[j] = dist[cnt];
			}
		}
	}
}

template < typename PointDataT >
uint FastKMeans<PointDataT>::init_point_to_cluster(uint point_ind, const PointDataT *px, const PointDataT *CX, uint dim,uint nclus, DistanceFltT *mindist, BoundFltT *low_b, const BoundFltT *cl_dist)
{
	bool use_low_b = true;

	if (low_b==NULL) use_low_b = false;
	uint bias = point_ind*nclus;
	
	const PointDataT *pcx = CX;
	DistanceFltT mind = compute_distance(px,pcx,dim);
	if (use_low_b) low_b[bias] = (BoundFltT)mind;
	uint assignment = 0;
	pcx+=dim;
	for ( uint j=1 ; j<nclus ; j++,pcx+=dim )
	{
		if (mind + BOUND_EPS <= cl_dist[assignment*nclus+j])
			continue;

		DistanceFltT d = compute_distance(px,pcx,dim);
		if(use_low_b) low_b[j+bias] = (BoundFltT)d;

		if (d<mind)
		{
			mind = d;
			assignment = j;
		}
	}
	mindist[point_ind] = mind;
	return(assignment);
}

template < typename PointDataT >
uint FastKMeans<PointDataT>::assign_point_to_cluster_ordinary(const PointDataT *px, const PointDataT *CX, uint dim,uint nclus)
{
	uint assignment = nclus;
	DistanceFltT mind = PREC_MAX;
	const PointDataT *pcx = CX;
	for ( uint j=0 ; j<nclus ; j++,pcx+=dim )
	{
		DistanceFltT d = compute_distance(px,pcx,dim);
		if (d<mind)
		{
			mind = d;
			assignment = j;
		}
	}
	assert(assignment < nclus);
	return(assignment);
}

template < typename PointDataT >
uint FastKMeans<PointDataT>::assign_point_to_cluster(uint point_ind, const PointDataT *px, const PointDataT *CX, uint dim,uint nclus, uint old_assignment, DistanceFltT *mindist, BoundFltT *s, BoundFltT *cl_dist, BoundFltT *low_b)
{
	bool up_to_date = false,use_low_b=true;;

	uint bias = point_ind*nclus;
	if (low_b==NULL)use_low_b=false;

	DistanceFltT mind = mindist[point_ind];

	if (mind+BOUND_EPS <= s[old_assignment])
	{
#ifdef KMEANS_VEBOSE
		saved_two++;
#endif
		return(old_assignment);
	}

	uint assignment = old_assignment;
	uint counter = assignment*nclus;
	const PointDataT *pcx = CX;
	for ( uint j=0 ; j<nclus ; j++,pcx+=dim )
	{
		if (j==old_assignment)
		{
			continue;
		}
		
		if (use_low_b && (mind+BOUND_EPS <= low_b[j+bias]))
		{
			continue;
		}

		if (mind+BOUND_EPS <= cl_dist[counter+j])
		{
			continue;
		}

		DistanceFltT d = 0.0;
		if (!up_to_date)
		{
			d = compute_distance(px,CX+assignment*dim,dim);
			mind = d;
			if(use_low_b) low_b[assignment+bias] = (BoundFltT)d;
			up_to_date = true;
		}
		
		if (!use_low_b)
			d = compute_distance(px,pcx,dim);
		else if ((mind > BOUND_EPS+low_b[j+bias]) || (mind > BOUND_EPS+cl_dist[counter+j]))
		{
			d =compute_distance(px,pcx,dim);
			low_b[j+bias] = (BoundFltT)d;
		}
		else
		{
			continue;
		}

		if (d<mind)
		{
			mind = d;
			assignment = j;
			counter = assignment*nclus;
			up_to_date = true;
		}
	}
	mindist[point_ind] = mind;

	return(assignment);
}

template < typename PointDataT >
typename FastKMeans<PointDataT>::DistanceFltT FastKMeans<PointDataT>::kmeans_run(PointDataT *CX,const PointDataT *X,uint *c,uint dim,uint npts,uint nclus,uint maxiter)
{
	PointDataT *tCX = (PointDataT *)calloc(nclus * dim, sizeof(PointDataT));
	if (tCX==NULL)	kmeans_error((char*)"Failed to allocate mem for Cluster points");

	/* number of points per cluster */
	uint *CN = (uint *) calloc(nclus, sizeof(uint)); 
	if (CX==NULL)	kmeans_error((char*)"Failed to allocate mem for assignment");
	
	/* old assignement of points to cluster */
	uint *old_c = (uint *) malloc(npts* sizeof(uint));
	if (old_c==NULL)	kmeans_error((char*)"Failed to allocate mem for temp assignment");

	/* assign to value which is out of range */
	for ( uint i=0 ; i<npts ; i++)
		old_c[i] = nclus;

	BoundFltT *low_b = (BoundFltT *) calloc(npts*nclus,sizeof(BoundFltT));
	bool use_low_b = false;
	if (low_b == NULL)
	{
		use_low_b = false;
	}
	else
	  {
	    use_low_b = true;
	    assert(low_b);
	  }


	BoundFltT *cl_dist = (BoundFltT *)calloc(nclus*nclus, sizeof(BoundFltT));
	if (cl_dist==NULL)	kmeans_error((char*)"Failed to allocate mem for cluster-cluster distance");

	BoundFltT *s = (BoundFltT *) malloc(nclus*sizeof(BoundFltT));
	if (s==NULL)	kmeans_error((char*)"Failed to allocate mem for assignment");

	BoundFltT *offset = (BoundFltT *) malloc(nclus * sizeof(BoundFltT)); /* change in distance of a cluster mean after a iteration */
	if (offset==NULL)	kmeans_error((char*)"Failed to allocate mem for bound points-nearest cluster");

	DistanceFltT *mindist = (DistanceFltT *)malloc(npts * sizeof(DistanceFltT));
	if (mindist==NULL)	kmeans_error((char*)"Failed to allocate mem for bound points-clusters");

	for ( uint i=0;i<npts;i++)
		mindist[i] = PREC_MAX;

	bool *cluster_changed = (bool *) malloc(nclus * sizeof(bool)); /* did the cluster changed? */
	if (cluster_changed==NULL)	kmeans_error((char*)"Failed to allocate mem for variable cluster_changed");
	for ( uint j=0 ; j<nclus ; j++ )
		cluster_changed[j] = true;


	uint iteration = 0;
	uint nchanged = 1;
	while (iteration < maxiter || maxiter == 0)
	{
		
		/* compute cluster-cluster distances */
		compute_cluster_distances(cl_dist, s, CX, dim,nclus, cluster_changed);
		
		/* assign all points from identical clusters to the first occurence of that cluster */
		remove_identical_clusters(CX, cl_dist, X, CN, c, dim, nclus, npts);
			
		/* find nearest cluster center */
		if (iteration == 0)
		{
		  
		  const PointDataT *px = X;
		  for ( uint i=0 ; i<npts ; i++,px+=dim)
			{
				c[i] = init_point_to_cluster(i,px,CX,dim,nclus,mindist,low_b,cl_dist);
				add_point_to_cluster(c[i],tCX,px,CN,dim);
			}
			nchanged = npts;
		}
		else
		{
			for ( uint j=0 ; j<nclus ; j++)
				cluster_changed[j] = false;

			nchanged = 0;
			const PointDataT *px = X;
			for ( uint i=0 ; i<npts ; i++,px+=dim)
			{
				c[i] = assign_point_to_cluster(i,px,CX,dim,nclus,old_c[i],mindist,s,cl_dist,low_b);

				if (old_c[i] == c[i]) continue;
				
				nchanged++;

				cluster_changed[c[i]] = true;
				cluster_changed[old_c[i]] = true;

				remove_point_from_cluster(old_c[i],tCX,px,CN,dim);
				add_point_to_cluster(c[i],tCX,px,CN,dim);
			}

		}


		/* fill up empty clusters */
		for ( uint j=0 ; j<nclus ; j++)
		{
			if (CN[j]>0) continue;
			uint *rperm = (uint*)malloc(npts*sizeof(uint));
			if (cluster_changed==NULL)	kmeans_error((char*)"Failed to allocate mem for permutation");

			randperm(rperm,npts);
			uint i = 0; 
			while (rperm[i]<npts && CN[c[rperm[i]]]<2) i++;
			if (i==npts)continue;
			i = rperm[i];

			cluster_changed[c[rperm[i]]] = true;
			cluster_changed[j] = true;
			const PointDataT *px = X + i*dim;
			remove_point_from_cluster(c[i],tCX,px,CN,dim);
			c[i] = j;
			add_point_to_cluster(j,tCX,px,CN,dim);
			/* void the bounds */
			s[j] = (BoundFltT)0.0;
			mindist[i] = 0.0;
			if (use_low_b)
				for ( uint k=0 ; k<npts ; k++ )
					low_b[k*nclus+j] = (BoundFltT)0.0;
			
			nchanged++;
			free(rperm);
		}

		/* no assignment changed: done */
		if (nchanged==0) break; 

		/* compute the offset */

		PointDataT *pcx = CX;
		PointDataT *tpcx = tCX;
		for ( uint j=0 ; j<nclus ; j++,pcx+=dim,tpcx+=dim )
		{
			offset[j] = (BoundFltT)0.0;
			if (cluster_changed[j])
			{
				offset[j] = (BoundFltT)compute_distance(pcx,tpcx,dim);
				memcpy(pcx,tpcx,dim*sizeof(PointDataT));
			}
		}
		
		/* update the lower bound */
		if (use_low_b)
		{
			for ( uint i=0,cnt=0 ; i<npts ; i++ )
				for ( uint j=0 ; j<nclus ; j++,cnt++ )
				{
					low_b[cnt] -= offset[j];
					if (low_b[cnt]<(BoundFltT)0.0) low_b[cnt] = (BoundFltT)0.0;
				}
		}

		for ( uint i=0; i<npts; i++)
			mindist[i] += (DistanceFltT)offset[c[i]];

		memcpy(old_c,c,npts*sizeof(uint));

		iteration++;

	}

	/* find nearest cluster center if iteration reached maxiter */
	if (nchanged>0)
	{
	    const PointDataT *px = X;
		for ( uint i=0 ; i<npts ; i++,px+=dim)
			c[i] = assign_point_to_cluster_ordinary(px,CX,dim,nclus);
	}
	DistanceFltT sse = compute_sserror(CX,X,c,dim,npts);

	if(low_b) free(low_b);
	free(cluster_changed);
	free(mindist);
	free(s);
	free(offset);
	free(cl_dist);
	free(tCX);
	free(CN);
	free(old_c);

	return(sse);
}


template < typename PointDataT >
typename FastKMeans<PointDataT>::DistanceFltT FastKMeans<PointDataT>::kmeans(PointDataT *CX,PointDataT *X,uint *assignment,uint dim,uint npts,uint nclus,uint maxiter, uint restarts)
{

  if (npts < nclus)
    {
      CX = (PointDataT*)calloc(nclus*dim,sizeof(PointDataT));
      memcpy(CX,X,dim*nclus*sizeof(PointDataT));
      DistanceFltT sse = 0.0;
      return(sse);
    }
  else if (npts == nclus)
    {
      memcpy(CX,X,dim*nclus*sizeof(PointDataT));
      DistanceFltT sse = 0.0;
      return(sse);
    }
  else if (nclus == 0)
    {
	   printf("Error: Number of clusters is 0\n");
	   exit(-1);
    }

  // initialization
  assert(CX != NULL);
  if (init_method_ == IM_RANDOM)
  {
    uint *order = (uint*)malloc(npts*sizeof(uint));

    //CX = (PointDataT*)calloc(nclus*dim,sizeof(PointDataT));
    /* generate new starting point */
    randperm(order,npts);
    for (uint i=0; i<nclus; i++)
    for ( uint k=0; k<dim; k++ )
	   CX[(i*dim)+k] = X[order[i]*dim+k];
    free(order);		
  }
  else if (init_method_ == IM_KMEANSPP) {
   /**
   * Chooses the initial centers in the k-means as proposed by
   * "Arthur, David; Vassilvitskii, Sergei: k-means++: The Advantages of 
   * Careful Seeding"
   *   
   * Params:
   *     k = number of centers 
   *     vecs = the dataset of points
   *     indices = indices in the dataset
   * Returns:
   */
   //void chooseCentersKMeanspp(int k, Dataset<float>& vecs, int* indices, int indices_length, float** centers, int& centers_length)
   // map:
   // k -> nclus
   // vecs -> X
   // vecs.rows -> npts
   // vecs.cols -> dim
   // indices -> (ignored)
   // indices_length -> (ignored)
   // centers -> CX
   // centers_length -> (ignored)
                 
       double currentPot = 0;
       double* closestDistSq = new double[npts];

       // Choose one random center and set the closestDistSq values
       int index = rand_int(npts);  
       assert(index >=0 && index < (int)npts);
       //centers[0] = vecs[index];
       memcpy(&CX[0], &X[index*dim], dim*sizeof(float));
       
       for (int i = 0; i < (int)npts; i++) {
           closestDistSq[i] = squared_dist(&X[i*dim], &X[index*dim], dim);
           currentPot += closestDistSq[i];
       }

       const int numLocalTries = 1;

       // Choose each center
       for (uint centerCount = 1; centerCount < nclus; centerCount++) {

           // Repeat several trials
           double bestNewPot = -1;
           int bestNewIndex; 
           for (int localTrial = 0; localTrial < numLocalTries; localTrial++) {
           
               // Choose our center - have to be slightly careful to return a valid answer even accounting
               // for possible rounding errors
           double randVal = rand_double(currentPot);
               for (index = 0; index < (int)npts-1; index++) {
                   if (randVal <= closestDistSq[index])
                       break;
                   else
                       randVal -= closestDistSq[index];
               }

               // Compute the new potential
               double newPot = 0;
               for (int i = 0; i < (int)npts; i++)
                   newPot += std::min( squared_dist(&X[i*dim], &X[index*dim], dim), closestDistSq[i] );

               // Store the best result
               if (bestNewPot < 0 || newPot < bestNewPot) {
                   bestNewPot = newPot;
                   bestNewIndex = index;
               }
           }

           // Add the appropriate center
           //centers[centerCount] = vecs[bestNewIndex];
           memcpy(&CX[centerCount*dim], &X[bestNewIndex*dim], dim*sizeof(float));	            
           currentPot = bestNewPot;
           for (int i = 0; i < (int)npts; i++)
               closestDistSq[i] = std::min( squared_dist(&X[i*dim], &X[bestNewIndex*dim], dim), closestDistSq[i] );
       }

	   delete[] closestDistSq;

  }
  else if (init_method_ == IM_USE_GIVEN_CENTROIDS) {
    // yep, nothing
  }

  else {
    printf("%s:%i: ERROR - unknown init method.\n", __FILE__, __LINE__);
    exit(1);
  }
    
  DistanceFltT sse = kmeans_run(CX,X,assignment,dim,npts,nclus,maxiter);

  uint res = restarts;
  if (res>0)
  {
      DistanceFltT minsse = sse;
      uint *order = (uint*)malloc(npts*sizeof(uint));
      PointDataT *bestCX = (PointDataT*) malloc(dim*nclus*sizeof(PointDataT));
      uint *bestassignment = (uint*)malloc(npts*sizeof(uint));

      memcpy(bestCX,CX,dim*nclus*sizeof(PointDataT));
      memcpy(bestassignment,assignment,npts*sizeof(uint));

      while (res>0)
	  {

		  /* generate new starting point */
		  randperm(order,npts);
		  for (uint i=0; i<nclus; i++)
			  for (uint k=0; k<dim; k++ )
				  CX[(i*dim)+k] = X[order[i]*dim+k];
		
		  sse = kmeans_run(CX,X,assignment,dim,npts,nclus,maxiter);
		  if (sse<minsse)
		  {
			  minsse = sse;
			  memcpy(bestCX,CX,dim*nclus*sizeof(PointDataT));
			  memcpy(bestassignment,assignment,npts*sizeof(uint));
		  }
		  res--;

	  }
      memcpy(CX,bestCX,dim*nclus*sizeof(PointDataT));
      memcpy(assignment,bestassignment,npts*sizeof(uint));
      sse = minsse;
      free(bestassignment);
      free(bestCX);
      free(order);
  }
  
  return(sse);
}

template < typename PointDataT >
double FastKMeans<PointDataT>::rand_double(double high, double low)
{
    return low + ((high-low) * (std::rand() / (RAND_MAX + 1.0)));
}

template < typename PointDataT >
int FastKMeans<PointDataT>::rand_int(int high, int low)
{
    return low + (int) ( double(high-low) * (std::rand() / (RAND_MAX + 1.0)));    
}


/**
 *  Compute the squared distance between two vectors. 
 *
 *	This is highly optimized, with loop unrolling, as it is one
 *	of the most expensive inner loops.
 */
template < typename PointDataT >
double FastKMeans<PointDataT>::squared_dist(DistanceFltT* a, DistanceFltT* b, int length) 
{
	double distsq = 0.0;
	double diff0, diff1, diff2, diff3;
	DistanceFltT* v1 = a;
	DistanceFltT* v2 = b;
	
	DistanceFltT* final_ = v1 + length;
	DistanceFltT* finalgroup = final_ - 3;

	/* Process 4 items with each loop for efficiency. */
	while (v1 < finalgroup) {
		diff0 = v1[0] - v2[0];	
		diff1 = v1[1] - v2[1];
		diff2 = v1[2] - v2[2];
		diff3 = v1[3] - v2[3];
		distsq += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
		v1 += 4;
		v2 += 4;
	}
	/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
	while (v1 < final_) {
		diff0 = *v1++ - *v2++;
		distsq += diff0 * diff0;
	}
	return distsq;
}
                
} // namespace

