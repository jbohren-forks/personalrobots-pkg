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

/*
Stereo Lib "2" for Open Stereo package
Federico Tombari
Willow Garage Inc.
email: tombari@willowgarage.com
CVLab - University of Bologna
email: federico.tombari@unibo.it
*/

#include "stereolib.h"
#define inline			// use this for Intel Compiler Debug mode
#include <stdio.h>

inline int min3(int a, int b, int c){
return (a<b) ? ((a<c)?a:c) : ((b<c)?b:c);
}

inline int min4(int a, int b, int c, int d){

int t1 = (a<b)?a:b;
int t2 = (c<d)?c:d;
return (t1<t2)?t1:t2;

}

inline int min(int a, int b){
return (a<b)?a:b;
}

//Application of the uniqueness constraint
void uniqueness_constraint_reducedrange(int w, short int* disp, int maxdisp, int r, int* mins, int row){

int x, d, x_oth, d_oth;

// int w = par.w;
// short int* disp = par.disp;
// int maxdisp = par.maxdisp;
// int r = par.radius;

//Init array
int* unique;
unique = (int *)malloc(w*sizeof(int));
for(x=r; x<w-r; x++)
	unique[x] = -1;
	
for(x=maxdisp+r+1; x<w-r; x++){
	d = disp[row*w + x]/16;

	//UNIQUENESS CONSTRAINT
	if(d > FILTERED){		//this is needed since previous filters may have already discarded this correspondence
		if(unique[x-d] == -1)
			unique[x-d] = d;
		else{
			d_oth = unique[x-d];	
			x_oth = x - d + d_oth;
			if(mins[x_oth] > mins[x]){
 				unique[x-d] = d;
 				disp[row*w + x_oth] = FILTERED;
 			}
			else{
				disp[row*w + x] = FILTERED;
 			}
		}
	}
}//y

free(unique);
}

//Scanline-optimization along horizontal directions
void do_stereo_so(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int16_t *text,	// texture output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  uint8_t *buf		// buffer storage
	  )
{

xwin = xwin-4; //bad hack to deal with small window sizes (e.g. 3x3)

int maxdisp = dlen;
int r = (xwin-1)/2;
int n = xwin;
//note: ywim is currently deprecated

int w = xim;
int h = yim;

unsigned char *L = lim;
unsigned char *R = rim;
	
//Parameters for regularization
int pi2 = tfilter_thresh*4*n;
int pi1 = tfilter_thresh*n;

//temp variables
int x,y,d,i,j;
int dbest=0;
int c_min, cost;

//printf("%d %d\r", w, h);

//FILTERS
//int sqn = n*n;
//Reliability filter is currently deprecated
//int rel_filter = sqn * par.rel_filter;

//Uniqueness-2 filter is currently deprecated
int *min_scores = (int *)malloc(w * sizeof(int ));

int ratio_filter = 100-ufilter_thresh;
//int peak_filter = tfilter_thresh;
int peak_filter = 15;

int sad_second_min;
int da, db, s1, s2, s3;

//data structured for Scanline Optimization
int **F;
int **B;

//int *diff;
F = (int **)calloc(w, sizeof(int *));
B = (int **)calloc(w, sizeof(int *));
//diff = (int *)calloc(w, sizeof(int));

for(x=0; x<w; x++){
	F[x] = (int *)calloc(maxdisp, sizeof(int));
	B[x] = (int *)calloc(maxdisp, sizeof(int));
}

//BEGIN INCREMENTAL COMPUTATION FOR RADIUS >0
int **acc = (int **)calloc(w, sizeof(int *));
int **V = (int **)calloc(w, sizeof(int*));
for(d=0; d<w; d++){
	V[d] = (int *)calloc(maxdisp, sizeof(int)); 
	acc[d] = (int *)calloc(maxdisp, sizeof(int)); 	
}

//FIRST ROW
//STAGE 1 - init acc
for(d=0; d<maxdisp; d++){
	for(i=maxdisp; i<maxdisp+n; i++){
		for(j=0; j<n; j++)
			V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
		//acc[d] += V[i][d];			
	}		
}

//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	for(d=0; d<maxdisp; d++){
		for(j=0; j<n; j++)
			V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
		//acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
	}//d
}//x
unsigned char *lp, *rp, *lpp, *rpp;
int ind1 = r+r+maxdisp;
//END INCREMENTAL COMPUTATION FOR RADIUS >0

for(y=r+1; y<h-r; y++){

	//first position
	for(d=0; d<maxdisp; d++){
		acc[maxdisp+r][d] = 0;
		for(i=maxdisp; i<maxdisp+n; i++){
			V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
			acc[maxdisp+r][d] += V[i][d];			
		}	
	}
		
	//compute ACC for all other positions
	lp = (unsigned char *) L + (y+r)*w + ind1;
	rp =  (unsigned char *) R + (y+r)*w + ind1;
	lpp = (unsigned char *) L + (y-r-1)*w + ind1;
	rpp = (unsigned char *) R + (y-r-1)*w + ind1;
		
	for(x=maxdisp+r+1; x<w-r; x++){
			
		lp++;
		rp++;
		lpp++;
		rpp++;
				
		//int dbestprev = dbest;
		for(d=0; d<maxdisp; d++){
				
			V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
			rp--;
			rpp--;
			acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];
		}//d
		rp += maxdisp;
		rpp += maxdisp;
	}//x to compute ACC
		
	//compute intensity edge (used in the currently adopted modified Potts model) 
	//for(x=maxdisp; x<w; x++)
	//	diff[x] = abs(L[w*y+x] - L[w*y+x-1]);

	//FORWARD
	//Border
	for(d=0; d<maxdisp; d++)
		F[maxdisp+r][d] = acc[maxdisp+r][d];

	for(x=maxdisp+r; x<w-r; x++){

		//pi2 = pi2b;
		//if(diff[x] < Tp)	
		//	pi2 = pi2a;

		c_min = F[x-1][0];
		dbest = 0;
		for(d=1; d<maxdisp; d++)
		if(F[x-1][d] < c_min){
			c_min = F[x-1][d];
			dbest = d;
		}

		F[x][0] =  acc[x][0] - c_min + min3(F[x-1][0], F[x-1][1]+pi1, c_min+pi2);
		for(d=1; d<maxdisp-1; d++){
			F[x][d] = acc[x][d] - c_min + min4(F[x-1][d], F[x-1][d-1]+pi1, F[x-1][d+1]+pi1, c_min+pi2);
		} //d
		F[x][maxdisp-1] = acc[x][maxdisp-1] - c_min + min3(F[x-1][maxdisp-1], F[x-1][maxdisp-2]+pi1, c_min+pi2);
	}//x1

	//BACKWARD
	//Border	
	for(d=0; d<maxdisp; d++)
		B[w-1-r][d] = acc[w-1-r][d];
	
	for(x=w-2-r; x>=maxdisp+r; x--){

		//pi2 = pi2b;
		//if(diff[x+1] < Tp)	
		//	pi2 = pi2a;

		c_min = B[x+1][0];
		dbest = 0;
		for(d=1; d<maxdisp; d++)
		if(B[x+1][d] < c_min){
			c_min = B[x+1][d];
			dbest = d;
		}

 		B[x][0] =  acc[x][0] - c_min + min3(B[x+1][0], B[x+1][1]+pi1, c_min+pi2);
 		for(d=1; d<maxdisp-1; d++){
 			B[x][d] = acc[x][d] - c_min + min4(B[x+1][d], B[x+1][d-1]+pi1, B[x+1][d+1]+pi1, c_min+pi2);
 		} //d
 		B[x][maxdisp-1] = acc[x][maxdisp-1] - c_min + min3(B[x+1][maxdisp-1], B[x+1][maxdisp-2]+pi1, c_min+pi2);
	}//x1

	for(x=maxdisp+r; x<w-r; x++){
		c_min = 32000;

		for(d=0; d<maxdisp; d++){
			cost = F[x][d] + B[x][d];
			if(cost < c_min){
				c_min = cost;
				dbest = d;
			}
		}
		disp[y*w+x] = dbest;	
		
		//******* FILTERS ********
		//1) rel filter
		//if(c_min > rel_filter)
		//	disp[y*w + x] = -2;

		//( 2)needed for uniqueness constraint filter )
		min_scores[x] = c_min;
	
		//3) uniqueness filter
		sad_second_min = 32000;
		for(d=0; d<dbest-1; d++){
			cost = F[x][d] + B[x][d];
			if(cost<sad_second_min){
				sad_second_min = cost;
			}
		}
		for(d=dbest+2; d<maxdisp; d++){
			cost = F[x][d] + B[x][d];
			if(cost<sad_second_min){
				sad_second_min = cost;
			}
		}	
		if( c_min*100  > ratio_filter*sad_second_min)
			disp[y*w + x] = FILTERED;
	
		//4) Peak Filter
		s1 = F[x][dbest-1] + B[x][dbest-1];
		s2 = F[x][dbest] + B[x][dbest];
		s3 = F[x][dbest+1] + B[x][dbest+1];
		da = (dbest>1) ? ( s1 - s2 ) : (s3 - s2);
		db =  (dbest<maxdisp-2) ? (s3 - s2) : (s1 - s2);		
		if(da + db < peak_filter)
			disp[y*w + x] = FILTERED;
		//******* FILTERS ********
	
		//subpixel refinement
		if(disp[y*w + x] != FILTERED){
			double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
			disp[y*w + x] = (int)(0.5 + 16*v);
		}
	}
	//if(par.unique == 1)
		uniqueness_constraint_reducedrange(w, disp, maxdisp, r, min_scores, y);
}

for(x=0; x<w; x++){
	free(acc[x]);
	free(F[x]);
	free(B[x]);
	free(V[x]);
}

free(F);
free(B);
//free(diff);
free(acc);

free(min_scores);

}


//Scanline-optimization along horizontal directions
void do_stereo_mw(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int16_t *text,	// texture output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  uint8_t *buf		// buffer storage
	  )
{

	int maxdisp = dlen;
	int r = (xwin-1)/2;
	int n = xwin;
	//note: ywim is currently deprecated
	
	int w = xim;
	int h = yim;
	
	unsigned char *L = lim;
	unsigned char *R = rim;
	//short int* disp = disp;

	int x,y,i,j,d;
	int sqn = n*n;

	int **acc = (int **)calloc(w, sizeof(int *));
	int **V = (int **)calloc(w, sizeof(int *));

	for(d=0; d<w; d++){
		V[d] = (int *)calloc(maxdisp, sizeof(int)); 
		acc[d] = (int *)calloc(maxdisp, sizeof(int)); 
	}

	int sad_min, dbest=0, temp;
	int sad_max = 255 * sqn;

	//FILTERS
	//int rel_filter = sqn * par.rel_filter;
	//int *min_scores = (int *)malloc(w * sizeof(int ));
	int sad_second_min;
	int da, db;
	int ratio_filter = 100-ufilter_thresh;
	int peak_filter = tfilter_thresh;

	//INIT - FIRST ROW
	//STAGE 1 - init acc
	for(d=0; d<maxdisp; d++){
		for(i=maxdisp; i<maxdisp+n; i++){
			for(j=0; j<n; j++)
				V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
			acc[maxdisp+r][d] += V[i][d];			
		}		
	}

	//STAGE 2: other positions
	for(x=maxdisp+r+1; x<w-r; x++){
		for(d=0; d<maxdisp; d++){
			for(j=0; j<n; j++)
				V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
			acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];
		}//d
	}//x

	unsigned char *lp, *rp, *lpp, *rpp;
	int ind1 = r+r+maxdisp;
	//OTHER ROWS
	for(y=r+1; y<h-r; y++){
		
		//first position
		for(d=0; d<maxdisp; d++){
			acc[maxdisp+r][d] = 0;
			for(i=maxdisp; i<maxdisp+n; i++){
				V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
				acc[maxdisp+r][d] += V[i][d];			
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
				
				//V[x+r][d] = V[x+r][d] + abs( L[(y+r)*w + x+r] - R[ (y+r)*w + x+r-d] ) - abs( L[ (y-r-1)*w+ x+r] - R[ (y-r-1)*w+ x+r-d] ); 
				//V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( L[ (y-r-1)*w+ x+r] - R[ (y-r-1)*w+ x+r-d] ); 
				V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
				
				rp--;
				rpp--;
				
				acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];				
			}//d
			
			rp += maxdisp;
			rpp += maxdisp;
		}
	
		//only right term exists
		for(x=maxdisp+r; x<maxdisp+n; x++){
			sad_min = sad_max;
			for(d=0; d<maxdisp; d++){
				temp = min(acc[x][d],acc[x+r][d]);
				if( temp < sad_min){
					sad_min = temp;
					dbest = d;
				}
			}
			disp[y*w + x] = dbest;
			
			//#ifdef FILTERS
			//min_scores[x] = sad_min;
			// 3) uniqueness filter
			sad_second_min = sad_max;
			for(d=0; d<maxdisp; d++){
				if(d != dbest && acc[x][d]<sad_second_min){
					sad_second_min = acc[x][d];
				}
			}	
			if( sad_min*100  > ratio_filter*sad_second_min)
				disp[y*w + x] = FILTERED;
		
			//4) Peak Filter
			da = (dbest>1) ? ( acc[x][dbest-2] - acc[x][dbest] ) : (acc[x][dbest+2] - acc[x][dbest]);
			db =  (dbest<maxdisp-2) ? (acc[x][dbest+2] - acc[x][dbest]) : (acc[x][dbest-2] - acc[x][dbest]);		
			if(da + db < peak_filter)
				disp[y*w + x] = FILTERED;
			//******* FILTERS ********
			
			//subpixel refinement
			if(disp[y*w + x] != FILTERED){
				int s1 = acc[x][dbest-1];
				int s2 = acc[x][dbest];
				int s3 = acc[x][dbest+1];
				double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
				disp[y*w + x] = (int)(0.5 + 16*v);
			}
		}
		
		//both terms exist 
		for(x=maxdisp+n; x<w-n; x++){
			sad_min = sad_max;
			for(d=0; d<maxdisp; d++){
				temp = min3(acc[x][d], acc[x-r][d], acc[x+r][d]);
				if( temp < sad_min){
					sad_min = temp;
					dbest = d;
				}
			}
			disp[y*w + x] = dbest;
			
			//#ifdef FILTERS
			//******* FILTERS ********
			// No reliability filter is applied
			// 2)needed for uniqueness constraint filter )
			//min_scores[x] = sad_min;
		
			// 3) uniqueness filter
			sad_second_min = sad_max;
			for(d=0; d<maxdisp; d++){
				if(d != dbest && acc[x][d]<sad_second_min){
					sad_second_min = acc[x][d];
				}
			}	
			if( sad_min*100  > ratio_filter*sad_second_min)
				disp[y*w + x] = FILTERED;
		
			//4) Peak Filter
			da = (dbest>1) ? ( acc[x][dbest-2] - acc[x][dbest] ) : (acc[x][dbest+2] - acc[x][dbest]);
			db =  (dbest<maxdisp-2) ? (acc[x][dbest+2] - acc[x][dbest]) : (acc[x][dbest-2] - acc[x][dbest]);		
			if(da + db < peak_filter)
				disp[y*w + x] = FILTERED;
			//******* FILTERS ********
			//#endif
			
			//subpixel refinement
			if(disp[y*w + x] != FILTERED){
				int s1 = acc[x][dbest-1];
				int s2 = acc[x][dbest];
				int s3 = acc[x][dbest+1];
				double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
				disp[y*w + x] = (int)(0.5 + 16*v);
			}
		}//x
		
		//only left term exists
		for(x=w-n; x<w-r; x++){
			sad_min = sad_max;
			for(d=0; d<maxdisp; d++){
				temp = min(acc[x][d],acc[x-r][d]);
				if( temp < sad_min){
					sad_min = temp;
					dbest = d;
				}
			}
			disp[y*w + x] = dbest;
			//min_scores[x] = sad_min;
			// 3) uniqueness filter
			sad_second_min = sad_max;
			for(d=0; d<maxdisp; d++){
				if(d != dbest && acc[x][d]<sad_second_min){
					sad_second_min = acc[x][d];
				}
			}	
			if( sad_min*100  > ratio_filter*sad_second_min)
				disp[y*w + x] = FILTERED;
		
			//4) Peak Filter
			da = (dbest>1) ? ( acc[x][dbest-2] - acc[x][dbest] ) : (acc[x][dbest+2] - acc[x][dbest]);
			db =  (dbest<maxdisp-2) ? (acc[x][dbest+2] - acc[x][dbest]) : (acc[x][dbest-2] - acc[x][dbest]);		
			if(da + db < peak_filter)
				disp[y*w + x] = FILTERED;
			//******* FILTERS ********
		
			//subpixel refinement
			if(disp[y*w + x] != FILTERED){
				int s1 = acc[x][dbest-1];
				int s2 = acc[x][dbest];
				int s3 = acc[x][dbest+1];
				double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
				disp[y*w + x] = (int)(0.5 + 16*v);
			}
		}

		
		
		//if(par.unique == 1)
		//	uniqueness_constraint_reducedrange(par, min_scores, y);
	}//y

	for(d=0; d<w;d++){
		free(V[d]);
		free(acc[d]);
	}
	free(V);
	free(acc);

	//free(min_scores);
}

//Scanline-optimization along horizontal directions
void do_stereo_dp(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int16_t *text,	// texture output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  uint8_t *buf		// buffer storage
	  )
{


xwin = xwin-4; //bad hack to deal with small window sizes (e.g. 3x3)

int maxdisp = dlen;
int r = (xwin-1)/2;
int n = xwin;
//note: ywim is currently deprecated

int w = xim;
int h = yim;

unsigned char *L = lim;
unsigned char *R = rim;
//short int* disp = disp;
	
//Parameters for regularization
int w2 = tfilter_thresh*4;
int w1 = tfilter_thresh;

//temp variables
int x,y,d,i,j;
int dbest=0;
int c_min, cost;

//FILTERS
//int sqn = n*n;
//Reliability filter is currently deprecated
//int rel_filter = sqn * par.rel_filter;

//Uniqueness-2 filter is currently deprecated
//int *min_scores = (int *)malloc(w * sizeof(int ));

int ratio_filter = 100-ufilter_thresh;
//int peak_filter = tfilter_thresh;
int peak_filter = 10;

int sad_second_min;
int da, db, s1, s2, s3;

//data structured for Dynamic Programming
int **S;
int **B;

//int *diff;
S = (int **)calloc(w, sizeof(int *));
B = (int **)calloc(w, sizeof(int *));
//diff = (int *)calloc(w, sizeof(int));

for(x=0; x<w; x++){
	S[x] = (int *)calloc(maxdisp, sizeof(int));
	B[x] = (int *)calloc(maxdisp, sizeof(int));
}

//BEGIN INCREMENTAL COMPUTATION FOR RADIUS >0
int **acc = (int **)calloc(w, sizeof(int *));
int **V = (int **)calloc(w, sizeof(int*));
for(d=0; d<w; d++){
	V[d] = (int *)calloc(maxdisp, sizeof(int)); 
	acc[d] = (int *)calloc(maxdisp, sizeof(int)); 	
}

//FIRST ROW
//STAGE 1 - init acc
for(d=0; d<maxdisp; d++){
	for(i=maxdisp; i<maxdisp+n; i++){
		for(j=0; j<n; j++)
			V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
		//acc[d] += V[i][d];			
	}		
}

//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	for(d=0; d<maxdisp; d++){
		for(j=0; j<n; j++)
			V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
		//acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
	}//d
}//x
unsigned char *lp, *rp, *lpp, *rpp;
int ind1 = r+r+maxdisp;
//END INCREMENTAL COMPUTATION FOR RADIUS >0

for(y=r+1; y<h-r; y++){

	//first position
	for(d=0; d<maxdisp; d++){
		acc[maxdisp+r][d] = 0;
		for(i=maxdisp; i<maxdisp+n; i++){
			V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
			acc[maxdisp+r][d] += V[i][d];			
		}	
	}
		
	//compute ACC for all other positions
	lp = (unsigned char *) L + (y+r)*w + ind1;
	rp =  (unsigned char *) R + (y+r)*w + ind1;
	lpp = (unsigned char *) L + (y-r-1)*w + ind1;
	rpp = (unsigned char *) R + (y-r-1)*w + ind1;
		
	for(x=maxdisp+r+1; x<w-r; x++){
			
		lp++;
		rp++;
		lpp++;
		rpp++;
				
		//int dbestprev = dbest;
		for(d=0; d<maxdisp; d++){	
			V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
			rp--;
			rpp--;
			acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];
		}//d
		rp += maxdisp;
		rpp += maxdisp;
	}//x to compute ACC

	//Stage 1 ->forward
	//Border
	for(d=0; d<maxdisp; d++)
		S[maxdisp+r][d] = acc[maxdisp+r][d];
	
	for(x=maxdisp+r+1; x<w-r; x++){
		c_min = S[x-1][0];
		dbest = 0;
		for(d=1; d<maxdisp; d++)
		if(S[x-1][d] < c_min){
			c_min = S[x-1][d];
			dbest = d;
		}
	
		S[x][0] = acc[x][0];
		if( S[x-1][0]<c_min+w2 ){
 			S[x][0] += S[x-1][0];
  			B[x][0] = d;
		}
		else{
			S[x][0] += c_min+w2;
  			B[x][0] = dbest;
		}
		S[x][maxdisp-1] = acc[x][maxdisp-1];
		if( S[x-1][maxdisp-1]<c_min+w2 ){
 			S[x][maxdisp-1] += S[x-1][maxdisp-1];
  			B[x][maxdisp-1] = d;
		}
		else{
			S[x][maxdisp-1] += c_min+w2;
  			B[x][maxdisp-1] = dbest;
		}

		for(d=1; d<maxdisp-1; d++){
			S[x][d] = acc[x][d];
			if( S[x-1][d]<c_min+w2 ){
 				if( S[x-1][d]<S[x-1][d-1]+w1 ){
 					if( S[x-1][d]<S[x-1][d+1]+w1 ){
   						S[x][d] += S[x-1][d];
   						B[x][d] = d;
 					}
 					else{
 						S[x][d] += S[x-1][d+1]+w1;
 						B[x][d] = d+1;
 					}
 				}
 				else{
 					if(S[x-1][d-1] < S[x-1][d+1]){
 						S[x][d] += S[x-1][d-1]+w1;
 						B[x][d] = d-1;
 					}
 					else{
 						S[x][d] += S[x-1][d+1]+w1;
 						B[x][d] = d+1;
 					}
 				}
 			}
 			else{
				if( c_min+w2 < S[x-1][d-1]+w1 ){
					if( c_min+w2 < S[x-1][d+1]+w1 ){
  						S[x][d] += c_min+w2;
  						B[x][d] = dbest;
					}
					else{
						S[x][d] += S[x-1][d+1]+w1;
						B[x][d] = d+1;
					}
				}
				else{
					if(S[x-1][d-1] < S[x-1][d+1]){
						S[x][d] += S[x-1][d-1]+w1;
						B[x][d] = d-1;
					}
					else{
						S[x][d] += S[x-1][d+1]+w1;
						B[x][d] = d+1;
					}
				}
 			}
 		} //d
 	}//x

	//Stage 2 -> backward
	//Border
	x = w-r-1;
	c_min = 32000;
	for(d=0; d<maxdisp; d++){
		cost = S[x][d];
		if(cost < c_min){
			c_min = cost;
			dbest = d;
		}
	}
	disp[y*w+x] = dbest;
	
	for(x=w-r-2; x>=maxdisp+r; x--){
	//for(x=maxdisp+r+1; x<w-r; x++){
// 		c_min = 32000;
// 		for(d=0; d<maxdisp; d++){
// 			cost = acc[x][d];
// 			if(cost < c_min){
// 				c_min = cost;
// 				dbest = d;
// 			}
// 		}
// 		disp[y*w+x] = dbest*16;

 		dbest = B[x+1][dbest];
 		disp[y*w+x] = dbest;	
		
		//******* FILTERS ********
		//1) rel filter
		//if(c_min > rel_filter)
		//	disp[y*w + x] = -2;

		//( 2)needed for uniqueness constraint filter )
		//min_scores[x] = c_min;
	
		//3) uniqueness filter
		/*
		sad_second_min = 32000;
		for(d=0; d<dbest-1; d++){
			cost = S[x][d];
			if(cost<sad_second_min){
				sad_second_min = cost;
			}
		}
		for(d=dbest+2; d<maxdisp; d++){
			cost = S[x][d];
			if(cost<sad_second_min){
				sad_second_min = cost;
			}
		}	
		if( c_min*100  > ratio_filter*sad_second_min)
			disp[y*w + x] = FILTERED;
		*/
		//4) Peak Filter
		s1 = S[x][dbest-1];
		s2 = S[x][dbest];
		s3 = S[x][dbest+1];
		da = (dbest>1) ? ( s1 - s2 ) : (s3 - s2);
		db =  (dbest<maxdisp-2) ? (s3 - s2) : (s1 - s2);		
		if(da + db < peak_filter)
			disp[y*w + x] = FILTERED;
		//******* FILTERS ********
		
		//subpixel refinement
		if(disp[y*w + x] != FILTERED){
			double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
			disp[y*w + x] = (int)(0.5 + 16*v);
		}
	}
	//if(par.unique == 1)
	//	uniqueness_constraint_reducedrange(par, min_scores, y);
}

for(x=0; x<w; x++){
	free(acc[x]);
	free(S[x]);
	free(B[x]);
	free(V[x]);
}

free(S);
free(B);
//free(diff);
free(acc);

//free(min_scores);

}







