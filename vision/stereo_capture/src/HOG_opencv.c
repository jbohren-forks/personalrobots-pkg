#include <cv.h>
#include <highgui.h>
#include <stdio.h>

typedef struct HOG_param{
  int cell_n;
  int cell_x,cell_y;
  int ori_bin;
  float d_bin_ori;
  int stride_x,stride_y;
}HOG_param;


void init_param(HOG_param *p)
{
  p->cell_n=3;
  p->cell_x=p->cell_y=6;
  p->ori_bin=9;
  p->d_bin_ori=360./(float)p->ori_bin;
  p->stride_x=p->stride_y=6;
}

void  make_theta_mod(IplImage *grey,IplImage *theta,IplImage * rho)
{

  IplImage *dx=cvCreateImage(cvSize(theta->width,theta->height),IPL_DEPTH_16S,1);
  IplImage *dy=cvCreateImage(cvSize(theta->width,theta->height),IPL_DEPTH_16S,1);
  cvSobel(grey,dx,1,0,1);
  cvSobel(grey,dy,0,1,1);


  short *val_dx;
  short *val_dy;

  float *p;

  float ang,mod;
  int x,y;
  for(x=0; x<dx->width;x++)
    for(y=0; y<dy->height;y++)
    {
      val_dx=(short *)(dx->imageData + y*dx->widthStep) + x;
      val_dy=(short *)(dy->imageData + y*dy->widthStep) + x;

      ang=cvFastArctan((float)*val_dy,(float)*val_dx);
      mod=sqrt((*val_dx)*(*val_dx)+(*val_dy)*(*val_dy));

      p=(float *)(theta->imageData+y*theta->widthStep)+x;
      *p=ang;
/*       printf("%i %i %f %f\n",x,y,ang,mod); */
//       cvSetReal2D(theta,y,x,ang);

      p=(float *)(rho->imageData+y*rho->widthStep)+x;
      *p=mod;

//       cvSetReal2D(rho,y,x,mod);
    }

  cvReleaseImage(&dx);
  cvReleaseImage(&dy);
}

void norm_L2(float *v,int n,float e)
{
  float sum=0;
  int i;
  for(i=0; i<n;i++)
    sum+=v[i]*v[i];


  sum=sqrt(sum+e*e);
  for(i=0; i<n;i++)
    v[i]/=sum;
}


void block_histogram(int x0,int y0,IplImage *theta,IplImage * rho,float *block_h,HOG_param *p)
{
  int N=(p->cell_n*p->cell_n*p->ori_bin);
  int i;
  for(i=0; i<N;i++)
    block_h[i]=0;

  float *val;
  int x,y;
  //Building 3-d histogram, in a vector 
  for(x=x0; x<x0+(p->cell_n*p->cell_x);x++)
    for(y=y0; y<y0+(p->cell_n*p->cell_y);y++)
      {
	val=(float *)(theta->imageData+y*theta->widthStep)+x;
	float th=*val;
// 	float th=cvGetReal2D(theta,y,x);
	val=(float *)(rho->imageData+y*rho->widthStep)+x;
	float rh=*val;
// 	float rh=cvGetReal2D(rho,y,x);

	int binx=cvFloor((float)(x-x0)/(float)p->cell_x);
	int biny=cvFloor((float)(y-y0)/(float)p->cell_y);
	int bino=cvFloor(th/p->d_bin_ori);


	int idx=binx*p->cell_n*p->ori_bin+biny*p->ori_bin+bino;
	block_h[idx]+=rh;

	//Smoothing the histogram

/* 	float rbinx= (((float)(x-x0))/(float)p->cell_x)-binx; */
/* 	float rbiny= (((float)(y-y0))/(float)p->cell_y)-biny; */
/* 	float rbino= (th/p->d_bin_ori)-bino; */


/* 	//This is a nice way to obtain the trilinear interpolation */
/*         // for the histogram.  */
/* 	int dbinx,dbiny,dbino; */
/* 	for(dbinx=0; dbinx<2;dbinx++) */
/* 	  for(dbiny=0; dbiny<2;dbiny++) */
/* 	    for(dbino=0; dbino<2;dbino++) */
/* 	      { */
/* 		if((binx+dbinx)<p->cell_n  && (biny+dbiny)<p->cell_n) */
/* 		  { */
/* 		    float r_s=rh*fabs(1-dbinx-rbinx)*fabs(1-dbiny-rbiny)*fabs(1-dbino-rbino); */

/* 		    int idx=(binx+dbinx)*p->cell_n*p->ori_bin+(biny+dbiny)*p->ori_bin+(bino+dbino)%p->ori_bin; */
/* 		    //Warp around on the orientation */

/* 		    block_h[idx]+=r_s; */
/* 		  } */
/* 	      } */
      }

  norm_L2(block_h,N,0.001);
}

int  build_HOG_feature(IplImage *grey,float *HOG_f,HOG_param *param)
{
  CvSize size_g;
  if(grey->roi!=NULL)
    size_g=cvSize(grey->roi->width,grey->roi->height);
  else
    size_g=cvSize(grey->width,grey->height);
  
  int block_x=param->cell_n*param->cell_x;
  int block_y=param->cell_n*param->cell_y;

  if(grey->nChannels==3 || (size_g.width-block_x)<=0 || (size_g.height-block_y)<=0)
    return -1;

  IplImage *theta=cvCreateImage(size_g,IPL_DEPTH_32F,1);
  IplImage *rho=cvCreateImage(size_g,IPL_DEPTH_32F,1);
 
  make_theta_mod(grey,theta,rho);

  int N=(param->cell_n*param->cell_n*param->ori_bin);
  float *block_h=(float *)malloc(N*sizeof(float));

  int num=0,x,y,i;
  for(x=0; x<=(size_g.width-block_x); x+=param->stride_x)
  for(y=0; y<=(size_g.height-block_y); y+=param->stride_y)
    {
      block_histogram(x,y,theta,rho,block_h,param);

      for(i=0;i<N;i++)
	HOG_f[i+N*num]=block_h[i];
      num++;
    }


  free (block_h);
  cvReleaseImage(&theta);
  cvReleaseImage(&rho);
  return 1;
}


int main(int argc, char** argv)
{
  if(argc<=1)
    return -1;

  IplImage *img=cvLoadImage(argv[1],CV_LOAD_IMAGE_ANYCOLOR);

  if(img==NULL)
    {
      fprintf(stderr,"Error loading image %s\n",argv[1]);
      return -2;
    }
  IplImage *grey=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);

  if(img->nChannels==3)
    cvCvtColor(img,grey,CV_RGB2GRAY);
  else
    cvCopy(img,grey,NULL);

  HOG_param *param=(HOG_param *)malloc(sizeof(HOG_param));
  init_param(param);

  int block_size=(param->cell_n*param->cell_n*param->ori_bin);

  CvRect r=cvRect(260,197,30,30);

  int block_x=param->cell_n*param->cell_x;
  int block_y=param->cell_n*param->cell_x;
  int n_block_x=cvFloor((r.width-block_x)/param->stride_x)+1; 
  int n_block_y=cvFloor((r.height-block_y)/param->stride_y)+1;
 
  int feature_size=n_block_x*n_block_y*(param->cell_n*param->cell_n*param->ori_bin);

  printf("Feature size %i\n",feature_size);

  float *HOG=(float *)malloc(feature_size*sizeof(float));


  cvSetImageROI(grey,r);
  if(build_HOG_feature(grey,HOG,param)>0)
    {
      int i;
      for(i=0;i<feature_size;i++)
	printf("%i %g\n",i+1,HOG[i]);
    }

  cvResetImageROI(grey);

  free(param);
  free(HOG);
  cvReleaseImage(&img);
  cvReleaseImage(&grey);
}
