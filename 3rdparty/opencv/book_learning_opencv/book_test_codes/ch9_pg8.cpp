//
//  added basic image reading parts
//  page 8 to 12
//
#include "AvgBackground.h"

  //Global storage
  IplImage *IavgF,*IdiffF, *IprevF, *IhiF, *IlowF,\
           *Iscratch,*Iscratch2; //Float, 3 channel images
  IplImage *Igray1,*Igray2,*Igray3, *Ilow1,*Ilow2,*Ilow3,\
           *Ihi1,*Ihi2,*Ihi3; //Float, 1 channel images
  IplImage *Imaskt; //Byte, 1 channel image
  float Icount; //Counts number of images learned for averaging later.

  //I is just a sample image for allocation purposes
  void AllocateImages(IplImage *I){ //I is passed in for sizing
  IavgF = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 3 );
  IdiffF = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 3 );
  IprevF = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 3 );
  IhiF = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 3 );
  IlowF = cvCreateImage(cvGetSize(I), IPL_DEPTH_32F, 3 );
  Ilow1 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Ilow2 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Ilow3 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Ihi1 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Ihi2 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Ihi3 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  cvZero(IavgF  );
  cvZero(IdiffF  );
  cvZero(IprevF  );
  cvZero(IhiF );
  cvZero(IlowF  );      
  Icount = 0.00001; //Protect against divide by zero
  
  Iscratch = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 3 );
  Iscratch2 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 3 );
  Igray1 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Igray2 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Igray3 = cvCreateImage( cvGetSize(I), IPL_DEPTH_32F, 1 );
  Imaskt = cvCreateImage( cvGetSize(I), IPL_DEPTH_8U, 1 );
  cvZero(Iscratch);
  cvZero(Iscratch2 );}

// Learn the background statistics for one more frame
// I is a color sample of the background, 3 channel, 8u
void accumulateBackground(IplImage *I){
   static int first = 1;
   cvCvtScale(I,Iscratch,1,0); //To float;
   if (!first){
      cvAcc(Iscratch,IavgF);
      cvAbsDiff(Iscratch,IprevF,Iscratch2);
      cvAcc(Iscratch2,IdiffF);
      Icount += 1.0;
   }
   first = 0;
   cvCopy(Iscratch,IprevF);
}


void createModelsfromStats(){
   cvConvertScale(IavgF,IavgF,(double)(1.0/Icount));
   cvConvertScale(IdiffF,IdiffF,(double)(1.0/Icount));
   
   //Make sure diff is always something
   cvAddS(IdiffF,cvScalar(1.0,1.0,1.0),IdiffF);  
   setHighThreshold(7.0);
   setLowThreshold(6.0);
}


void setHighThreshold(float scale)
{
   cvConvertScale(IdiffF,Iscratch,scale);
   cvAdd(Iscratch,IavgF,IhiF);
   cvSplit( IhiF, Ihi1,Ihi2,Ihi3, 0 );
}


void setLowThreshold(float scale)
{
   cvConvertScale(IdiffF,Iscratch,scale);
   cvSub(IavgF,Iscratch,IlowF);
   cvSplit( IlowF, Ilow1,Ilow2,Ilow3, 0 );
}



// Create a binary: 0,255 mask where 255 means forehground pixel
// I      Input image, 3 channel, 8u
// Imask      Mask image to be created, 1 channel 8u
//
void backgroundDiff(IplImage *I,IplImage *Imask)
{
   cvCvtScale(I,Iscratch,1,0); //To float;
   cvSplit( Iscratch, Igray1,Igray2,Igray3, 0 );
   //Channel 1
   cvInRange(Igray1,Ilow1,Ihi1,Imask);
   //Channel 2
   cvInRange(Igray2,Ilow2,Ihi2,Imaskt);
   cvOr(Imask,Imaskt,Imask);
   //Channel 3
   cvInRange(Igray3,Ilow3,Ihi3,Imaskt);
   cvOr(Imask,Imaskt,Imask)
   //Finally, invert the results
   cvSubRS( Imask, 255, Imask);
}



void DeallocateImages()
{
   cvReleaseImage(&IavgF);
   cvReleaseImage(&IdiffF );
   cvReleaseImage(&IprevF );
   cvReleaseImage(&IhiF );
   cvReleaseImage(&IlowF );
   cvReleaseImage(&Ilow1  );
   cvReleaseImage(&Ilow2  );
   cvReleaseImage(&Ilow3  );
   cvReleaseImage(&Ihi1   );
   cvReleaseImage(&Ihi2   );
   cvReleaseImage(&Ihi3  );
   cvReleaseImage(&Iscratch);
   cvReleaseImage(&Iscratch2);
   cvReleaseImage(&Igray1  );
   cvReleaseImage(&Igray2 );
   cvReleaseImage(&Igray3 );
   cvReleaseImage(&Imaskt);
}

