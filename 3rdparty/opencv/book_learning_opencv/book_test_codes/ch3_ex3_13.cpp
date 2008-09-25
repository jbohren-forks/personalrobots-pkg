#include <cv.h>
#include <highgui.h>

int main(int argc, char** argv)
{
    IplImage* src;
    if( argc == 7 && ((src=cvLoadImage(argv[1],1)) != 0 ))
    {
		int x = atoi(argv[2]);
		int y = atoi(argv[3]);
		int width = atoi(argv[4]);
		int height = atoi(argv[5]);
		int add = atoi(argv[6]);
		cvSetImageROI(src, cvRect(x,y,width,height));
		cvAddS(src, cvScalar(add),src);
		cvResetImageROI(src);
        cvNamedWindow( "Roi_Add", 1 );
        cvShowImage( "Roi_Add", src );
        cvWaitKey();
    }
    return 0;
}
