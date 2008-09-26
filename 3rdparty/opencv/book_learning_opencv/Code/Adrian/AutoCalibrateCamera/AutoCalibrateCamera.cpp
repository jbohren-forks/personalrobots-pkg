// AutoCalibrateCamera.cpp : Defines the entry point for the console application.
//
#include <cv.h>
#include <highgui.h>

const int n_boards = 12;
const int board_dt = 10;
const int board_w  = 7;
const int board_h  = 5;
const int board_n  = board_w * board_h;
const CvSize board_sz = cvSize( board_w, board_h );

int main(int argc, char* argv[]) {
	
	CvCapture* capture = cvCreateCameraCapture( 0 );
	assert( capture );

	cvNamedWindow( "Calibration" );

	CvMat* object_points     = cvCreateMat( n_boards * board_n, 3, CV_32FC1 );
	CvMat* image_points      = cvCreateMat( n_boards * board_n, 2, CV_32FC1 );
	CvMat* point_counts      = cvCreateMat( n_boards, 1, CV_32FC1 );
	CvMat* intrinsic_matrix  = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* distortion_coeffs = cvCreateMat( 4, 1, CV_32FC1 );

	IplImage* image = cvQueryFrame( capture );
	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	int corner_count;
	int successes = 0;
	for( int frame=0; image; frame++ ) {

		cvShowImage( "Calibration", image );

		// every Nth frame try to grab a chessboard.
		//
		if( frame % board_dt == 0 ) {

			int found = cvFindChessboardCorners(
				image,
				board_sz,
				corners,
				&corner_count, 
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
			);

			// If we got a good board, add it to our data
			//
			if( found = board_n ) {
				for( int i=0; i<board_n; i++ ) {
					CV_MAT_ELEM( *image_points,  float, i, 0 ) = corners[i].x;
					CV_MAT_ELEM( *image_points,  float, i, 1 ) = corners[i].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = (float)(i/board_w);
					CV_MAT_ELEM( *object_points, float, i, 1 ) = (float)(i%board_w);
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
					CV_MAT_ELEM( *point_counts,  float, i, 0 ) = board_n;
				}
				successes++;
			}
		}
			
		// (attempt to) get the next frame
		//
		image = cvQueryFrame( capture );

		if( successes == n_boards ) break;

		cvWaitKey(15);
	}
	
	// At this point we have all of the chessboard corners we need.
	//
	{
		// Initialize the intrinsic matrix such that the two focal
		// lengths have a ratio of 1.0
		//
		CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
		CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

		cvCalibrateCamera2(
			object_points,
			image_points,
			point_counts,
			cvGetSize( image ),
			intrinsic_matrix,
			distortion_coeffs,
			NULL,
			NULL,
			CV_CALIB_FIX_ASPECT_RATIO
		);
	}

	// Build the undistort map which we will use for all 
	// subsequent frames.
	//
	IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap(
		intrinsic_matrix,
		distortion_coeffs,
		mapx,
		mapy
	);

	// Just run the camera to the screen, now only showing the undistorted
	// image.
	//
	while( image ) {
			
		cvRemap( image, image, mapx, mapy );
		image = cvQueryFrame( capture );
		cvWaitKey(15);	
	
	}

	return 0;
}

