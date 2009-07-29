/* Test system for outlet detector
Created by: Alexey Latyshev
*/

#ifndef _OUTLET_DETECTOR_TEST_H
#define _OUTLET_DETECTOR_TEST_H

//#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
//using namespace std;

#include "outlet_detection/outlet_detector.h"

// Base structure for testing. Contains train and test outlet and number of matching between.
// n_matches == -1 means that there was no testing yet.
typedef struct 
{
	char filename[1024];
	vector<outlet_t> test_outlet;
	vector<outlet_t> real_outlet;
	int n_matches;
} outlet_test_elem;


//Reading test file from the hard drive
//Returns number of outlets in the test template
// Test file has the following structure:
// First line: n - number of holes
// Oter lines: filename,x1,y1,... 
// where xi and yi are the coordinates of holes. Power holes are the first ones, ground holes are the last ones.
// Number of ground holes is in two times less than number of power holes.
int readTestFile(char* filename,vector<outlet_test_elem>& test_data);

int writeTestFile(char* filename,vector<outlet_test_elem>& data);

//Copies test outlets into real outlets. So we are able to create test file by test resultes
void convertTestToReal(vector<outlet_test_elem>& data);

//Shows Real Outlet to the user
void showRealOutlet(const outlet_test_elem& test_elem, CvMat* intrinsic_matrix, CvMat* distortion_params);

IplImage* getRealOutletImage(const outlet_test_elem& test_elem, CvMat* intrinsic_matrix, CvMat* distortion_params);

//Corrects Real Outlet Position
void setRealOutlet(outlet_test_elem& test_elem, CvMat* intrinsic_matrix = 0, CvMat* distortion_params = 0);

// Writes the file with test resultes
int writeTestResults(char* filename, const vector<outlet_test_elem>& test_data);

int compareOutlets(outlet_test_elem& test_elem, int accuracy = 3);
int compareAllOutlets(vector<outlet_test_elem>& test_data, int accuracy = 3);

//run test
void runOutletDetectorTest(CvMat* intrinsic_matrix, CvMat* distortion_params, const outlet_template_t& outlet_templ,vector<outlet_test_elem>& test_data, char* output_path = 0);

#endif