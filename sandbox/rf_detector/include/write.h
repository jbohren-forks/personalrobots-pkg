

#include "cxcore.h"
# include <vector>
using namespace std;

void writeFloat2text( string filename, vector<float> & data);
void writeInt2text( string filename, vector<int> & data);
void writePoint2text( string filename, cv::Vector<cv::Point> & points);
void writeRect2text( string filename, cv::Vector<cv::Rect> & Rect_);
bool read2DFloatText( string filename, vector< vector<float> > & data);
bool readFloattext( string filename, vector< vector<float> > & data);
bool readIntegertext( string filename, vector< int > & data);
