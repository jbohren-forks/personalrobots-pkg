
#include <fstream>
#include <sstream>
#include "write.h"

using namespace std;

void writeFloat2text( string filename, vector<float> & data)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure
    int vector_size = data.size();
    for (int k = 0; k < vector_size; k++){
        fout << data[k] << " ";
    }
    fout.close();
}
void writeInt2text( string filename, vector<int> & data)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure

    int vector_size = data.size();
    for (int k = 0; k < vector_size; k++){
        fout << data[k] << " ";
    }
    fout.close();
}
void writePoint2text( string filename, cv::Vector<cv::Point> & points)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure

    int vector_size = points.size();
    for (int k = 0; k < vector_size; k++){
        fout << points[k].x << " " << points[k].y << endl;
    }
    fout.close();
}
void writeRect2text( string filename, cv::Vector<cv::Rect> & Rect_)
{
    ofstream fout;
    fout.open(filename.c_str());
//    if (!fout)
//        return; // Failure

    int vector_size = Rect_.size();
    for (int k = 0; k < vector_size; k++){
        fout << Rect_[k].x << " " << Rect_[k].y << " "
            << Rect_[k].width << " " << Rect_[k].height << endl;
    }
    fout.close();
}
bool read2DFloatText( string filename, vector< vector<float> > & data)
{
    ifstream fp(filename.c_str(), ios::in);
    if (!fp)
        return false; // Failure

    std::string line;
    vector<float> tmp_float;
    // For each line...
    while (std::getline(fp, line)) {
        stringstream iss(line);
        tmp_float.clear();
        float ff;

        // For each character token...
        while (iss >> ff) {
            // Do something with c
            tmp_float.push_back( ff);
        }
        data.push_back( tmp_float);
    }
        fp.close();
    return true;
}
bool readFloattext( string filename, vector< vector<float> > & data)
{
    ifstream fp(filename.c_str(), ios::in | ios::binary);
    if (!fp)
        return false; // Failure
//    fp.seekg(0, ios::end);
//    const streamsize size = fp.tellg();
//    fp.seekg(0);


    float tmp_float;
    fp.read((char *)&tmp_float,sizeof(float));
    float NDim = tmp_float;
    data.resize(NDim);
    int dim_count = 0;
//    for (int i=0; i< size; i++){
    while (fp.good()){
        fp.read((char *)&tmp_float,sizeof(float));
        data.at(dim_count).push_back(tmp_float);
        dim_count++;
        if (dim_count >= NDim)
            dim_count = 0;
        }
//      cout << data.at(dim_count-1).size() << data.at(dim_count).size() << endl;
        data.at(dim_count-1).pop_back();
        fp.close();
}
bool readIntegertext( string filename, vector< int > & data)
{
    ifstream fin(filename.c_str(), ios::in | ios::binary);;
    if (!fin)
        return false; // Failure
    int tmp;
    fin.read((char *)&tmp,sizeof(int));
    int NDim = tmp;
    while (fin.good()){
        fin.read((char *)&tmp,sizeof(int));
        data.push_back(tmp);
    }
    fin.close();
    // pop_back one dummy element
    data.pop_back();
}
