#include <descriptors_2d/descriptors_2d.h>
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>

using namespace std;
using namespace cv;


#define MAX_INF_DIST 1
#define MAX_L2_DIST 1

void writeResultsToFile(vvf results, string name) {
  ofstream f(name.c_str(), ios::out);
  for(size_t i=0; i<results.size(); ++i) {
    for(size_t j=0; j<results[i].size(); ++j) {
      f << results[i][j] << " ";
    }
    f << endl;
  }
  f.close();
}

void writeResultsToFileBinary(const vvf& results, int result_size, string name) {
  FILE * pFile;
  pFile = fopen(name.c_str(), "wb");

  int num_results = results.size();
  fwrite(&num_results, sizeof(int), 1, pFile);
  fwrite(&result_size, sizeof(int), 1, pFile);
  bool valid = true;
  bool invalid = false;
  for(size_t i=0; i<results.size(); ++i) {
    if(results[i].empty()) 
      fwrite(&invalid, sizeof(bool), 1, pFile);
    else {
      fwrite(&valid, sizeof(bool), 1, pFile);
      fwrite(results[i].begin(), sizeof(float), results[i].size(), pFile);
    }
  }
  fclose(pFile);
}

void readResultsFromFileBinary(string name, vvf& results) {
  assert(results.empty());

  FILE* pFile;
  pFile = fopen(name.c_str(), "rb");
  if(!pFile)
    return;

  int result_size = 0;
  int num_results = 0;
  bool valid = false;
  fread(&num_results, sizeof(int), 1, pFile);
  fread(&result_size, sizeof(int), 1, pFile);
  for(int i=0; i<num_results; ++i) {
    fread(&valid, sizeof(bool), 1, pFile);
    if(valid) {
      float* results_buf = (float*) malloc(sizeof(float)*result_size);
      fread(results_buf, sizeof(float), result_size, pFile);
      results.push_back(Vector<float>(results_buf, result_size, false));
    }
    else {
      results.push_back(Vector<float>());
      assert(results.back().empty());
    }
  }
  fclose(pFile);
}


Vector<KeyPoint> getPoints() {
  srand(0);
  Vector<KeyPoint> points;
  points.push_back(KeyPoint(342, 342, 1));
  points.push_back(KeyPoint(442, 442, 1));
  points.push_back(KeyPoint(242, 242, 1));
  points.push_back(KeyPoint(639, 479, 1));
  return points;
}

bool compareResults(const Vector<float>& v1, const Vector<float>& v2) {
  assert(v1.size() == v2.size());
  double l2 = 0;
  double linf = 0;

  double sum_squares = 0;
  for(size_t i=0; i<v1.size(); ++i) {
    if(fabs(v1[i] - v2[i]) > linf) 
      linf = fabs(v1[i] - v2[i]);
    sum_squares += pow(v1[i] - v2[i], 2);
  }
  l2 = sqrt(sum_squares);

  if(linf < MAX_INF_DIST && l2 < MAX_L2_DIST)
    return true;
  else {
    cout << "linf: " << linf << ", l2: " << l2 << ".  MAX_L2_DIST = " << MAX_L2_DIST << " and MAX_INF_DIST = " << MAX_INF_DIST << endl;
    return false;
  }
}


TEST(descriptors, SuperpixelColorHistogram) {
  SuperpixelColorHistogram desc(20, 0.5, 10);
  string name = "sch.results";

  IplImage* img = cvLoadImage("test/frame0000.jpg");
  vvf results;
  Vector<KeyPoint> points = getPoints();
  desc.compute(img, points, results);

  mkdir("test/output", S_IRWXO | S_IRWXU);
  writeResultsToFileBinary(results, desc.getSize(), "test/output/" + name);
  vvf results2;
  readResultsFromFileBinary("test/correct-output/" + name, results2);
  for(size_t i=0; i<results.size(); ++i) {
    EXPECT_TRUE(compareResults(results[i], results2[i]));
  }
}

TEST(descriptors, Hog) {
  HogWrapper desc;
  string name = "hog.results";

  IplImage* img = cvLoadImage("test/frame0000.jpg");
  vvf results;
  Vector<KeyPoint> points = getPoints();
  desc.compute(img, points, results);

  mkdir("test/output", S_IRWXO | S_IRWXU);
  writeResultsToFileBinary(results, desc.getSize(), "test/output/" + name);
  vvf results2;
  readResultsFromFileBinary("test/correct-output/" + name, results2);
  for(size_t i=0; i<results.size(); ++i) {
    EXPECT_TRUE(compareResults(results[i], results2[i]));
  }
}

TEST(descriptors, SURF) {
  SurfWrapper desc;
  string name = "surf.results";

  IplImage* img = cvLoadImage("test/frame0000.jpg");
  vvf results;
  Vector<KeyPoint> points = getPoints();
  desc.compute(img, points, results);

  mkdir("test/output", S_IRWXO | S_IRWXU);
  writeResultsToFileBinary(results, desc.getSize(), "test/output/" + name);
  vvf results2;
  readResultsFromFileBinary("test/correct-output/" + name, results2);
  for(size_t i=0; i<results.size(); ++i) {
    EXPECT_TRUE(compareResults(results[i], results2[i]));
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
