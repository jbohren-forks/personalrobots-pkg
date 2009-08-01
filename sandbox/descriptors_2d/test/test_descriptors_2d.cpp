#include <descriptors_2d/descriptors_2d.h>
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;


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

Vector<KeyPoint> getPoints() {
  srand(0);
  Vector<KeyPoint> points;
  points.push_back(KeyPoint(342, 342, 1));
  points.push_back(KeyPoint(442, 442, 1));
  points.push_back(KeyPoint(242, 242, 1));
  points.push_back(KeyPoint(639, 479, 1));
  return points;
}

TEST(descriptors, SuperpixelColorHistogram) {
  SuperpixelColorHistogram desc(20, 0.5, 10);
  string name = "sch.txt";

  IplImage* img = cvLoadImage("test/frame0000.jpg");
  vvf results;
  Vector<KeyPoint> points = getPoints();
  desc.compute(img, points, results);

  mkdir("test/output", S_IRWXO | S_IRWXU);
  writeResultsToFile(results, "test/output/" + name);
  string command = "diff test/correct-output/" + name + " test/output/" + name + " > /dev/null";
  EXPECT_TRUE(system(command.c_str()) == 0);
}


TEST(descriptors, Hog) {
  HogWrapper desc;
  string name = "hog.txt";

  IplImage* img = cvLoadImage("test/frame0000.jpg");
  vvf results;
  Vector<KeyPoint> points = getPoints();
  desc.compute(img, points, results);

  mkdir("test/output", S_IRWXO | S_IRWXU);
  writeResultsToFile(results, "test/output/" + name);
  string command = "diff test/correct-output/" + name + " test/output/" + name + " > /dev/null";
  EXPECT_TRUE(system(command.c_str()) == 0);
}

TEST(descriptors, SURF) {
  SurfWrapper desc;
  string name = "surf.txt";

  IplImage* img = cvLoadImage("test/frame0000.jpg");
  vvf results;
  Vector<KeyPoint> points = getPoints();
  desc.compute(img, points, results);

  mkdir("test/output", S_IRWXO | S_IRWXU);
  writeResultsToFile(results, "test/output/" + name);
  string command = "diff test/correct-output/" + name + " test/output/" + name + " > /dev/null";
  EXPECT_TRUE(system(command.c_str()) == 0);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
