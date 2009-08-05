#include <descriptors_2d_gpl/descriptors_2d_gpl.h>
#include <descriptors_2d/test_descriptors_2d.h>
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>


using namespace std;
using namespace cv;

TEST(descriptors, Daisy) {
  Daisy desc;
  string name = "daisy.results";
  EXPECT_TRUE(descriptorTest(&desc, name));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
