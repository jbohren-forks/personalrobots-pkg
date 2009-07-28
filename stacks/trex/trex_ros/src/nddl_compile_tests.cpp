/**
 * This file uses the nddl-reader program in trex to test the compilation of files.
 * @author Tony Pratkanis
 */

#include <ros/console.h>
#include <fstream>
#include <vector>
#include <string>
#include <gtest/gtest.h>


/**
 * Test the compilation.
 */
TEST(executive_trex, compile_nddl_files) {
  system("pwd");

  std::vector<std::string> files;
  std::ifstream input("test_file_list.txt");

  ASSERT_EQ(false, input.fail());
  ASSERT_EQ(true, input.is_open());

  while(!input.eof()) {
    std::string name;
    std::getline(input, name);
    if (name != "") {
      files.push_back(name);
    }
  }

  for (unsigned int i = 0; i < files.size(); i++) {
    ROS_INFO("Run %s", files[i].c_str());
    unsigned int ret = system(std::string("rosrun trex nddl_tester " + files[i] + " 2> " + files[i] + ".testresult.txt").c_str());
    ROS_INFO("Finished %s", files[i].c_str()); 
    ASSERT_EQ(0, ret);
  }
}



int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
