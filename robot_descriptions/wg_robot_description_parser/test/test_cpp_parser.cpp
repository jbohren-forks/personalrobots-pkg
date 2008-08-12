#include <urdf/URDF.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <string>
using namespace robot_desc;

int runExternalProcess(const std::string &executable, const std::string &args)
{
    return system((executable + " " + args).c_str());
}

TEST(URDF, EmptyFile)
{
    URDF file;
    file.loadFile("test/data/test1.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);
    std::stringstream ss1;
    file.print(ss1);
    
    std::stringstream ss2;
    std::ifstream in2("test/data/test1.txt");
    ss2 << in2;
    
    printf("'%s'\n", ss2.str().c_str());
    
    EXPECT_TRUE(ss1 == ss2);
}
/*
TEST(URDF, SimpleFile)
{
    URDF file;
    file.loadFile("test/data/test2.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);
}

TEST(URDF, ComplexFile)
{
    URDF file;
    file.loadFile("test/data/test3.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);
}
*/
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
