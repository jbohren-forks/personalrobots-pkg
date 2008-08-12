#include <urdf/URDF.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <string>
using namespace robot_desc;

int runExternalProcess(const std::string &executable, const std::string &args)
{
    return system((executable + " " + args).c_str());
}

TEST(URDF, EmptyFile)
{
    URDF file;
    EXPECT_TRUE(file.loadFile("test/data/test1.xml"));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
