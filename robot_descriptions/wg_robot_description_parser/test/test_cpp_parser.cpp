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
    
    std::ofstream f("/tmp/test1.txt");
    file.print(f);
    f.close();
    int result = runExternalProcess("diff", "test/data/test1.txt /tmp/test1.txt");
        
    EXPECT_TRUE(result == 0);
}

TEST(URDF, SimpleFile)
{
    URDF file;
    file.loadFile("test/data/test2.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);
    
    std::ofstream f("/tmp/test2.txt");
    file.print(f);
    f.close();
    int result = runExternalProcess("diff", "test/data/test2.txt /tmp/test2.txt");
        
    EXPECT_TRUE(result == 0);
}


TEST(URDF, ComplexFile)
{
    URDF file;
    file.loadFile("test/data/test3.xml");
    EXPECT_TRUE(file.getErrorCount() == 0);

    std::ofstream f("/tmp/test3.txt");
    file.print(f);
    f.close();
    int result = runExternalProcess("diff", "test/data/test3.txt /tmp/test3.txt");
        
    EXPECT_TRUE(result == 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
