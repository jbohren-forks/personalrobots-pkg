#include <urdf/URDF.h>
#include <gtest/gtest.h>
#include <cstdlib>
using namespace robot_desc;

TEST(URDF, EmptyFile)
{
    EXPECT_TRUE(true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
