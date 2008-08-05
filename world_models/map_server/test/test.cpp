#include "map_server/map_server.h"
#include <gtest/gtest.h>

TEST(MapServer, ImageLoading)
{
  EXPECT_EQ(4,4);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
