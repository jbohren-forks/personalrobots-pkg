/**
 * This file provides unit tests for components used to integrate with a toplogical map. Note that the map
 * on question is a simplified component for testing pruposes. However, it should be sufficient for testing
 * the constraints and flaw handler capabilities for integrating a topological map representation
 * int NDDL and TREX
 * @author Conor McGann
 */

#include <ros/console.h>
#include <executive_trex_pr2/topological_map.h>
#include <set>
#include <gtest/gtest.h>

using namespace executive_trex_pr2;

void setV (topological_map::OccupancyGrid& grid, unsigned r0, unsigned dr, unsigned rmax, unsigned c0, unsigned dc, unsigned cmax, bool val) 
{
  for (unsigned r=r0; r<rmax; r+=dr) {
    for (unsigned c=c0; c<cmax; c+=dc) {
      grid[r][c] = val;
    }
  }
}

// Get a sample grid:
// Dimensions: 21 * 24
// Resolution: 1
static const unsigned int HEIGHT_21(21);
static const unsigned int WIDTH_24(24);
const topological_map::OccupancyGrid& GRID_3_3_ALL_CONNECTED(){
  static topological_map::OccupancyGrid grid(boost::extents[HEIGHT_21][WIDTH_24]);
  static bool initialized(false);
  if(!initialized){
    setV(grid, 0, 1, HEIGHT_21, 0, 1, WIDTH_24, false);
    setV(grid, 7, 7, HEIGHT_21, 0, 1, WIDTH_24, true);
    setV(grid, 0, 1, HEIGHT_21, 8, 8, WIDTH_24, true);
    setV(grid, 3, 7, HEIGHT_21, 8, 8, WIDTH_24, false);
    setV(grid, 7, 7, HEIGHT_21, 4, 8, WIDTH_24, false);
    initialized = true;
  }

  return grid;
}

TEST(executive_trex_pr2, map_accessor){
  const topological_map::OccupancyGrid& grid = GRID_3_3_ALL_CONNECTED();
  TopologicalMapAdapter map(grid, 1.0);

  // Generate points in the space and make sure we get a region for each point
  for(unsigned int counter = 0; counter < 10000; counter++){
    // Obtain random values for x and y
    double mantissa = 100.0 / (rand() % 100 + 1); 
    double x = (rand() % (WIDTH_24 + 5)) + mantissa;
    double y = (rand() % (HEIGHT_21 + 5)) + mantissa;

    unsigned int region_id = map.getRegion(x, y);
    if(region_id == 0){
      bool valid = map.isObstacle(x, y) || x >= WIDTH_24 || y >= HEIGHT_21;
      ROS_INFO_COND(!valid, "Should be off the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
    else {
      bool valid = !map.isObstacle(x, y) && x < WIDTH_24 && y < HEIGHT_21;
      ROS_INFO_COND(!valid, "Should be on the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
  }
}

int main(int argc, char** argv){
  srand(NULL);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
