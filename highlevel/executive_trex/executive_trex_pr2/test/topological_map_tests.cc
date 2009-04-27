/**
 * This file provides unit tests for components used to integrate with a toplogical map. Note that the map
 * on question is a simplified component for testing pruposes. However, it should be sufficient for testing
 * the constraints and flaw handler capabilities for integrating a topological map representation
 * int NDDL and TREX
 * @author Conor McGann
 */

#include <ros/console.h>
#include <robot_msgs/Door.h>
#include <executive_trex_pr2/topological_map.h>
#include <set>
#include <gtest/gtest.h>
#include "Assembly.hh"

using namespace executive_trex_pr2;
using namespace TREX;
using namespace EUROPA;


TREX::Assembly assembly("pr2", "test");
ConstraintEngineId ce(assembly.getConstraintEngine());

/**
 * Utility to print rows and columns that are in doorways
 */
void printDoors(std::ostream& os){
  unsigned int region_id(1);
  unsigned int door_counter(0);

  bool is_doorway(false);
  while(TopologicalMapAdapter::instance()->isDoorway(region_id, is_doorway)){

    if(is_doorway){
      door_counter++;
      os << "Door in region " << region_id << " containing cells:\n";
      const topological_map::Region& region = *(TopologicalMapAdapter::instance()->getRegionCells(region_id));
      for(topological_map::Region::const_iterator it = region.begin(); it != region.end(); ++it){
	const topological_map::Cell2D& cell = *it;
	os << "  [" << cell.c << ", " << cell.r << "]\n";
      }

    }

    region_id++;
  }

  os << door_counter << " doors in all\n";
}

/**
 * A Default Grid Structure to use when testing
 */
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
static const double RESOLUTION(1.0);

topological_map::OccupancyGrid& GRID_3_3_ALL_CONNECTED(){
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

// Shared setup for all tests

/**
 * Given a spatial window, pich a random float in that area
 */
void pickPointInSpace(unsigned int W, unsigned int H, double& x, double& y){
  double mantissa = 100.0 / (rand() % 100 + 1); 
  x = (rand() % W) + mantissa;
  y = (rand() % H) + mantissa;
}


/**
 * Test the function to get the next move
 */
TEST(executive_trex_pr2, map_get_next_move){
  std::ifstream is("test/willow.tmap");
  TopologicalMapAdapter map(is);
  Variable<IntervalDomain> next_x(ce, IntervalDomain());
  Variable<IntervalDomain> next_y(ce, IntervalDomain());
  Variable<BoolDomain> thru_doorway(ce, BoolDomain());
  Variable<IntervalDomain> current_x(ce, IntervalDomain());
  Variable<IntervalDomain> current_y(ce, IntervalDomain());
  Variable<IntervalDomain> target_x(ce, IntervalDomain());
  Variable<IntervalDomain> target_y(ce, IntervalDomain());
  MapGetNextMoveConstraint::MapGetNextMoveConstraint map_get_next_move("map_next_move", "Default", ce, 
								  makeScope(next_x.getId(), next_y.getId(), thru_doorway.getId(), current_x.getId(), current_y.getId(), target_x.getId(), target_y.getId()));

  ASSERT_TRUE(ce->propagate());
}

/**
 * Test reading a map in from a file
 */
TEST(executive_trex_pr2, map_read_from_file){
  std::ifstream is("test/willow.tmap");
  TopologicalMapAdapter map(is);
  std::ofstream os("doors.willow.out");
  printDoors(os);


  // Here is a set of points that should be OK
  ASSERT_EQ(map.isObstacle(19.45, 28.95), false);
  ASSERT_EQ(map.isObstacle(16.5, 28.4), false);
  ASSERT_EQ(map.isObstacle(15.0, 25.1), false);
  ASSERT_EQ(map.isObstacle(21.15, 32.0), false);
  ASSERT_EQ(map.isObstacle(47.350,  6.595), false);

  ASSERT_EQ(map.isObstacle(54.162, 20.087), false);
  ASSERT_EQ(map.isObstacle(12.118, 39.812), false);
  ASSERT_EQ(map.isObstacle(18.912, 28.568), false);
  ASSERT_EQ(map.isObstacle(18.542, 12.301), false);
  ASSERT_EQ(map.isObstacle(50.659, 6.916), false);

  // Points that are ligitimately obstacles
  ASSERT_EQ(map.isObstacle(15.0, 22.05), true);


  robot_msgs::Door door_state;
  ASSERT_EQ(TopologicalMapAdapter::instance()->getDoorState(206, door_state), true);
}

TEST(executive_trex_pr2, map_accessor){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);

  // Generate points in the space and make sure we get a region for each point
  for(unsigned int counter = 0; counter < 10000; counter++){
    // Obtain random values for x and y
    double x, y;
    pickPointInSpace(WIDTH_24 + 5, HEIGHT_21 + 5, x, y);

    unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(x, y);
    if(region_id == 0){
      bool valid = TopologicalMapAdapter::instance()->isObstacle(x, y) || x >= WIDTH_24 || y >= HEIGHT_21;
      ROS_INFO_COND(!valid, "Should be off the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
    else {
      bool valid = !TopologicalMapAdapter::instance()->isObstacle(x, y) && x < WIDTH_24 && y < HEIGHT_21;
      ROS_INFO_COND(!valid, "Should be on the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
  }
}

TEST(executive_trex_pr2, map_accessor_regression_tests){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);
  // Check a point that shoud be in a region.
  unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(8.5, 0.5);
  ROS_INFO_COND(region_id == 0, "Should be on the map but isn't with (%f, %f)", 8.5, 0.5);
  ASSERT_TRUE(region_id > 0 || TopologicalMapAdapter::instance()->isObstacle(8.5, 0.5));
}


/**
 * Test the relation for map connector to point
 */
TEST(executive_trex_pr2, map_connector_constraint){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);

  Variable<IntervalIntDomain> v_connector(ce, IntervalIntDomain());
  Variable<IntervalDomain> v_x(ce, IntervalDomain());
  Variable<IntervalDomain> v_y(ce, IntervalDomain());

  MapConnectorConstraint::MapConnectorConstraint map_connector("map_connector", "Default", ce, makeScope(v_connector.getId(), v_x.getId(), v_y.getId()));

  // Select all the connectors by iterating over numbers that might be
  for(unsigned int i = 0; i < (HEIGHT_21 * WIDTH_24); i++){
    double x, y;
    if(TopologicalMapAdapter::instance()->getConnectorPosition(i, x, y)){

      // Now bind the connector and verify x and y. Going this direction, the relationship can be exact
      v_connector.specify(i);
      ASSERT_TRUE(ce->propagate());
      ROS_INFO_COND(!v_x.derivedDomain().isSingleton(), v_x.toString().c_str());
      ASSERT_TRUE(v_x.derivedDomain().isSingleton());
      ASSERT_EQ(v_x.derivedDomain().getSingletonValue(), x);
      ASSERT_TRUE(v_y.derivedDomain().isSingleton());
      ROS_INFO_COND(!v_y.derivedDomain().isSingleton(), v_y.toString().c_str());
      ASSERT_EQ(v_y.derivedDomain().getSingletonValue(), y);
      v_connector.reset();

      // Now bind x and y to get i. Note that we will expect a reasonable error bound.
      v_x.specify(x);
      v_y.specify(y);
      ASSERT_TRUE(ce->propagate());
      ASSERT_TRUE(v_connector.isSingleton());
      ASSERT_EQ(v_connector.derivedDomain().getSingletonValue(), i);
      v_x.reset();
      v_y.reset();

      // Conversely, when x and y are outside of the error bound, we should not see a match. So if we bind all
      // then we get an inconsistency
      v_connector.specify(i);
      v_x.specify(x + RESOLUTION * 2);
      v_y.specify(y + RESOLUTION * 2);
      ASSERT_FALSE(ce->propagate());
      v_connector.reset();
      v_x.reset();
      v_y.reset();
    }
  }
}



/**
 * Tes the relation for map connector to point
 */
TEST(executive_trex_pr2, map_region_from_position_constraint){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);

  Variable<IntervalIntDomain> v_region(ce, IntervalIntDomain());
  Variable<IntervalDomain> v_x(ce, IntervalDomain());
  Variable<IntervalDomain> v_y(ce, IntervalDomain());

  MapGetRegionFromPositionConstraint::MapGetRegionFromPositionConstraint map_region_from_position("map_region_from_position", "Default", ce, makeScope(v_region.getId(), v_x.getId(), v_y.getId()));

  // Generate points in the space and make sure we get a region for each point
  for(unsigned int counter = 0; counter < 100; counter++){
    // Obtain random values for x and y
    double x, y;
    pickPointInSpace(WIDTH_24 + 5, HEIGHT_21 + 5, x, y);
    unsigned int region_id = TopologicalMapAdapter::instance()->getRegion(x, y);

    // Now bind x and y - should propagate
    v_x.specify(x);
    v_y.specify(y);
    ASSERT_TRUE(ce->propagate());
    ASSERT_TRUE(v_region.isSingleton());
    ASSERT_EQ(v_region.derivedDomain().getSingletonValue(), region_id);
    v_x.reset();
    v_y.reset();
  }
}

/**
 * Test the check for a region being a doorway
 */
TEST(executive_trex_pr2, map_is_doorway_constraint){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);

  Variable<BoolDomain> v_result(ce, IntervalIntDomain());
  Variable<IntervalIntDomain> v_region(ce, IntervalIntDomain());

  MapIsDoorwayConstraint::MapIsDoorwayConstraint map_is_doorway("map_is_doorway", "Default", ce, makeScope(v_result.getId(), v_region.getId()));

  // Select for a bunch of IDS
  for(unsigned int id = 0; id < (WIDTH_24 * HEIGHT_21); id++){

    v_region.specify(id);
    bool is_doorway(true);
    if(!TopologicalMapAdapter::instance()->isDoorway(id, is_doorway))
      ASSERT_FALSE(is_doorway);
    else {
      ASSERT_TRUE(ce->propagate());
      ASSERT_TRUE(v_result.derivedDomain().isSingleton());
      ASSERT_EQ(v_result.derivedDomain().getSingletonValue(), is_doorway);
    }

    v_region.reset();
  }
}

/**
 * Test reading a map from a grid and writing the doors found
 */
TEST(executive_trex_pr2, map_output_doors){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);
  std::ofstream os("doors.out");
  printDoors(os);
}

int main(int argc, char** argv){
  srand(NULL);
  std::ofstream debug_file("Debug.log");
  DebugMessage::setStream(debug_file);
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
