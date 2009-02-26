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
#include "Assembly.hh"

using namespace executive_trex_pr2;
using namespace TREX;
using namespace EUROPA;


TREX::Assembly assembly("pr2", "test");
ConstraintEngineId ce(assembly.getConstraintEngine());

/**
 * Given a spatial window, pich a random float in that area
 */
void pickPointInSpace(unsigned int W, unsigned int H, double& x, double& y){
  double mantissa = 100.0 / (rand() % 100 + 1); 
  x = (rand() % W) + mantissa;
  y = (rand() % H) + mantissa;
}

TEST(executive_trex_pr2, map_accessor){
  // Generate points in the space and make sure we get a region for each point
  for(unsigned int counter = 0; counter < 10000; counter++){
    // Obtain random values for x and y
    double x, y;
    pickPointInSpace(TopologicalMapAccessor::instance()->getNumCols() + 5, TopologicalMapAccessor::instance()->getNumRows() + 5, x, y);

    unsigned int region_id = TopologicalMapAccessor::instance()->getRegion(x, y);
    if(region_id == 0){
      bool valid = TopologicalMapAccessor::instance()->isObstacle(x, y) || x >= TopologicalMapAccessor::instance()->getNumCols() || y >= TopologicalMapAccessor::instance()->getNumRows();
      ROS_INFO_COND(!valid, "Should be off the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
    else {
      bool valid = !TopologicalMapAccessor::instance()->isObstacle(x, y) && x < TopologicalMapAccessor::instance()->getNumCols() && y < TopologicalMapAccessor::instance()->getNumRows();
      ROS_INFO_COND(!valid, "Should be on the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
  }
}

TEST(executive_trex_pr2, map_accessor_regression_tests){
  // Check a point that shoud be in a region.
  unsigned int region_id = TopologicalMapAccessor::instance()->getRegion(8.5, 0.5);
  ROS_INFO_COND(region_id == 0, "Should be on the map but isn't with (%f, %f)", 8.5, 0.5);
  ASSERT_TRUE(region_id > 0 || TopologicalMapAccessor::instance()->isObstacle(8.5, 0.5));
}

/**
 * Tesy the relation for map connector to point
 */
TEST(executive_trex_pr2, map_connector_constraint){

  Variable<IntervalIntDomain> v_connector(ce, IntervalIntDomain());
  Variable<IntervalDomain> v_x(ce, IntervalDomain());
  Variable<IntervalDomain> v_y(ce, IntervalDomain());

  MapConnectorConstraint::MapConnectorConstraint map_connector("map_connector", "Default", ce, makeScope(v_connector.getId(), v_x.getId(), v_y.getId()));

  // Select all the connectors by iterating over numbers that might be
  for(unsigned int i = 0; i < (TopologicalMapAccessor::instance()->getNumRows() * TopologicalMapAccessor::instance()->getNumCols()); i++){
    double x, y;
    if(TopologicalMapAccessor::instance()->getConnectorPosition(i, x, y)){

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
      v_x.specify(x + TopologicalMapAccessor::instance()->getResolution() * 2);
      v_y.specify(y + TopologicalMapAccessor::instance()->getResolution() * 2);
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

  Variable<IntervalIntDomain> v_region(ce, IntervalIntDomain());
  Variable<IntervalDomain> v_x(ce, IntervalDomain());
  Variable<IntervalDomain> v_y(ce, IntervalDomain());

  MapGetRegionFromPositionConstraint::MapGetRegionFromPositionConstraint map_region_from_position("map_region_from_position", "Default", ce, makeScope(v_region.getId(), v_x.getId(), v_y.getId()));

  // Generate points in the space and make sure we get a region for each point
  for(unsigned int counter = 0; counter < 100; counter++){
    // Obtain random values for x and y
    double x, y;
    pickPointInSpace(TopologicalMapAccessor::instance()->getNumCols() + 5, TopologicalMapAccessor::instance()->getNumRows() + 5, x, y);
    unsigned int region_id = TopologicalMapAccessor::instance()->getRegion(x, y);

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

  Variable<BoolDomain> v_result(ce, IntervalIntDomain());
  Variable<IntervalIntDomain> v_region(ce, IntervalIntDomain());

  MapIsDoorwayConstraint::MapIsDoorwayConstraint map_is_doorway("map_is_doorway", "Default", ce, makeScope(v_result.getId(), v_region.getId()));

  // Select for a bunch of IDS
  for(unsigned int id = 0; id < (TopologicalMapAccessor::instance()->getNumCols() * TopologicalMapAccessor::instance()->getNumRows()); id++){

    // If it is not a region, then binding it should produce an inconsistency
    v_region.specify(id);
    bool is_doorway(true);
    if(!TopologicalMapAccessor::instance()->isDoorway(id, is_doorway))
      ASSERT_FALSE(ce->propagate());
    else {
      ASSERT_TRUE(ce->propagate());
      ASSERT_TRUE(v_result.derivedDomain().isSingleton());
      ASSERT_EQ(v_result.derivedDomain().getSingletonValue(), is_doorway);
    }

    v_region.reset();
  }
}

int main(int argc, char** argv){
  srand(NULL);
  std::ofstream debug_file("Debug.log");
  DebugMessage::setStream(debug_file);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
