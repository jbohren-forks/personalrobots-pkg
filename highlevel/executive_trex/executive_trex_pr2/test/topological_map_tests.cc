/**
 * This file provudes unit tests for components used to integrate with a toplogical map. Note that the map
 * on question is a simplified component for testing pruposes. However, it should be sufficient for testing
 * the constraints and flaw handler capabilities for integrating a topological map representation
 * int NDDL and TREX
 * @author Conor McGann
 */


#include <executive_trex_pr2/topological_map.h>
#include <set>
#include <gtest/gtest.h>

TEST(executive_trex_pr2, map_connector_constraint){
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
