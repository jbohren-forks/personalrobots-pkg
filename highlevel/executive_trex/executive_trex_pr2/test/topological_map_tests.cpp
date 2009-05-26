/**
 * This file provides unit tests for components used to integrate with a toplogical map. Note that the map
 * on question is a simplified component for testing pruposes. However, it should be sufficient for testing
 * the constraints and flaw handler capabilities for integrating a topological map representation
 * int NDDL and TREX
 * @author Conor McGann
 */

#include <ros/console.h>
#include <door_msgs/Door.h>
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
  TopologicalMapAdapter map(is, "test/willow.tmap.door_overrides.xml");
  Variable<IntervalDomain> next_x(ce, IntervalDomain(), false, true, "x");
  Variable<IntervalDomain> next_y(ce, IntervalDomain(), false, true, "y");
  Variable<IntervalDomain> next_z(ce, IntervalDomain(), false, true, "z");
  Variable<IntervalDomain> next_qx(ce, IntervalDomain(), false, true, "qx");
  Variable<IntervalDomain> next_qy(ce, IntervalDomain(), false, true, "qy");
  Variable<IntervalDomain> next_qz(ce, IntervalDomain(), false, true, "qz");
  Variable<IntervalDomain> next_qw(ce, IntervalDomain(), false, true, "qw");
  Variable<BoolDomain> thru_doorway(ce, BoolDomain(), false, true, "thru_doorway");
  Variable<IntervalDomain> current_x(ce, IntervalDomain(), false, true, "current_x");
  Variable<IntervalDomain> current_y(ce, IntervalDomain(), false, true, "current_y");
  Variable<IntervalDomain> target_x(ce, IntervalDomain(), false, true, "target_x");
  Variable<IntervalDomain> target_y(ce, IntervalDomain(), false, true, "target_y");
  Variable<IntervalDomain> target_z(ce, IntervalDomain(), false, true, "target_z");
  Variable<IntervalDomain> target_qx(ce, IntervalDomain(), false, true, "target_qx");
  Variable<IntervalDomain> target_qy(ce, IntervalDomain(), false, true, "target_qy");
  Variable<IntervalDomain> target_qz(ce, IntervalDomain(), false, true, "target_qz");
  Variable<IntervalDomain> target_qw(ce, IntervalDomain(), false, true, "target_qw");

  std::vector<ConstrainedVariableId> scope;
  scope.push_back(next_x.getId());
  scope.push_back(next_y.getId());
  scope.push_back(next_z.getId());
  scope.push_back(next_qx.getId());
  scope.push_back(next_qy.getId());
  scope.push_back(next_qz.getId());
  scope.push_back(next_qw.getId());
  scope.push_back(thru_doorway.getId());
  scope.push_back(current_x.getId());
  scope.push_back(current_y.getId());
  scope.push_back(target_x.getId());
  scope.push_back(target_y.getId());
  scope.push_back(target_z.getId());
  scope.push_back(target_qx.getId());
  scope.push_back(target_qy.getId());
  scope.push_back(target_qz.getId());
  scope.push_back(target_qw.getId());

  target_z.specify(0);
  target_qx.specify(0);
  target_qy.specify(0);
  target_qz.specify(0);
  target_qw.specify(0);

  MapGetNextMoveConstraint::MapGetNextMoveConstraint map_get_next_move("map_next_move", "Default", ce, scope);
  ASSERT_TRUE(ce->propagate());
  /*
ARG[8]:x(6081) DERIVED=float:CLOSED[14.584563537376155,
 14.584563537376155]
 ARG[9]:y(6082) DERIVED=float:CLOSED[26.792833131576032,
 26.792833131576032]
 ARG[10]:x(478) DERIVED=float:CLOSED[12.062500000000000,
 12.062500000000000]
 ARG[11]:y(479) DERIVED=float:CLOSED[22.287500000000001,
 22.287500000000001]
 ARG[12]:z(480) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[13]:qx(481) DERIVED=float:CLOSED[0.000000000000000,
 0.000000000000000]
 ARG[14]:qy(482) DERIVED=float:CLOSED[0.000000000000000,
 0.000000000000000]
 ARG[15]:qz(483) DERIVED=float:CLOSED[-0.767812094850370,
 -0.767812094850370]
 ARG[16]:qw(484) DERIVED=float:CLOSED[0.640675102529735,
 0.640675102529735]
   */
  /* COMMENT OUT FOR NOW. A test for Ticket: 1480
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(14.5845);
  current_y.specify(26.7928);
  target_x.specify(12.0625);
  target_y.specify(22.2875);
  ASSERT_TRUE(ce->propagate());
  */
  /*
ARG[8]:x(17085) (S)  DERIVED=float:CLOSED[12.737500000000001, 12.737500000000001]
 ARG[9]:y(17086) (S)  DERIVED=float:CLOSED[22.062500000000000, 22.062500000000000]
 ARG[10]:x(478) (S)  DERIVED=float:CLOSED[12.699999999999999, 12.699999999999999]
 ARG[11]:y(479) (S)  DERIVED=float:CLOSED[22.500000000000000, 22.500000000000000]
 ARG[12]:z(480) (S)  DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[13]:qx(481) (S)  DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[14]:qy(482) (S)  DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[15]:qz(483) (S)  DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[16]:qw(484) (S)  DERIVED=float:CLOSED[1.000000000000000, 1.000000000000000]
  */

  // Make sure it handles going straight to target
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(16.2587);
  current_y.specify(17.5384);
  target_x.specify(12.6999);
  target_y.specify(22.5000);
  ASSERT_TRUE(ce->propagate());

  /*
    [Map:get_next_move]AFTER: [1242925482.950660]map_get_next_move(6066)
    ARG[0]:x(6049) DERIVED=float:CLOSED[14.512500000000001, 14.512500000000001]
    ARG[1]:y(6050) DERIVED=float:CLOSED[19.087500000000002, 19.087500000000002]
    ARG[2]:z(6051) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[3]:qx(6052) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[4]:qy(6053) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[5]:qz(6054) DERIVED=float:CLOSED[-0.723346873689404, -0.723346873689404]
    ARG[6]:qw(6055) DERIVED=float:CLOSED[0.690484829901255, 0.690484829901255]
    ARG[7]:thru_doorway(6064) DERIVED=bool:CLOSED[0, 0]
    ARG[8]:x(5796) DERIVED=float:CLOSED[16.258719465113977, 16.258719465113977]
    ARG[9]:y(5797) DERIVED=float:CLOSED[17.538415136846279, 17.538415136846279]
    ARG[10]:x(480) DERIVED=float:CLOSED[12.699999999999999, 12.699999999999999]
    ARG[11]:y(481) DERIVED=float:CLOSED[22.500000000000000, 22.500000000000000]
  */

  // Make sure it does not go too far
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(16.2587);
  current_y.specify(17.538);
  target_x.specify(12.6999);
  target_y.specify(22.5000);
  ASSERT_TRUE(ce->propagate());
  ASSERT_TRUE(next_y.lastDomain().getSingletonValue()  < 18);

  // Make sure it goes far enough across a door
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(14.41);
  current_y.specify(17.53);
  target_x.specify(12.6999);
  target_y.specify(22.5000);
  ASSERT_TRUE(ce->propagate());
  ASSERT_TRUE(next_y.lastDomain().getSingletonValue()  > 18);
  /*
 ARG[0]:x(41449) DERIVED=float:CLOSED[12.737500000000001, 12.737500000000001]
 ARG[1]:y(41450) DERIVED=float:CLOSED[22.062500000000000, 22.062500000000000]
 ARG[2]:z(41451) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[3]:qx(41452) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[4]:qy(41453) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[5]:qz(41454) DERIVED=float:CLOSED[-0.102813666709054, -0.102813666709054]
 ARG[6]:qw(41455) DERIVED=float:CLOSED[0.994700633325344, 0.994700633325344]
 ARG[7]:thru_doorway(41464) DERIVED=bool:CLOSED[1, 1]
 ARG[8]:x(41196) DERIVED=float:CLOSED[12.713803256264846, 12.713803256264846]
 ARG[9]:y(41197) DERIVED=float:CLOSED[22.456525936838421, 22.456525936838421]
 ARG[10]:x(521) DERIVED=float:CLOSED[19.237500000000001, 19.237500000000001]
 ARG[11]:y(522) DERIVED=float:CLOSED[32.737500000000004, 32.737500000000004]
 ARG[12]:z(523) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[13]:qx(524) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[14]:qy(525) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[15]:qz(526) DERIVED=float:CLOSED[0.027562365153114, 0.027562365153114]
 ARG[16]:qw(527) DERIVED=float:CLOSED[0.999620085846201, 0.999620085846201]
  */

  // Make sure it goes far enough across a door
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(12.7138);
  current_y.specify(22.456);
  target_x.specify(19.2375);
  target_y.specify(32.7375);
  ASSERT_TRUE(ce->propagate());
  ASSERT_TRUE(thru_doorway.lastDomain().getSingletonValue() == false);

  /*
   * Set values based on the following:
   ARG[0]:x(15337) DERIVED=float:CLOSED[13.949999999999999, 13.949999999999999]
   ARG[1]:y(15338) DERIVED=float:CLOSED[20.512499999999999, 20.512499999999999]
   ARG[2]:z(15339) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
   ARG[3]:qx(15340) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
   ARG[4]:qy(15341) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
   ARG[5]:qz(15342) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
   ARG[6]:qw(15343) DERIVED=float:CLOSED[1.000000000000000, 1.000000000000000]
   ARG[7]:thru_doorway(15351) DERIVED=bool:CLOSED[1, 1]
   ARG[8]:x(14927) (S)  DERIVED=float:CLOSED[13.949999999999999, 13.949999999999999]
   ARG[9]:y(14928) (S)  DERIVED=float:CLOSED[20.512499999999999, 20.512499999999999]
   ARG[10]:x(591) (S)  DERIVED=float:CLOSED[12.687500000000000, 12.687500000000000]
   ARG[11]:y(592) (S)  DERIVED=float:CLOSED[19.937500000000000, 19.937500000000000]
  */

  // Getting into brians office.
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(13.9499);
  current_y.specify(20.5124);
  target_x.specify(12.6875);
  target_y.specify(19.9375);
  ASSERT_TRUE(ce->propagate());
  ASSERT_TRUE(next_x.lastDomain().isSingleton());
  ASSERT_TRUE(next_y.lastDomain().isSingleton());
  ASSERT_TRUE(next_z.lastDomain().isSingleton());
  ASSERT_TRUE(next_qx.lastDomain().isSingleton());
  ASSERT_TRUE(next_qy.lastDomain().isSingleton());
  ASSERT_TRUE(next_qz.lastDomain().isSingleton());
  ASSERT_TRUE(next_qw.lastDomain().isSingleton());
  ASSERT_TRUE(thru_doorway.lastDomain().isSingleton());

  // Verify that the resulting point is not the same as the current point
  ASSERT_FALSE(current_x.lastDomain().intersects(next_x.lastDomain()) && current_y.lastDomain().intersects(next_y.lastDomain()));

  // Reset and try regression test case for source and target in the same region
  /*
    ARG[8]:x(40464) (S)  DERIVED=float:CLOSED[13.358695623295763, 13.358695623295763]
    ARG[9]:y(40465) (S)  DERIVED=float:CLOSED[20.482971797587421, 20.482971797587421]
    ARG[10]:x(591) (S)  DERIVED=float:CLOSED[12.687500000000000, 12.687500000000000]
    ARG[11]:y(592) (S)  DERIVED=float:CLOSED[19.937500000000000, 19.937500000000000]
  */
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(13.3586);
  current_y.specify(20.4829);
  target_x.specify(12.6875);
  target_y.specify(19.9375);
  ASSERT_TRUE(ce->propagate());

  /* Going to an outlet
    [map:get_next_move]BEFORE: [1242843828.751974]map_get_next_move(100878)
    ARG[0]:x(100861) DERIVED=float:CLOSED[-inf, +inf]
    ARG[1]:y(100862) DERIVED=float:CLOSED[-inf, +inf]
    ARG[2]:z(100863) DERIVED=float:CLOSED[-inf, +inf]
    ARG[3]:qx(100864) DERIVED=float:CLOSED[-inf, +inf]
    ARG[4]:qy(100865) DERIVED=float:CLOSED[-inf, +inf]
    ARG[5]:qz(100866) DERIVED=float:CLOSED[-inf, +inf]
    ARG[6]:qw(100867) DERIVED=float:CLOSED[-inf, +inf]
    ARG[7]:thru_doorway(100876) DERIVED=bool:CLOSED[0, 1]
    ARG[8]:x(75624) (S)  DERIVED=float:CLOSED[13.574999999999999, 13.574999999999999]
    ARG[9]:y(75625) (S)  DERIVED=float:CLOSED[21.887499999999999, 21.887499999999999]
    ARG[10]:x(523) (S)  DERIVED=float:CLOSED[19.237500000000001, 19.237500000000001]
    ARG[11]:y(524) (S)  DERIVED=float:CLOSED[32.737500000000004, 32.737500000000004]
  */
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(13.5749);
  current_y.specify(21.8874);
  target_x.specify(19.2375);
  target_y.specify(32.7375);
  ASSERT_TRUE(ce->propagate());
  /*
    ARG[0]:x(21065) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[1]:y(21066) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[2]:z(21067) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[3]:qx(21068) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[4]:qy(21069) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[5]:qz(21070) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[6]:qw(21071) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
    ARG[7]:thru_doorway(21080) DERIVED=bool:CLOSED[1, 1]
    ARG[8]:x(17397) (S)  DERIVED=float:CLOSED[19.112500000000001, 19.112500000000001]
    ARG[9]:y(17398) (S)  DERIVED=float:CLOSED[29.162500000000001, 29.162500000000001]
    ARG[10]:x(523) (S)  DERIVED=float:CLOSED[19.237500000000001, 19.237500000000001]
    ARG[11]:y(524) (S)  DERIVED=float:CLOSED[32.737500000000004, 32.737500000000004]
   */
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(19.1125);
  current_y.specify(29.1625);
  target_x.specify(19.2375);
  target_y.specify(32.7375);
  ASSERT_TRUE(ce->propagate());
  ASSERT_FALSE(next_x.lastDomain().isMember(0));

  /*
[map:get_next_move]AFTER: [1242859025.327424]map_get_next_move(12244)
 ARG[0]:x(12227) DERIVED=float:CLOSED[12.699999999999999, 12.699999999999999]
 ARG[1]:y(12228) DERIVED=float:CLOSED[22.500000000000000, 22.500000000000000]
 ARG[2]:z(12229) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[3]:qx(12230) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[4]:qy(12231) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[5]:qz(12232) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[6]:qw(12233) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[7]:thru_doorway(12242) DERIVED=bool:CLOSED[1, 1]
 ARG[8]:x(11980) (S)  DERIVED=float:CLOSED[14.412500000000001, 14.412500000000001]
 ARG[9]:y(11981) (S)  DERIVED=float:CLOSED[17.537500000000001, 17.537500000000001]
 ARG[10]:x(480) (S)  DERIVED=float:CLOSED[12.699999999999999, 12.699999999999999]
 ARG[11]:y(481) (S)  DERIVED=float:CLOSED[22.500000000000000, 22.500000000000000]

   */
  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(14.4125);
  current_y.specify(17.5375);
  target_x.specify(12.6999);
  target_y.specify(22.5000);
  ASSERT_TRUE(ce->propagate());
  ASSERT_FALSE(next_x.lastDomain().isMember(12.6999));

  /*
[map:get_next_move]AFTER: [1242860675.581439]map_get_next_move(14486)
 ARG[0]:x(14469) DERIVED=float:CLOSED[14.462500000000000, 14.462500000000000]
 ARG[1]:y(14470) DERIVED=float:CLOSED[18.149999999999999, 18.149999999999999]
 ARG[2]:z(14471) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[3]:qx(14472) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[4]:qy(14473) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[5]:qz(14474) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[6]:qw(14475) DERIVED=float:CLOSED[0.000000000000000, 0.000000000000000]
 ARG[7]:thru_doorway(14484) DERIVED=bool:CLOSED[1, 1]
 ARG[8]:x(14221) (S)  DERIVED=float:CLOSED[14.487500000000001, 14.487500000000001]
 ARG[9]:y(14222) (S)  DERIVED=float:CLOSED[18.550000000000001, 18.550000000000001]
 ARG[10]:x(480) (S)  DERIVED=float:CLOSED[12.699999999999999, 12.699999999999999]
 ARG[11]:y(481) (S)  DERIVED=float:CLOSED[22.500000000000000, 22.500000000000000]
   */

  current_x.reset();
  current_y.reset();
  target_x.reset();
  target_y.reset();
  current_x.specify(14.4875);
  current_y.specify(18.5500);
  target_x.specify(12.6999);
  target_y.specify(22.5000);
  ASSERT_TRUE(ce->propagate());
  ASSERT_TRUE(next_y.lastDomain().getSingletonValue() > current_y.lastDomain().getSingletonValue());
}

/**
 * Test the function to get an outlet approach pose
 */
TEST(executive_trex_pr2, map_get_outlet_approach_pose){
  std::ifstream is("test/willow.tmap");
  TopologicalMapAdapter map(is, "test/willow.tmap.door_overrides.xml");
  Variable<IntervalDomain> x(ce, IntervalDomain());
  Variable<IntervalDomain> y(ce, IntervalDomain());
  Variable<IntervalDomain> z(ce, IntervalDomain());
  Variable<IntervalDomain> qx(ce, IntervalDomain());
  Variable<IntervalDomain> qy(ce, IntervalDomain());
  Variable<IntervalDomain> qz(ce, IntervalDomain());
  Variable<IntervalDomain> qw(ce, IntervalDomain());
  Variable<IntervalIntDomain> outlet_id(ce, IntervalIntDomain());
  std::vector<ConstrainedVariableId> scope;
  scope.push_back(x.getId());
  scope.push_back(y.getId());
  scope.push_back(z.getId());
  scope.push_back(qx.getId());
  scope.push_back(qy.getId());
  scope.push_back(qz.getId());
  scope.push_back(qw.getId());
  scope.push_back(outlet_id.getId());

  MapGetOutletApproachPoseConstraint::MapGetOutletApproachPoseConstraint map_get_outlet_approach_pose("map_get_outlet_approach_pose", "Default", ce, scope);

  ASSERT_TRUE(ce->propagate());

  // Bind the outlet to 1 and check results
  outlet_id.specify(1);
  ASSERT_TRUE(ce->propagate());

  // Bind the outlet to 2 and check results
  outlet_id.reset();
  outlet_id.specify(2);
  ASSERT_TRUE(ce->propagate());

  // Bind the outlet to an invalid id and check results
  outlet_id.reset();
  outlet_id.specify(9999);
  ASSERT_FALSE(ce->propagate());
}

/**
 * Test reading a map in from a file
 */
TEST(executive_trex_pr2, map_read_from_file){
  std::ifstream is("test/willow.tmap");
  TopologicalMapAdapter map(is, "test/willow.tmap.door_overrides.xml");
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

  // Points that are legitimately obstacles
  ASSERT_EQ(map.isObstacle(30.0, 20.05), true);


  door_msgs::Door door_state;
  unsigned int door_region_id = TopologicalMapAdapter::instance()->getRegion(14.2,18.5);
  ASSERT_EQ(TopologicalMapAdapter::instance()->getDoorState(door_region_id, door_state), true);
}

TEST(executive_trex_pr2, map_accessor){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);

  // Generate points in the space and make sure we get a region for each point
  for(unsigned int counter = 0; counter < 10000; counter++){
    // Obtain random values for x and y
    double x, y;
    pickPointInSpace(WIDTH_24 + 5, HEIGHT_21 + 5, x, y);

    try{
      TopologicalMapAdapter::instance()->getRegion(x, y);
      bool valid = !TopologicalMapAdapter::instance()->isObstacle(x, y) && x < WIDTH_24 && y < HEIGHT_21;
      ROS_INFO_COND(!valid, "Should be on the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
    catch(...){
      bool valid = TopologicalMapAdapter::instance()->isObstacle(x, y) || x >= WIDTH_24 || y >= HEIGHT_21;
      ROS_INFO_COND(!valid, "Should be off the map but isn't with (%f, %f)", x, y);
      ASSERT_TRUE(valid);
    }
  }
}

TEST(executive_trex_pr2, map_accessor_regression_tests){
  TopologicalMapAdapter map(GRID_3_3_ALL_CONNECTED(), RESOLUTION);
  // Check a point that shoud be in a region.
  try{
    TopologicalMapAdapter::instance()->getRegion(8.5, 0.5);
  }
  catch(...){
    ROS_INFO("Should be on the map but isn't with (%f, %f)", 8.5, 0.5);
    ASSERT_TRUE(TopologicalMapAdapter::instance()->isObstacle(8.5, 0.5));
  }
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
    try{
      // Obtain random values for x and y
      double x, y;
      pickPointInSpace(WIDTH_24 + 5, HEIGHT_21 + 5, x, y);
      std::cout << "This";
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
    catch(std::runtime_error e){
      std::cout << "This";
    }
    catch(...){
      std::cout << "This";
    }
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
