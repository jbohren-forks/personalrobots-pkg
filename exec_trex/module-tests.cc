
#include "Nddl.hh"
#include "ExecDefs.hh"
#include "Logger.hh"
#include "Agent.hh"
#include "TestSupport.hh"
#include "CalcDistanceConstraint.hh"

using namespace TREX;
using namespace EUROPA;

class ConstraintTests {
public:
  static bool test(){
    runTest(testCalcDistanceConstraints);
    return true;
  }

private:
  static bool testCalcDistanceConstraints(){
    Variable<IntervalDomain> target(ENGINE,IntervalDomain());
    Variable<IntervalDomain> x1(ENGINE,IntervalDomain());
    Variable<IntervalDomain> y1(ENGINE,IntervalDomain());
    Variable<IntervalDomain> z1(ENGINE,IntervalDomain());
    Variable<IntervalDomain> x2(ENGINE,IntervalDomain());
    Variable<IntervalDomain> y2(ENGINE,IntervalDomain());
    Variable<IntervalDomain> z2(ENGINE,IntervalDomain());

    std::vector<ConstrainedVariableId> scope;
    scope.push_back(target.getId());
    scope.push_back(x1.getId());
    scope.push_back(y1.getId());
    scope.push_back(z1.getId());
    scope.push_back(x2.getId());
    scope.push_back(y2.getId());
    scope.push_back(z2.getId());


    TREX::CalcDistanceConstraint c(LabelStr("calcDistance"),LabelStr("Default"),ENGINE, scope);

    assertTrue(ENGINE->propagate());
    assertTrue(!target.derivedDomain().isSingleton(), "Should not be propagated");

    x1.specify(10);
    y1.specify(10);
    z1.specify(10);
    x2.specify(10);
    y2.specify(10);
    z2.specify(10);

    assertTrue(target.derivedDomain().isSingleton() && target.derivedDomain().getSingletonValue() == 0.0, target.derivedDomain().toString());

    z2.reset();
    z2.specify(100);
    assertTrue(target.derivedDomain().isSingleton() && target.derivedDomain().getSingletonValue() == 90.0, target.derivedDomain().toString());

    return true;
  }

};

int main() {
  initTREX();
  runTestSuite(ConstraintTests::test);
  return 0;
}
