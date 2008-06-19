#include "CalcCommandConstraint.hh"
#include "map_helpers.h"
#include "WavefrontPlanner.hh"

namespace EUROPA {

  CalcCommandConstraint::CalcCommandConstraint(const LabelStr& name,
					       const LabelStr& propagatorName,
					       const ConstraintEngineId& constraintEngine,
					       const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables)
  {
  }
  
  void CalcCommandConstraint::handleExecute() {
    BoolDomain& boolDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
    check_error(!boolDom.isOpen());
   //  if(boolDom.isSingleton()) {
//       check_error(boolDom.getSingletonValue() == true);
//     } else {
//       check_error(!boolDom.isSingleton());
//     }

    // BoolDomain& boolDoneDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
//     check_error(!boolDoneDom.isOpen());
//     check_error(!boolDoneDom.isSingleton());

    if(m_variables[COM_VEL_X]->lastDomain().isSingleton() &&
       m_variables[COM_VEL_TH]->lastDomain().isSingleton()) {
      return;
    }

    //testing for singletoness
    if(!m_variables[LOCAL_X]->lastDomain().isSingleton() ||
       !m_variables[LOCAL_Y]->lastDomain().isSingleton() ||
       !m_variables[LOCAL_TH]->lastDomain().isSingleton()||
       !m_variables[GOAL_X]->lastDomain().isSingleton()  ||
       !m_variables[GOAL_Y]->lastDomain().isSingleton()  ||
       !m_variables[GOAL_TH]->lastDomain().isSingleton()) {
      return;
    }

    if(!boolDom.isSingleton()) {
      //std::cout << "Bool unbound\n\n";
    }
    
  //   std::cout << "Local x " << m_variables[LOCAL_X]->lastDomain().getSingletonValue()
//  	      << " Local y " << m_variables[LOCAL_Y]->lastDomain().getSingletonValue()
//  	      << " Local th " << m_variables[LOCAL_TH]->lastDomain().getSingletonValue() << std::endl;
    
//     std::cout << "Goal x " << m_variables[GOAL_X]->lastDomain().getSingletonValue()
// 	      << " Goal y " << m_variables[GOAL_Y]->lastDomain().getSingletonValue()
// 	      << " Goal th " << m_variables[GOAL_TH]->lastDomain().getSingletonValue() << std::endl;
//     if(m_variables[COM_VEL_X]->lastDomain().isSingleton() &&
//        m_variables[COM_VEL_TH]->lastDomain().isSingleton()) {
//       std::cout << "Pre Cmd vel x " << m_variables[COM_VEL_X]->lastDomain().getSingletonValue()
// 		<< " th " << m_variables[COM_VEL_TH]->lastDomain().getSingletonValue() << std::endl;
//     }
    
    if(WavefrontPlanner::Instance()->GenerateLocalPlan(m_variables[LOCAL_X]->lastDomain().getSingletonValue(),
						       m_variables[LOCAL_Y]->lastDomain().getSingletonValue(),
						       m_variables[GOAL_X]->lastDomain().getSingletonValue(),  
						       m_variables[GOAL_Y]->lastDomain().getSingletonValue()) < 0) {
      boolDom.remove(true);
      check_error(boolDom.isSingleton());
      getCurrentDomain(m_variables[COM_VEL_X]).intersect(0.0,0.0);
      getCurrentDomain(m_variables[COM_VEL_TH]).intersect(0.0,0.0);
      return;
    }

    double vx, va;
    
    if(WavefrontPlanner::Instance()->DetermineDiffDriveCmds(vx, va,
							    m_variables[LOCAL_X]->lastDomain().getSingletonValue(), 
							    m_variables[LOCAL_Y]->lastDomain().getSingletonValue(), 
							    m_variables[LOCAL_TH]->lastDomain().getSingletonValue(),
							    m_variables[GOAL_X]->lastDomain().getSingletonValue(),  
							    m_variables[GOAL_Y]->lastDomain().getSingletonValue(), 
							    m_variables[GOAL_TH]->lastDomain().getSingletonValue()) < 0) {
      boolDom.remove(true);
      check_error(boolDom.isSingleton());
      getCurrentDomain(m_variables[COM_VEL_X]).intersect(0.0,0.0);
      getCurrentDomain(m_variables[COM_VEL_TH]).intersect(0.0,0.0);
      return;  
    }

    boolDom.remove(false);
    check_error(boolDom.isSingleton());
    check_error(boolDom.getSingletonValue() == true);
    //std::cout << "BoolDom up " << boolDom.getUpperBound() << std::endl;
    //std::cout << "BoolDom low " << boolDom.getLowerBound() << std::endl;
    if(m_variables[COM_VEL_X]->lastDomain().isSingleton() &&
       m_variables[COM_VEL_TH]->lastDomain().isSingleton()) {
      //std::cout << "cur_vel_x " << m_variables[COM_VEL_X]->lastDomain().getSingletonValue() 
      //		<< " cur_vel_th " << m_variables[COM_VEL_TH]->lastDomain().getSingletonValue() << std::endl ; 
    }
    //std::cout << "vx " << vx << " va " << va << std::endl << std::endl;
    getCurrentDomain(m_variables[COM_VEL_X]).intersect(vx,vx);
    getCurrentDomain(m_variables[COM_VEL_TH]).intersect(va,va);
    //std::cout << "BoolDom up " << boolDom.getUpperBound() << std::endl;
    //std::cout << "BoolDom low " << boolDom.getLowerBound() << std::endl;
  //   std::cout << "Local x " << m_variables[LOCAL_X]->lastDomain().getSingletonValue()
// 	      << " Local y " << m_variables[LOCAL_Y]->lastDomain().getSingletonValue()
// 	      << " Local th " << m_variables[LOCAL_TH]->lastDomain().getSingletonValue() << std::endl;

//     std::cout << "Goal x " << m_variables[GOAL_X]->lastDomain().getSingletonValue()
// 	      << " Goal y " << m_variables[GOAL_Y]->lastDomain().getSingletonValue()
// 	      << " Goal th " << m_variables[GOAL_TH]->lastDomain().getSingletonValue() << std::endl;

//     std::cout << "Cmd vel x " << m_variables[COM_VEL_X]->lastDomain().getSingletonValue()
// 	      << " th " << m_variables[COM_VEL_TH]->lastDomain().getSingletonValue() << std::endl;

    return;
  }
}
