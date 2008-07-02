#include "CalcCommandConstraint.hh"
#include "map_helpers.h"
#include "WavefrontPlanner.hh"
#include "Logger.hh"

namespace TREX {
  CalcCommandConstraint::CalcCommandConstraint(const LabelStr& name,
					       const LabelStr& propagatorName,
					       const ConstraintEngineId& constraintEngine,
					       const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {
    m_logger = TREX::Logger::request();
  }
  CalcCommandConstraint::~CalcCommandConstraint() {
    m_logger->release();
  }

  void CalcCommandConstraint::logData(const unsigned int cycles, const bool bd, 
				      const double x_value, const double th_value) {
    FILE* log = m_logger->getFile();
    if (log) {
      fprintf(log, "\t<CalcCommandConstraint time=\"%u\" value=\"%d\" x_v=\"%f\" th_v=\"%f\"/>\n",
	      cycles, bd, x_value, th_value);
    }
  }
  
  void CalcCommandConstraint::handleExecute() {
    static unsigned int sl_cycles(0);
    sl_cycles++;
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
      //debugMsg("CalcCommandConstraint::handleExecute", "Bool unbound");
    }
    
  //   debugMsg("CalcCommandConstraint::handleExecute", "Local x "
//             << m_variables[LOCAL_X]->lastDomain().getSingletonValue()
//  	      << " Local y " << m_variables[LOCAL_Y]->lastDomain().getSingletonValue()
//  	      << " Local th " << m_variables[LOCAL_TH]->lastDomain().getSingletonValue());
    
//     debugMsg("CalcCommandConstraint::handleExecute", "Goal x "
//             << m_variables[GOAL_X]->lastDomain().getSingletonValue()
// 	      << " Goal y " << m_variables[GOAL_Y]->lastDomain().getSingletonValue()
// 	      << " Goal th " << m_variables[GOAL_TH]->lastDomain().getSingletonValue());
//     if(m_variables[COM_VEL_X]->lastDomain().isSingleton() &&
//        m_variables[COM_VEL_TH]->lastDomain().isSingleton()) {
//         debugMsg("CalcCommandConstraint::handleExecute", "Pre Cmd vel x " << m_variables[COM_VEL_X]->lastDomain().getSingletonValue()
// 		<< " th " << m_variables[COM_VEL_TH]->lastDomain().getSingletonValue());
//     }
    
    if(WavefrontPlanner::Instance()->GenerateLocalPlan(m_variables[LOCAL_X]->lastDomain().getSingletonValue(),
						       m_variables[LOCAL_Y]->lastDomain().getSingletonValue(),
						       m_variables[GOAL_X]->lastDomain().getSingletonValue(),  
						       m_variables[GOAL_Y]->lastDomain().getSingletonValue()) < 0) {
      boolDom.set(false);
      getCurrentDomain(m_variables[COM_VEL_X]).set(0.0);
      getCurrentDomain(m_variables[COM_VEL_TH]).set(0.0);
      logData(sl_cycles, false, 0.0, 0.0);
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
      boolDom.set(false);
      check_error(boolDom.isSingleton());
      getCurrentDomain(m_variables[COM_VEL_X]).set(0.0);
      getCurrentDomain(m_variables[COM_VEL_TH]).set(0.0);
      logData(sl_cycles, false, 0.0, 0.0);
      return;  
    }

    boolDom.set(true);
    check_error(boolDom.isSingleton());
    check_error(boolDom.getSingletonValue() == true);
    //debugMsg("CalcCommandConstraint::handleExecute", "BoolDom up " << boolDom.getUpperBound());
    //debugMsg("CalcCommandConstraint::handleExecute", "BoolDom low " << boolDom.getLowerBound());
    if(m_variables[COM_VEL_X]->lastDomain().isSingleton() &&
       m_variables[COM_VEL_TH]->lastDomain().isSingleton()) {
      //debugMsg("CalcCommandConstraint::handleExecute", "cur_vel_x " << m_variables[COM_VEL_X]->lastDomain().getSingletonValue() 
      //		<< " cur_vel_th " << m_variables[COM_VEL_TH]->lastDomain().getSingletonValue()); 
    }
    //debugMsg("CalcCommandConstraint::handleExecute", "vx " << vx << " va " << va);
    getCurrentDomain(m_variables[COM_VEL_X]).set(vx);
    getCurrentDomain(m_variables[COM_VEL_TH]).set(va);


    logData(sl_cycles, true, vx, va);

    //debugMsg("CalcCommandConstraint::handleExecute", "BoolDom up " << boolDom.getUpperBound());
    //debugMsg("CalcCommandConstraint::handleExecute", "BoolDom low " << boolDom.getLowerBound());
  // debugMsg("CalcCommandConstraint::handleExecute", "Local x " << m_variables[LOCAL_X]->lastDomain().getSingletonValue()
// 	      << " Local y " << m_variables[LOCAL_Y]->lastDomain().getSingletonValue()
// 	      << " Local th " << m_variables[LOCAL_TH]->lastDomain().getSingletonValue());

//    debugMsg("CalcCommandConstraint::handleExecute", "Goal x " << m_variables[GOAL_X]->lastDomain().getSingletonValue()
// 	      << " Goal y " << m_variables[GOAL_Y]->lastDomain().getSingletonValue()
// 	      << " Goal th " << m_variables[GOAL_TH]->lastDomain().getSingletonValue());

//     debugMsg("CalcCommandConstraint::handleExecute", "Cmd vel x " << m_variables[COM_VEL_X]->lastDomain().getSingletonValue()
// 	      << " th " << m_variables[COM_VEL_TH]->lastDomain().getSingletonValue());

    return;
  }
}
