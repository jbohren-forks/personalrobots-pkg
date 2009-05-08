#include "executive_trex_pr2/calc_angle_diff_constraint.h"
#include "Logger.hh"
#include <math.h>

#define ANG_NORM(a) atan2(sin((a)),cos((a)))

namespace TREX {
  CalcAngleDiffConstraint::CalcAngleDiffConstraint(const LabelStr& name,
						   const LabelStr& propagatorName,
						   const ConstraintEngineId& constraintEngine,
						   const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables),
      m_diff(getCurrentDomain(variables[ANGLE_DIFF])),
      m_angle1(getCurrentDomain(variables[ANGLE_1])),
      m_angle2(getCurrentDomain(variables[ANGLE_2]))
  {
    m_logger = TREX::Logger::request();
  }

  CalcAngleDiffConstraint::~CalcAngleDiffConstraint() {
    m_logger->release();
  }
  
  void CalcAngleDiffConstraint::handleExecute() {
  
    if(!m_angle1.areBoundsFinite() ||
       !m_angle2.areBoundsFinite()) {
      return;
    }

    if(!m_angle1.isSingleton() ||
       !m_angle2.isSingleton()) {
      return;
    }

    double d1, d2;
    double d;
    
    double ta = m_angle1.getSingletonValue();
    double tb = m_angle2.getSingletonValue();
    
    ta = ANG_NORM(ta);
    tb = ANG_NORM(tb);
    d1 = ta-tb;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
      d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
      d = d1;
    else
      d = d2;

    m_diff.set(d);

    debugMsg("CalcAngleDiffConstraint:handleExecute", "Angle 1 " << ta << " Angle 2 " << tb << " computed diff " << 
	     d);

    //FILE* log = m_logger->getFile();
    //if (log) {
    //  fprintf(log, "\t<CalcAngleDiffConstraint time=\"%u\" value=\"%d\"/>\n", sl_cycles, result);
    //}
  }
}
