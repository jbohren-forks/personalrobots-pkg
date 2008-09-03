#include "CalcCommandConstraintPlayback.hh"
#include "XMLUtils.hh"
#include "Agent.hh"
#include "Playback.hh"

namespace TREX {
  CalcCommandConstraintPlayback::CalcCommandConstraintPlayback(const LabelStr& name,
					       const LabelStr& propagatorName,
					       const ConstraintEngineId& constraintEngine,
					       const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {
    m_playback = Playback::request();
  }
  
  CalcCommandConstraintPlayback::~CalcCommandConstraintPlayback() {
    m_playback->release();
  }
  
  void CalcCommandConstraintPlayback::handleExecute() {
    static unsigned int sl_cycles(0);
    sl_cycles++;
    BoolDomain& boolDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
    check_error(!boolDom.isOpen());

    
    std::vector<EUROPA::TiXmlElement*> nodes;
    m_playback->getNodes("CalcCommandConstraint", sl_cycles, nodes);
    //Iterate over all command messages.
    for (std::vector<EUROPA::TiXmlElement*>::iterator it = nodes.begin(); it != nodes.end(); it++) {
      EUROPA::TiXmlElement* node = *it;
      int value = 0; double x_v = 0, th_v = 0;
      node->Attribute("value", &value);
      node->Attribute("x_v", &x_v);
      node->Attribute("th_v", &th_v);
      boolDom.set(value);
      getCurrentDomain(m_variables[COM_VEL_X]).set(x_v);
      getCurrentDomain(m_variables[COM_VEL_TH]).set(th_v);
    }
    return;
  }
}
