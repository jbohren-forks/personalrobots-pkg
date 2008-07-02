#include "CalcGlobalPathConstraintPlayback.hh"
#include "XMLUtils.hh"
#include "Playback.hh"
#include "Agent.hh"

namespace TREX {
  CalcGlobalPathConstraintPlayback::CalcGlobalPathConstraintPlayback(const LabelStr& name,
						     const LabelStr& propagatorName,
						     const ConstraintEngineId& constraintEngine,
						    const std::vector<ConstrainedVariableId>& variables)
    : Constraint(name, propagatorName, constraintEngine, variables) {
    m_playback = TREX::Playback::request();
  }

  CalcGlobalPathConstraintPlayback::~CalcGlobalPathConstraintPlayback() {
    m_playback->release();
  }
  
  void CalcGlobalPathConstraintPlayback::handleExecute() {
    static unsigned int sl_cycles(0);
    sl_cycles++;

    BoolDomain& boolDom = static_cast<BoolDomain&>(getCurrentDomain(m_variables[0]));
    check_error(!boolDom.isOpen());


    
    std::vector<EUROPA::TiXmlElement*> nodes;
    m_playback->getNodes("CalcGlobalPathConstraint", sl_cycles, nodes);
    //Iterate over all command messages.
    for (std::vector<EUROPA::TiXmlElement*>::iterator it = nodes.begin(); it != nodes.end(); it++) {
      EUROPA::TiXmlElement* node = *it;
      int value = 0;
      node->Attribute("value", &value);
      boolDom.set(value);
    }

  }
}
