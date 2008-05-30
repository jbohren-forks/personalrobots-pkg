#include "Adapter.hh"
/**#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
*/
#include "Executive.hh"

/**
 * @author Conor McGann
 * @brief Monitors timelines of interest and publishes over ROS message bus
 */
namespace TREX {

  class Monitor: public Adapter {
  public:
    Monitor(const LabelStr& agentName, const TiXmlElement& configData)
      : Adapter(agentName, "monitor", 0, 0, "monitor.cfg"){}

    /**
     * Just reuse base class, but swap since base class assumes internals
     */
    void queryTimelineModes(std::list<LabelStr>& externals, std::list<LabelStr>& internals){
      Adapter::queryTimelineModes(internals, externals);
    }

    void handleNextTick(){}

    /**
     * @brief Post observations on ROS
     */
    void notify(const Observation& observation){
      debugMsg("Monitor:notify", "Observed: " << observation.toString());
      Executive::instance()->publish(observation);
    }

  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<Monitor> l_Monitor_Factory("Monitor");
}
