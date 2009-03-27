#include "ROSStateAdapter.hh"
#include "IntervalDomain.hh"
#include <deprecated_msgs/RobotBase2DOdom.h>

namespace TREX {
  namespace deprecated {
    class BaseStateAdapter: public ROSStateAdapter<deprecated_msgs::RobotBase2DOdom> {
    public:
      BaseStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
	: ROSStateAdapter<deprecated_msgs::RobotBase2DOdom> ( agentName, configData) {
      }

      virtual ~BaseStateAdapter(){}


    private:
      void fillObservationParameters(ObservationByValue* obs){
	// Get the 2D Pose
	double x, y, th;
	get2DPose(x, y, th);
	obs->push_back("x", new IntervalDomain(x));
	obs->push_back("y", new IntervalDomain(y));
	obs->push_back("th",new IntervalDomain(th));
      }
    };

    // Allocate a Factory
    TeleoReactor::ConcreteFactory<BaseStateAdapter> l_BaseStateAdapter_Factory("BaseStateAdapter");
  }
}
