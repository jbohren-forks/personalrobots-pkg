#include "ROSStateAdapter.hh"
#include "IntervalDomain.hh"
#include <mechanism_control/MechanismState.h>
#include <vector>
#include <rosTF/rosTF.h>

using namespace mechanism_control;
using namespace std;
using namespace libTF;

namespace TREX {

  class EndEffectorStateAdapter: public ROSStateAdapter<MechanismState> {
  public:
    EndEffectorStateAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSStateAdapter<MechanismState> (agentName, configData), tfClient(*(ros::Node *) m_node)
    {
      count = 42;
    }

    virtual ~EndEffectorStateAdapter(){}

  private:

    int count;
    rosTFClient tfClient;
    
    void fillObservationParameters(ObservationByValue* obs){

      TFPoint p, endEffectorPos;
      p.x = 0;
      p.y = 0;
      p.z = 0;
      p.frame = "forearm_roll_right";
      p.time = 0;
      try {
	endEffectorPos = tfClient.transformPoint ("base_link", p);
      }
      catch (TransformReference::LookupException& ex) {
	cout << "No transform available error\n";
      }
      catch (TransformReference::ConnectivityException& ex) {
	cout << "Connectivity error\n";
      }
      catch (TransformReference::ExtrapolateException& ex) {
	cout << "Extrapolation error\n";
      }

      cout << "End effector pos is (" << endEffectorPos.x << ", " << endEffectorPos.y << ", " << endEffectorPos.z << ")\n";
      
      

      // Debugging: just use dummy values for now
      obs->push_back("rot1_1", new IntervalDomain(count++));
      obs->push_back("rot1_2", new IntervalDomain(count++));
      obs->push_back("rot1_3", new IntervalDomain(count++));
      obs->push_back("rot2_1", new IntervalDomain(count++));
      obs->push_back("rot2_2", new IntervalDomain(count++));
      obs->push_back("rot2_3", new IntervalDomain(count++));
      obs->push_back("rot3_1", new IntervalDomain(count++));
      obs->push_back("rot3_2", new IntervalDomain(count++));
      obs->push_back("rot3_3", new IntervalDomain(count++));
      obs->push_back("x", new IntervalDomain(count++));
      obs->push_back("y", new IntervalDomain(count++));
      obs->push_back("z", new IntervalDomain(count++));
    }
  };

  // Allocate factory
  TeleoReactor::ConcreteFactory<EndEffectorStateAdapter> EndEffectorStateAdapter_Factory("EndEffectorStateAdapter");
  
}

      
