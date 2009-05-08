
#include "Adapter.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Domains.hh"
#include "executive_trex_pr2/executive.hh"
#include <vector>
#include <executive_trex_pr2/TrexMonitor.h>

namespace TREX {

  class Monitor : public Adapter {
  public:
    Monitor(const LabelStr& agentName, const TiXmlElement& configData)
      : Adapter(agentName, configData, 1, 0, 1), m_topic_name("") {
      m_topic_name = extractData(configData, "topicName").c_str();

      m_node = Executive::request();
      std::vector<LabelStr> timelines;
      for (TiXmlElement * child = configData.FirstChildElement();
           child != NULL;
           child = child->NextSiblingElement()) {
	if(strcmp(child->Value(), "MoniterTimeline") == 0) {
	  LabelStr name = extractData(*child, "name");
	 m_timelines.push_back(name);
	}
      }


      if (m_topic_name == "") {
	m_topic_name = "/trex/debug";
	ROS_ERROR("No topic for monitor, setting default: %s.", m_topic_name.c_str());
      }
      ros::Node::instance()->advertise<executive_trex_pr2::TrexMonitor>(m_topic_name, 10);
    }

    ~Monitor() { 
      
    }

    void notify(Observation const &obs) {
      executive_trex_pr2::TrexMonitor msg;
      msg.timeline = obs.getObjectName().c_str();
      msg.set_variable_names_size(obs.countParameters());
      msg.set_variable_values_size(obs.countParameters());
      msg.set_variable_types_size(obs.countParameters());
      for (unsigned int i = 0; i < obs.countParameters(); i++) {
	msg.variable_names[i] = obs[i].first.c_str();
	msg.variable_types[i] = msg.TYPE_UNKNOWN;
	if (obs[i].second->isSingleton()) {
	  msg.variable_values[i] = domain_val_to_str(*obs[i].second, obs[i].second->getSingletonValue());
	  msg.variable_types[i] = msg.TYPE_SINGLETON;
	} else if (obs[i].second->isInterval()) {
	  msg.variable_values[i] = domain_val_to_str(*obs[i].second, obs[i].second->getLowerBound())
	    + "," + domain_val_to_str(*obs[i].second, obs[i].second->getUpperBound());
	  msg.variable_types[i] = msg.TYPE_INTERVAL; 
	} else if (obs[i].second->isEnumerated()) {
	  std::list<double> values;
	  obs[i].second->getValues(values);
	  std::list<double>::const_iterator it = values.begin(), endi = values.end();
	  msg.variable_values[i] = "{";
	  for( ; endi!=it; ++it )
	    msg.variable_values[i] += domain_val_to_str(*obs[i].second, *it);
	  msg.variable_values[i] += "}";		      
	  msg.variable_types[i] = msg.TYPE_ENUMERATED; 
	} else {
	  msg.variable_names[i] = std::string("Unknown domain:") + std::string(obs[i].second->toString().c_str());
	  msg.variable_types[i] = msg.TYPE_UNKNOWN;
	}
      }
      ros::Node::instance()->publish(m_topic_name, msg);
    }

    void queryTimelineModes(std::list<LabelStr>& externals, std::list<LabelStr>& internals){
      externals.assign(m_timelines.begin(), m_timelines.end());
    }

  private:
    ExecutiveId m_node; /*! The node. */
    std::vector<LabelStr> m_timelines; /*! The list of monitered timelines. */
    std::string m_topic_name;
  };


  // Allocate a Factory
  TeleoReactor::ConcreteFactory<Monitor> l_Monitor_Factory("Monitor");
}












