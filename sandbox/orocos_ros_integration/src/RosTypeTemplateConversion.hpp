#ifndef ROS_TYPE_TEMPLATE_CONVERSION_HPP
#define ROS_TYPE_TEMPLATE_CONVERSION_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <rtt/Ports.hpp>

using namespace std_msgs;

namespace RTT
{
  template <typename T_ros,typename Type>
  struct RosTypeConversion{
    typedef T_ros RosType;
    typedef boost::shared_ptr<T_ros> RosTypePtr;
    typedef boost::shared_ptr<T_ros const> RosTypeConstPtr;
  
    static bool copyRosToOrocos(const RosType& ros,Type& orocos){
      log(Error)<<"Failing conversion of RosType to type "<<detail::DataSourceTypeInfo<Type>::getType()<<"."<<endlog();
      return false;
    }

    static bool copyOrocosToRos(const Type& orocos,RosType& ros){
      log(Error)<<"Failed conversion of type "<<detail::DataSourceTypeInfo<Type>::getType()<<"to RosType."<<endlog();
      return false;
    };

  };
  
  template <typename T>
  struct StdRosTypeConversion: public RosTypeConversion<T,T>{};
  
  
  class RosReadPort{
  public:
    ros::Subscriber sub_;
  };
  
  template <typename T>
  class TemplateRosReadPort : public RosReadPort {
  private:
    T raw_;
    RTT::ValueDataSource<T> data_;
    PortInterface* port_;
    
  public:
    static RosReadPort* createRosReadPort(ros::NodeHandle* node,RTT::PortInterface* port){
      return new TemplateRosReadPort(node,port);
    };
  private:
    TemplateRosReadPort(ros::NodeHandle* node,RTT::PortInterface* port):
      port_(port)
    {
      log(Debug)<<"subscribing to topic "<<"~"<<port->getName()<<endlog();
      sub_=node->subscribe<typename StdRosTypeConversion<T>::RosType>("~"+port->getName(),1,boost::bind(&TemplateRosReadPort<T>::copyData,this,_1));

    };
 
    void copyData(typename StdRosTypeConversion<T>::RosTypeConstPtr msg){
      log(Debug)<<"Copying data for topic "<<port_->getName()<<endlog();
      log(Debug)<<"Doing conversion"<<endlog();
      StdRosTypeConversion<T>::copyRosToOrocos(msg,raw_);
      log(Debug)<<"Set raw data in DataSource"<<endlog();
      data_.set(raw_);
      log(Debug)<<"Update port connection with datasource"<<endlog();
      port_->connection()->getDataSource()->update(data_.clone());
      log(Debug)<<"Port updated"<<endlog();
    };
  };

  class RosWritePort{
  public:
    ros::Publisher pub_;
    RTT::Handle event_handler_;
  };
  
  template <typename T>
  class TemplateRosWritePort : public RosWritePort {
  private:
    RTT::ValueDataSource<T> data_;
    typename StdRosTypeConversion<T>::RosType msg_;
    typename RTT::DataPort<T>::NewDataOnPortEvent* event_;

  public:
    static RosWritePort* createRosWritePort(ros::NodeHandle* node,RTT::PortInterface* port,RTT::TaskContext* tc){
      return new TemplateRosWritePort(node,port,tc);
    }
  private:
    TemplateRosWritePort(ros::NodeHandle* node,RTT::PortInterface* port,RTT::TaskContext* tc){
      log(Debug)<<"advertising "<<"~"<<port->getName()<<endlog();
      pub_ = node->advertise<typename StdRosTypeConversion<T>::RosType>("~"+port->getName(),1);
      event_ = port->getNewDataOnPortEvent();
      event_handler_ = event_->connect(boost::bind(&TemplateRosWritePort<T>::publishCallback,this,_1),tc->engine()->events());
    };

    void publishCallback(RTT::PortInterface* port){
      log(Debug)<<"publishing on topic "<<"~"<<port->getName()<<endlog();
      //Copy the data from the Orocos DataType to the Ros message
      data_.update(port->connection()->getDataSource().get());
      StdRosTypeConversion<T>::copyOrocosToRos(data_.get(),msg_);
      //Publish the message
      pub_.publish(msg_);
    };
  };

  class RosPortCreator{
  private:
    static RosPortCreator* instance;
    typedef std::map<std::string,boost::function<RosWritePort*(ros::NodeHandle*,RTT::PortInterface*,RTT::TaskContext*)> > WritePortCreators;
    WritePortCreators write_port_creators_;
    typedef std::map<std::string,boost::function<RosReadPort*(ros::NodeHandle*,RTT::PortInterface*)> > ReadPortCreators;
    ReadPortCreators read_port_creators_;
    
  public:
    RosWritePort* createWritePort(ros::NodeHandle* node,RTT::PortInterface* port,RTT::TaskContext* tc){
      const std::string& type_name = port->getTypeInfo()->getTypeName();
      WritePortCreators::iterator it=write_port_creators_.find(type_name);
      if(it!=write_port_creators_.end())
	return it->second(node,port,tc); 
      else{
	log(Error)<<"Could not find a RosWritePortCreator for type: "<<type_name<<endlog();
	return 0;
      }
    };
    RosReadPort* createReadPort(ros::NodeHandle* node,RTT::PortInterface* port){
      const std::string& type_name = port->getTypeInfo()->getTypeName();
      ReadPortCreators::iterator it=read_port_creators_.find(type_name);
      if(it!=read_port_creators_.end())
	return it->second(node,port); 
      else{
	log(Error)<<"Could not find a RosReadPortCreator for type: "<<type_name<<endlog();
	return 0;
      }
    };
    
    template <class T>
    bool registerType(const std::string& type_name){
      std::pair<WritePortCreators::iterator,bool> result1 = RosPortCreator::Instance()->write_port_creators_.insert(WritePortCreators::value_type(type_name,&TemplateRosWritePort<T>::createRosWritePort));
      std::pair<ReadPortCreators::iterator,bool> result2 = RosPortCreator::Instance()->read_port_creators_.insert(ReadPortCreators::value_type(type_name,&TemplateRosReadPort<T>::createRosReadPort));
      return result1.second&&result2.second;
    };
    
    static RosPortCreator* Instance(){
      if(instance==0)
	instance=new RosPortCreator();
      return instance;
    };
    RosPortCreator(){};
    ~RosPortCreator(){
      if(instance!=0)
	delete instance;
    };
  };
}
#endif
