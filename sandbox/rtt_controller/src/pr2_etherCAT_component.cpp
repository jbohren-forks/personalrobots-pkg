#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>

#include <mechanism_control/mechanism_control.h>
#include <ethercat_hardware/ethercat_hardware.h>
#include <urdf/parser.h>
#include <ocl/ComponentLoader.hpp>

#include <sys/stat.h>

using namespace RTT;

class pr2_etherCAT : public TaskContext{
private:
    Property<std::string> interface,xml_file;
    Property<std::vector<std::string> > joint_names;
    ReadDataPort<std::vector<double> > joint_eff_port;
    WriteDataPort<std::vector<double> > joint_pos_port;
    WriteDataPort<std::vector<double> > joint_vel_port;
    EthercatHardware ec;
    MechanismControl* mc;
    std::vector<double> joint_pos,joint_vel,joint_eff;
    std::vector<mechanism::JointState*> joints;
    
  bool reset_motors_;
public:
    pr2_etherCAT(const std::string& name):
        TaskContext(name,PreOperational),
        interface("interface","Connect to EtherCAT devices on this interface","eth0"),
        xml_file("robot_description","Load the robot description from this file"),
        joint_names("joint_names","Names of the joints to allow control for with"),
        joint_eff_port("joint_efforts"),
        joint_pos_port("joint_positions"),
        joint_vel_port("joint_velocities")
    {
        this->properties()->addProperty(&interface);
        this->properties()->addProperty(&xml_file);
        this->properties()->addProperty(&joint_names);
        
        this->ports()->addPort(&joint_pos_port);
	this->ports()->addPort(&joint_vel_port);
        this->ports()->addPort(&joint_eff_port);
        
	this->methods()->addMethod(method("reset_motors",&pr2_etherCAT::reset_motors,this),"reset the motors");
    }
    ~pr2_etherCAT(){};
    
    bool configureHook(){
        // Initialize the hardware interface
        bool allow_unprogrammed=false;
        bool publish_motor_model=false;
        ec.init(const_cast<char *>(interface.rvalue().c_str()), allow_unprogrammed, publish_motor_model);
        
        // Load robot description
        TiXmlDocument xml;
        struct stat st;
        if (0 == stat(xml_file.rvalue().c_str(), &st)){
            xml.LoadFile(xml_file.rvalue());
        }else{
            log(Error)<<"Could not find xml document, "<< xml_file.rvalue() <<"with robot description"<<endlog();
            return false;
        }
        TiXmlElement *root_element = xml.RootElement();
        TiXmlElement *root = xml.FirstChildElement("robot");
        if (!root || !root_element){
            log(Error)<<"Could not parse the xml from "<< xml_file<<endlog();
            return false;
        }
        urdf::normalizeXml(root_element);

        // Register actuators with mechanism control
        bool allow_override=false;
        ec.initXml(root, allow_override);
        // Create mechanism control
        mc = new MechanismControl(ec.hw_);
        // Initialize mechanism control from robot description
        mc->initXml(root);

        for(unsigned int i=0;i<joint_names.rvalue().size();i++)
            joints.push_back(mc->state_->getJointState(joint_names.rvalue()[i]));
        
        joint_pos.assign(joints.size(),0.0);
        joint_vel.assign(joints.size(),0.0);
        joint_eff.assign(joints.size(),0.0);
	
	joint_pos_port.Set(joint_pos);
	joint_vel_port.Set(joint_vel);
        return true;
    }
    bool startHook(){
      reset_motors();
      return true;
    }
    void updateHook(){
      if(reset_motors_){
	ec.update(reset_motors_,false);
	reset_motors_=false;
      }	else
	ec.update(reset_motors_,false);
        //Get joint states & set joint_efforts
        joint_eff_port.Get(joint_eff);
	//Set efforts to zero first;
	mc->state_->zeroCommands();
        for(unsigned int i=0;i<joints.size();i++){
            joint_pos[i]=joints[i]->position_;
            joint_vel[i]=joints[i]->velocity_;
            joints[i]->commanded_effort_=joint_eff[i];
        }
        joint_pos_port.Set(joint_pos);
        joint_vel_port.Set(joint_vel);
        //Set joint efforts
        mc->update();
    }

    void stopHook(){
        /* Shutdown all of the motors on exit */
        for (unsigned int i = 0; i < ec.hw_->actuators_.size(); ++i){
            ec.hw_->actuators_[i]->command_.enable_ = false;
            ec.hw_->actuators_[i]->command_.effort_ = 0;
        }
        ec.update(false, true);
    }
    
    
    void cleanupHook(){
        if(!mc)
            delete mc;
    }

  void reset_motors(){
    reset_motors_=true;
  }
};
ORO_CREATE_COMPONENT(pr2_etherCAT);
