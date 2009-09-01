
#include "RosToolkit.hpp"
#include "Orocos2Ros.hpp"
#include "RosTypeConversion.hpp"
#include <rtt/Toolkit.hpp>

RTT::RosPortCreator* RTT::RosPortCreator::instance=0;

std::string RosToolkitPlugin::getName(){return "RosToolkit";}

bool RosToolkitPlugin::loadTypes(){
    RTT::RosPortCreator::Instance()->registerType<double>("double");
    RTT::RosPortCreator::Instance()->registerType<std::vector<double> >("array");
    RTT::RosPortCreator::Instance()->registerType<Float64>("Float64");
    RTT::RosPortCreator::Instance()->registerType<Float64MultiArray>("Float64MultiArray");

    RTT::types()->addType( new RTT::Float64TypeInfo());
    RTT::types()->addType( new RTT::Float64MultiArrayTypeInfo());
    return true;
}

RosToolkitPlugin RosToolkit;

ORO_TOOLKIT_PLUGIN(RosToolkit);

