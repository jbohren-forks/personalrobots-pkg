#include <mechanism_controller_test/null_hardware.h>

NullHardware::NullHardware(){
  hw_ = new HardwareInterface(0);
}

NullHardware::~NullHardware(){
}

void NullHardware::initXml(TiXmlElement *config){
}
