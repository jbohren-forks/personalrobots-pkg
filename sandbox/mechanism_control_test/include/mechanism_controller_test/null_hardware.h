#ifndef NULL_HARDWARE_H
#define NULL_HARDWARE_H

#include <hardware_interface/hardware_interface.h>
#include <tinyxml/tinyxml.h>

class NullHardware
{
public:
  /*!
   * \brief Constructor
   */
  NullHardware();

  /*!
   * \brief Destructor
   */
  ~NullHardware();

  void update();

  void initXml(TiXmlElement *config);

  HardwareInterface *hw_;

private:
};

#endif
