#ifndef H_PR2_ADAPTER_UTILITIES
#define H_PR2_ADAPTER_UTILITIES

#include "trex_ros/adapter_utilities.h"
#include <plugs_msgs/PlugStow.h>

using namespace EUROPA;
using namespace TREX;
using namespace trex_ros;
namespace trex_pr2 {

  /**
   * @brief A class to scope a set of ros - TREX conversion utilities
   *
   * By convention, we write data from TREX to ROS and read data from ROS to TREX. This is reflected
   * in the use of read and write method qualifiers.
   */
  class Pr2AdapterUtilities : AdapterUtilities {
  public:
    /**
     * @brief Read plug stow message
     */
    static void read(ObservationByValue& obs, const plugs_msgs::PlugStow& msg);
    
    /**
     * @brief Stuff token data into a plug stow message
     */
    static void write(const TokenId& token, plugs_msgs::PlugStow& msg);
  };

}

#endif
