#ifndef H_PR2_ADAPTER_UTILITIES
#define H_PR2_ADAPTER_UTILITIES

#include "trex_ros/adapter_utilities.h"
#include <plugs_msgs/PlugStow.h>
#include <door_msgs/Door.h>

namespace trex_temp_proj {

  /**
   * @brief A class to scope a set of ros - TREX conversion utilities
   *
   * By convention, we write data from TREX to ROS and read data from ROS to TREX. This is reflected
   * in the use of read and write method qualifiers.
   */
  class TempAdapterUtilities : trex_ros::AdapterUtilities {
  public:
    /**
     * @brief Stuff token data into a door message
     */
    static void write(const TREX::TokenId& token, door_msgs::Door& msg);

    /**
     * @brief Read door message into the observation
     */
    static void read(TREX::ObservationByValue& obs, const door_msgs::Door& msg);

    /**
     * @brief Read plug stow message
     */
    static void read(TREX::ObservationByValue& obs, const plugs_msgs::PlugStow& msg);
    
    /**
     * @brief Stuff token data into a plug stow message
     */
    static void write(const TREX::TokenId& token, plugs_msgs::PlugStow& msg);
  };

}

#endif
