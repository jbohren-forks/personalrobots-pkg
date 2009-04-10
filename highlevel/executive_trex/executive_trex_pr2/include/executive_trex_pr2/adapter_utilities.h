#ifndef H_ADAPTER_UTILITIES
#define H_ADAPTER_UTILITIES

#include "IntervalDomain.hh"
#include "StringDomain.hh"
#include "Token.hh"
#include "Observer.hh"

// Message types of interest
#include <robot_msgs/Pose.h>
#include <robot_msgs/Point32.h>
#include <std_msgs/String.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/PlugStow.h>

using namespace EUROPA;
using namespace TREX;

namespace executive_trex_pr2 {

  /**
   * @brief A class to scope a set of ros - TREX conversion utilities
   *
   * By convention, we write data from TREX to ROS and read data from ROS to TREX. This is reflected
   * in the use of read and write method qualifiers.
   */
  class AdapterUtilities {

  public:

    /**
     * @brief Extracts a frame if from a token. Will return the empty string if no
     * frame is found. Assumes the parameter name is 'frame_id'.
     */
    static std::string getFrame(const TokenId& token);

    /**
     * @brief Sets a frame in an observation. Assumes the parameter name is 'frame_id'.
     */
    static void setFrame(const std::string& frame_id, ObservationByValue& obs);

    /**
     * @brief Stuff token data into a door message
     */
    static void write(const TokenId& token, robot_msgs::Door& msg);

    /**
     * @brief Read door message into the observation
     */
    static void read(ObservationByValue& obs, const robot_msgs::Door& msg);

    /**
     * @brief Read plug stow message
     */
    static void read(ObservationByValue& obs, const robot_msgs::PlugStow& msg);

    /**
     * @brief Stuff token data into a plug stow message
     */
    static void write(const TokenId& token, robot_msgs::PlugStow& msg);

    /**
     * @brief Bind intervals to the singleton, or the domain midpoint
     */
    template <class T>
    static void write(const AbstractDomain& dom, T& target){
      double value = (dom.isSingleton() ? dom.getSingletonValue() : (dom.getLowerBound() + dom.getUpperBound()) / 2 );
      condDebugMsg(!dom.isSingleton(), "trex:warning:dispatching", "Reducing unbound paramater " << dom.toString() << " to " << value);
      target = static_cast<T>(value);
    }

    /**
     * @brief Bind named paramater of a token to value of the given type
     */
    template <class T>
    static void write(const char* param_name, const TokenId& token, T& target){
      ConstrainedVariableId var = token->getVariable(param_name);
      ROS_ASSERT(var.isValid());
      const AbstractDomain& dom = var->lastDomain();
      ROS_ASSERT(dom.isNumeric());
      double value;
      write(dom, value);
      target = value;
    }

    // Bind intervals to the singleton, or the domain midpoint
    template <class T>
    static void read(const char* name, ObservationByValue& obs, const T& source){
      obs.push_back(name, new IntervalDomain(source));
    }

    static StringDomain* toStringDomain(const std_msgs::String& msg);

    // bind a string
    static void write(const StringDomain& dom, std_msgs::String& msg);

    /**
     * @brief Reads values into an observation
     * @param obs The output observation
     * @param x The x position
     * @param y The y position
     * @param th The yaw angle
     */
    static void readPose(ObservationByValue& obs, double x, double y, double th);

    /**
     * @brief Writes values from a token
     * @param token The output observation
     * @param x The x position
     * @param y The y position
     * @param th The yaw angle
     */
    static void writePose(const TokenId& token, float& x, float& y, float& th);


    /**
     * @brief Reads values into an observation
     * @param obs The output observation
     * @param x The x position
     * @param y The y position
     * @param z The z angle
     */
    static void readPoint(ObservationByValue& obs, double x, double y, double z);

    /**
     * @brief Writes values from a token
     * @param token The output observation
     * @param x The x position
     * @param y The y position
     * @param z The z angle
     */
    static void writePoint(const TokenId& token, float& x, float& y, float& z);

    /**
     * @brief A utility function that fills out x, y, and th 
     * @param x The x position
     * @param y The y position
     * @param th The yaw angle
     */
    static void get2DPose(const robot_msgs::Pose& pose, double& x, double& y, double& th);
 
  };
}

#endif
