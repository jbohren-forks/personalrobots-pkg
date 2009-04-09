#ifndef H_ROSAdapter
#define H_ROSAdapter

#include "Executive.hh"
#include "Adapter.hh"
#include "Debug.hh"
#include "StringDomain.hh"
#include "Token.hh"

// Message types of interest
#include <robot_msgs/Pose.h>
#include <robot_msgs/Point32.h>
#include <std_msgs/String.h>
#include <robot_msgs/Door.h>

namespace TREX {

  class ROSAdapter: public Adapter {
  public:
    ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead);
    ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, const std::string& state_topic);

    virtual ~ROSAdapter();

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

    /**
     * @brief Extracts a frame if from a token. Will return the empty string if no
     * frame is found. Assumes the parameter name is 'frame_id'.
     */
    static std::string getFrame(const TokenId& token);

    /**
     * @brief Sets a frame in an observatio. Assumes the parameter name is 'frame_id'.
     */
    static void setFrame(const std::string frame_id, ObservationByValue* obs);

    /**
     * @brief Stuff token data into a door message
     */
    static void writeTokenToDoorMessage(const TokenId& token, robot_msgs::Door& msg);

    // Bind intervals to the singleton, or the domain midpoint
    template <class T>
    static void write(const AbstractDomain& dom, T& target){
      double value = (dom.isSingleton() ? dom.getSingletonValue() : (dom.getLowerBound() + dom.getUpperBound()) / 2 );
      condDebugMsg(!dom.isSingleton(), "trex:warning:dispatching", "Reducing unbound paramater " << dom.toString() << " to " << value);
      target = static_cast<T>(value);
    }

    // Bind names paramater to value
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
  private:
    bool isInitialized() const;

    bool handleRequest(const TokenId& goal);

    void handleRecall(const TokenId& goal);

    void commonInit(const TiXmlElement& configData);

    bool m_initialized; /*!< Hook to flag if expected ros initialization messages have arrived */
    std::vector<std::string> nddlNames_;
    std::vector<std::string> rosNames_;

  protected:
    static unsigned int QUEUE_MAX(){return 10;}
    virtual void handleCallback();
    virtual void registerSubscribers() {}
    virtual void registerPublishers() {}
    virtual Observation* getObservation() = 0;
    virtual bool dispatchRequest(const TokenId& goal, bool enabled){return true;}

    const std::vector<std::string>& nddlNames() const {return nddlNames_;}
    const std::vector<std::string>& rosNames() const {return rosNames_;}

    /********************************************************
     * CORE TOKEN / ROS TYPE CONVERSIONS
     *******************************************************/
    StringDomain* toStringDomain(const std_msgs::String& msg);

    // bind a string
    void write(const StringDomain& dom, std_msgs::String& msg);

    /**
     * @brief Reads values into an observation
     * @param obs The output observation
     * @param x The x position
     * @param y The y position
     * @param th The yaw angle
     */
    void readPose(ObservationByValue& obs, double x, double y, double th);

    /**
     * @brief Writes values from a token
     * @param token The output observation
     * @param x The x position
     * @param y The y position
     * @param th The yaw angle
     */
    void writePose(const TokenId& token, float& x, float& y, float& th);


    /**
     * @brief Reads values into an observation
     * @param obs The output observation
     * @param x The x position
     * @param y The y position
     * @param z The z angle
     */
    void readPoint(ObservationByValue& obs, double x, double y, double z);

    /**
     * @brief Writes values from a token
     * @param token The output observation
     * @param x The x position
     * @param y The y position
     * @param z The z angle
     */
    void writePoint(const TokenId& token, float& x, float& y, float& z);

    /**
     * @brief A utility function that fills out x, y, and th 
     * @param x The x position
     * @param y The y position
     * @param th The yaw angle
     */
    void get2DPose(const robot_msgs::Pose& pose, double& x, double& y, double& th);
 
    /**
     * @brief Helper method to obtain an index for a given ros name.
     * @param The rosName to look for
     * @param The index to fill if found
     * @return True if found, else false
     */
    bool rosIndex(const std::string& rosName, unsigned int& ind) const;

  
  
    ExecutiveId m_node; /*! The node. */
    const std::string timelineName;
    const std::string timelineType;
    const std::string stateTopic;
  };
}
#endif
