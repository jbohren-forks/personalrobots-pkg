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

// For transform support
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>

namespace TREX {


  class ROSAdapter: public Adapter {
  public:
    ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead);
    ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, const std::string& state_topic);

    virtual ~ROSAdapter();

    void handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer);

    void handleNextTick();

    bool synchronize();

  private:
    bool isInitialized() const;

    bool handleRequest(const TokenId& goal);

    void handleRecall(const TokenId& goal);

    void commonInit(const TiXmlElement& configData);

    static bool hasFrameParam(const TiXmlElement& configData);

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

    /**
     * @brief Simple utility to wrape TF checking and message
     */
    void checkTFEnabled();

    const std::vector<std::string>& nddlNames() const {return nddlNames_;}
    const std::vector<std::string>& rosNames() const {return rosNames_;}

    /********************************************************
     * CORE TOKEN / ROS TYPE CONVERSIONS
     *******************************************************/
    StringDomain* toStringDomain(const std_msgs::String& msg);

    // bind a string
    void write(const StringDomain& dom, std_msgs::String& msg);

    // Bind intervals to the singleton, or the domain midpoint
    template <class T>
    void write(const AbstractDomain& dom, T& target){
      double value = (dom.isSingleton() ? dom.getSingletonValue() : (dom.getLowerBound() + dom.getUpperBound() / 2) );
      condDebugMsg(!dom.isSingleton(), "trex:warning:dispatching", "Reducing unbound paramater " << dom.toString() << " to " << value);
      target = static_cast<T>(value);
    }

    // Bind names paramater to value
    template <class T>
    void write(const char* param_name, const TokenId& token, T& target){
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
    void read(const char* name, ObservationByValue& obs, const T& source){
      obs.push_back(name, new IntervalDomain(source));
    }

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
    void get2DPose(double& x, double& y, double& th);

    /**
     * @brief A utility function that obtains a pose sourced in the frame given by the source frame id
     * and transformed into the adapter frame given by the member variable - frame_id of the adapter.
     * @param source_frame_id The frame used to query tf.
     * @param out The output point
     * @param in The input point
     */
    template<class T>
    void transformPoint(const std::string& source_frame_id, T& out, const T& in){
      checkTFEnabled();

      TREX_INFO("ros:debug:synchronization", nameString() << "Transforming point with source frame = " << source_frame_id);

      tf::Stamped<tf::Point> tmp;
      tmp.stamp_ = ros::Time();
      tmp.frame_id_ = source_frame_id;
      tmp[0] = in.x;
      tmp[1] = in.y;
      tmp[2] = in.z;

      // Should we be waiting here?
      try{
	tf.transformPoint(frame_id, tmp, tmp);
	out.x = tmp[0];
	out.y = tmp[1];
	out.z = tmp[2];
      }
      catch(tf::LookupException& ex) {
	TREX_INFO("ros:debug:synchronization", nameString() << "No transform available. Error: " << ex.what());
	ROS_ERROR("No Transform available Error: %s\n", ex.what());
      }
      catch(tf::ConnectivityException& ex) {
	TREX_INFO("ros:debug:synchronization", nameString() << "Connectivity Error: " << ex.what());
	ROS_ERROR("Connectivity Error: %s\n", ex.what());
      }
      catch(tf::ExtrapolationException& ex) {
	TREX_INFO("ros:debug:synchronization", nameString() << "Extrapolation Error: " << ex.what());
	ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      }

      TREX_INFO("ros:debug:synchronization", "Transformed point from [" << source_frame_id << "]" << 
		"<" << in.x << ", " << in.y << ", " << in.z << "> to [" << frame_id << "]" << 
		"<" << out.x << ", " << out.y << ", " << out.z << ">");
    }

    /**
     * @brief Utility to test if we can transform from the given frame. Call during synchronization
     * for handling error reporting if expected frames are not provided.
     */
    bool canTransform(const std::string& source_frame);

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
    const bool tf_enabled;
    std::string frame_id; /**< The frame to use for transforms, if enabled. Note that this is mutable based on task context */
    tf::TransformListener tf; /**< Used to do transforms */
  };
}
#endif
