/**
 * Just a place holder for code written and removed. Could be handy later.
 */
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



  void ROSAdapter::checkTFEnabled(){
    if(!tf_enabled){
      ROS_ERROR("Attempted to call a transform for an adapater that is not transform enabled. Check your TREX input configuration for the adapter %s and ensure the 'frame_id' parameter is set",
		getName().c_str());
    }
  }

  bool ROSAdapter::canTransform(const std::string& source_frame_id){
    static const ros::Duration timeout(ros::Duration().fromSec(1.0));

      // The message must have a frame id set
    if(!tf.canTransform(frame_id, source_frame_id, ros::Time::now(), timeout)){
      debugMsg("ros:error:synchronization", nameString() << "No transfrom available from " << source_frame_id << " to " << frame_id); 
      return false;
    }

    return true;
  }

  bool ROSAdapter::hasFrameParam(const TiXmlElement& configData){
    return configData.Attribute("frame_id") != NULL;
  }

/* Initialization from config file
    // If there is a frame specified, then we will use its value and mark the adapter as transform enabled
    const char* frame_param = configData.Attribute("frame_id");
    if(frame_param == NULL){
      frame_id = "NOT APPLICABLE";
    }
    else {
      frame_id = frame_param;
      TREX_INFO("ros:info", "Storing data for " << timelineName.c_str() << " in the " << frame_id << " frame.");
    }
*/

/*
      // If tf is enabled, and we are dispatching a request or a recall, then we should update the frame
      // we want passed on the goal
      if(tf_enabled){
	ConstrainedVariableId frame_var = goal->getVariable("frame_id");
	ROS_ASSERT(frame_var.isId() && frame_var.isValid());
	// If the frame parameter is open, or a singleton, then close it by restricting the base domain
	// to the closed string domain given by the current domain
	if(frame_var->lastDomain().isOpen() || !frame_var->lastDomain().isSingleton()){
	  StringDomain dom;
	  dom.insert(frame_id);
	  dom.close();
	  frame_var->restrictBaseDomain(dom);
	}
	else{
	  LabelStr lblStr = frame_var->lastDomain().getSingletonValue();
	  frame_id = lblStr.toString();
	}
      }
*/


    static bool hasFrameParam(const TiXmlElement& configData);
