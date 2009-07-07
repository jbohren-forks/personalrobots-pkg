#ifndef __RECONFIGURATOR_H__
#define __RECONFIGURATOR_H__

namespace dynamic_reconfigure

/**
 * Keeps track of the reconfigure callback function.
 */
class AbstractReconfigurator
{
public:
  AbstractReconfigurator() : callback_(ignore)
  {
  }
  
  set_callback(boost::function<void(int)> callback)
  {
    callback_ = callback;
  }

  clear_callback()
  {
    callback_ = ignore;
  }

private:
  ignore(int) // @todo is this necessary?
  {
  }

  boost::function<void(int)> callback_;
};

template <class ConfigManipulator>
class Reconfigurator : public AbstractReconfigurator
{
public:
  Reconfigurator(NodeHandle &nh) : node_handle_(nh)
  {
    config_ = ConfigManipulator::defaults;
    ConfigManipulator::read_from_param_server(node_handle_, config_);
    // Write to make sure everything is filled in.
    ConfigManipulator::write_to_param_server(node_handle_, config_);
    
    get_service_ = node_handle_.advertizeService("~get_configuration", &get_config_service, this)
    set_service_ = node_handle_.advertizeService("~set_configuration", &set_config_service, this)
  }

  get_config(ConfigManipulator::ConfigType &config)
  {
    config = config_;
  }

  set_config(const ConfigManipultor::ConfigType &config)
  {
    config_ = config;
    ConfigManipulator::write_to_param_server(node_handle_, config_);
  }

private:
  bool get_config_service(ConfigManipulator::GetService::Request &req, ConfigManipulator::SetService::Response &rsp)
  {
    rsp.defaults = ConfigManipulator::get_defaults();
    rsp.min = ConfigManipulator::get_min();
    rsp.max = ConfigManipulator::get_max();
    return true;
  }

  bool set_config_service(ConfigManipulator::GetService::Request &req, ConfigManipulator::SetService::Response &rsp)
  {
    int level = ConfigManipulator::get_change_level(req.config, config_);

    set_config(req.config);

    // We expect config_ to be read, and possibly written during the
    // callback.
    callback_();
    
    rsp.config = config_;
  }

  ConfigManipulator::ConfigType config_;
  NodeHandle node_handle_;
  ros::ServiceServer get_service_;
  ros::ServiceServer set_service_;
};
                                 
class ConfigManipulator
{
  read_from_param_server();
  write_to_param_server();
  get_change_level();
}

#endif
