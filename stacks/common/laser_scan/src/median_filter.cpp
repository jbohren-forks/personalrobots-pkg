#include "laser_scan/median_filter.h"

namespace laser_scan
{
LaserMedianFilter::LaserMedianFilter():
  num_ranges_(1)
{
  
};

bool LaserMedianFilter::configure()
{
  TiXmlNode * child_node = this->raw_xml_.get()->FirstChild("filters");
  if (!child_node)
  {
    ROS_ERROR("Cannot Configure LaserMedianFilter: Didn't find filters tag within LaserMedianFilter. Filter definitions needed inside for processing range and intensity");
    return false;
  }
  TiXmlElement * child = child_node->ToElement();
  
  latest_xml_.reset( child->Clone()->ToElement());
  
  if (range_filter_) delete range_filter_;
  range_filter_ = new filters::MultiChannelFilterChain<float>("filters", "filters::MultiChannelFilterBase<float>");
  if (!range_filter_->configure(num_ranges_, latest_xml_.get())) return false;
  
  if (intensity_filter_) delete intensity_filter_;
  intensity_filter_ = new filters::MultiChannelFilterChain<float>("filters", "filters::MultiChannelFilterBase<float>");
  if (!intensity_filter_->configure(num_ranges_, latest_xml_.get())) return false;
  return true;
};

LaserMedianFilter::~LaserMedianFilter()
{
  delete range_filter_;
  delete intensity_filter_;
};

bool LaserMedianFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
  if (!this->configured_) 
  {
    ROS_ERROR("LaserMedianFilter not configured");
    return false;
  }
  boost::mutex::scoped_lock lock(data_lock);
  scan_out = scan_in; ///Quickly pass through all data \todo don't copy data too


  if (scan_in.get_ranges_size() != num_ranges_) //Reallocating
  {
    ROS_INFO("Laser filter clearning and reallocating due to larger scan size");
    delete range_filter_;
    delete intensity_filter_;


    num_ranges_ = scan_in.get_ranges_size();
    
    range_filter_ = new filters::MultiChannelFilterChain<float>("filters", "filters::MultiChannelFilterBase<float>");
    if (!range_filter_->configure(num_ranges_, latest_xml_.get())) return false;
    
    intensity_filter_ = new filters::MultiChannelFilterChain<float>("filters", "filters::MultiChannelFilterBase<float>");
    if (!intensity_filter_->configure(num_ranges_, latest_xml_.get())) return false;
    
  }

  /** \todo check for length of intensities too */
  range_filter_->update(scan_in.ranges, scan_out.ranges);
  intensity_filter_->update(scan_in.intensities, scan_out.intensities);


  return true;
}
}
