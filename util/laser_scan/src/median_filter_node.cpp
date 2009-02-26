/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/node.h"
#include "laser_scan/LaserScan.h"
#include "laser_scan/median_filter.h"

static std::string median_filter_xml = "<filter type=\"MedianFilter\" name=\"median_test_5\"> <params number_of_observations=\"5\"/></filter>";


class MedianFilterNode 
{
public:
  laser_scan::LaserScan msg;


  MedianFilterNode(ros::Node& anode) :  filter_chain_(), node_(anode)
  {
    std::string filter_xml;
    node_.advertise<laser_scan::LaserScan>("~output", 1000);
    node_.param("~filters", filter_xml, median_filter_xml);
    printf("Got ~filters as: %s\n", filter_xml.c_str());
    TiXmlDocument xml_doc;
    xml_doc.Parse(filter_xml.c_str());
    TiXmlElement * config = xml_doc.RootElement();

    filter_chain_.configure(1, config);
    node_.subscribe("scan_in", msg, &MedianFilterNode::callback,this, 3);
  }
  void callback()
  {
    filter_chain_.update (msg, msg);
    node_.publish("~output", msg);
  }

protected:
  filters::FilterChain<laser_scan::LaserScan> filter_chain_;
  ros::Node& node_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node ros_node("scan_filter_node");
  
  MedianFilterNode t(ros_node);
  ros_node.spin();
  
  return 0;
}

