/***************************************************************************
  tag: Peter Soetens  Thu Jul 3 15:30:14 CEST 2008  deployer.cpp

                        deployer.cpp -  description
                           -------------------
    begin                : Thu July 03 2008
    copyright            : (C) 2008 Peter Soetens
    email                : peter.soetens@fmtc.be

 ***************************************************************************
 *   This program is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 ***************************************************************************/

#include <rtt/os/main.h>
#include <rtt/RTT.hpp>

#include <ocl/TaskBrowser.hpp>
#include "RosDeploymentComponent.hpp"
#include <iostream>
#include <string>
#include "deployer-funcs.hpp"
#include <boost/thread/thread.hpp>

namespace po = boost::program_options;

int ORO_main(int argc, char** argv)
{
  std::string             script;
  std::string             name("Deployer");
  std::string             path;
  po::variables_map       vm;
  
  int rc = OCL::deployerParseCmdLine(argc, argv, script, name, path,vm);
  if (0 != rc)
    {
      return rc;
    }

  ros::init(argc,argv,"RTT/"+name);
  ros::NodeHandle dummy;

  OCL::RosDeploymentComponent dc( name );
  dc.properties()->getProperty<std::string>("ComponentPath")->set(path);
  
  if ( !script.empty() )
    {
      dc.kickStart( script );
    }
    
  if(!dc.connectRosPorts())
    log(RTT::Warning)<<"Some of the RosPorts were not succesfully connected."<<RTT::endlog();
  OCL::TaskBrowser tb( &dc );
  
  boost::thread* ros_thread = new boost::thread(boost::bind(&ros::spin));
  //ros::spinOnce();
  tb.loop();
  delete ros_thread;
  return 0;
}
