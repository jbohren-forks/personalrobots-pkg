/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include "rdf_parser/rdf.h"
#include <iostream>

using namespace std;
using namespace rdf_parser;


int main(int argc, char** argv)
{
  if (argc < 2){
    cerr << "Expect xml file to parse" << endl;
    return -1;
  }
  TiXmlDocument rdf_xml;
  rdf_xml.LoadFile(argv[1]);
  TiXmlElement *robot_xml = rdf_xml.FirstChildElement("robot");
  if (!robot_xml){
    cerr << "Could not parse the xml" << endl;
    return -1;
  }

  RDF robot;
  if (!robot.initXml(robot_xml)){
    cerr << "Parsing the xml failed" << endl;
    return -1;
  }

  // get info from parser
  std::cout << "---------- Finished Loading from URDF XML, Now Checking RDF structure ------------" << std::endl;
  // get root link
  Link* root_link=robot.getRoot();
  cout << "root " << root_link->getName() << " has " << root_link->getChildren()->size() << " children" << endl << endl;
  if (!root_link) return -1;


  // test first child
  std::vector<Link*>* children = root_link->getChildren();
  Link* first_child = *children->begin();
  if (!first_child) return -1;
  cout << "first child is: " << first_child->getName() << std::endl;
  // test children of first child
  std::cout << "first child has " << first_child->getChildren()->size() << " children" << endl << endl;


  // go through all children, print grandchildren
  cout << "root link: " << root_link->getName() << endl;
  int count = 0;
  for (std::vector<Link*>::iterator child = children->begin(); child != children->end(); child++)
  {
    if (*child)
    {
      cout << "  child(" << count++ << "):  " << (*child)->getName() << endl;
      // first grandchild
      std::vector<Link*>* grandchildren = (*child)->getChildren();
      int count2 = 0;
      for (std::vector<Link*>::iterator grandchild = grandchildren->begin(); grandchild != grandchildren->end(); grandchild++)
        if (*grandchild)
          cout << "    grandchild(" << count2++ << "): " << (*grandchild)->getName() << endl;
    }
    else
    {
      cout << "root link: " << root_link->getName() << " has a null child!" << *child << std::endl;
      return -1;
    }
  }
  return 0;
}

