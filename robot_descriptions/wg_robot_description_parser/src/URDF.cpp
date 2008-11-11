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

/** \author Ioan Sucan */

#include <urdf/URDF.h>
#include <math_utils/MathExpression.h>
#include <string_utils/string_utils.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <queue>

namespace robot_desc {
    
    /* Macro to mark the fact a certain member variable was set. Also
       prints a warning if the same member was set multiple times. */
#define MARK_SET(node, owner, variable)					\
    {									\
	if (owner->isSet[#variable]) {					\
	    errorMessage("'" + std::string(#variable) +			\
			 "' already set");				\
	    errorLocation(node); }					\
	else								\
	    owner->isSet[#variable] = true;				\
    }
    

    /* Operator for sorting objects by name */
    template<typename T>
    struct SortByName
    {
	bool operator()(const T *a, const T *b) const
	{
	    return a->name < b->name;
	}
    };
    
    void URDF::freeMemory(void)
    {
	clearDocs();
	
	for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	    delete i->second;
	for (std::map<std::string, Group*>::iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	    delete i->second;
	for (std::map<std::string, Frame*>::iterator i = m_frames.begin() ; i != m_frames.end() ; i++)
	    delete i->second;
	for (std::map<std::string, Chain*>::iterator i = m_chains.begin() ; i != m_chains.end() ; i++)
	    delete i->second;
    }
    
    void URDF::clear(void)
    {
	freeMemory();
	m_name.clear();
	m_source.clear();
	m_location.clear();
	m_links.clear();
	m_frames.clear();
	m_groups.clear();
	m_paths.clear();
	m_paths.push_back("");
	m_linkRoots.clear();
	clearTemporaryData();
	m_errorCount = 0;
    }
    
    void URDF::setVerbose(bool verbose)
    {
	m_verbose = verbose;
    }
    
    void URDF::rememberUnknownTags(bool remember)
    {
	m_rememberUnknownTags = remember;
    }
    
    unsigned int URDF::getErrorCount(void) const
    {
	return m_errorCount;	
    }
    
    const std::string& URDF::getRobotName(void) const
    {
	return m_name;
    }
    
    URDF::Link* URDF::getLink(const std::string &name) const
    {
	std::map<std::string, Link*>::const_iterator it = m_links.find(name);
	return it == m_links.end() ? NULL : it->second;
    }
    
    URDF::Link* URDF::getJointLink(const std::string &name) const
    {	
	for (std::map<std::string, Link*>::const_iterator i = m_links.begin() ; i != m_links.end() ; i++)
	    if (i->second->joint->name == name)
		return i->second;
	return NULL;	
    }
    
    void URDF::getLinks(std::vector<Link*> &links) const
    {
	std::vector<Link*> localLinks;
	for (std::map<std::string, Link*>::const_iterator i = m_links.begin() ; i != m_links.end() ; i++)
	    localLinks.push_back(i->second);
	std::sort(localLinks.begin(), localLinks.end(), SortByName<Link>());
	links.insert(links.end(), localLinks.begin(), localLinks.end());
    }

    URDF::Frame* URDF::getFrame(const std::string &name) const
    {
	std::map<std::string, Frame*>::const_iterator it = m_frames.find(name);
	return it == m_frames.end() ? NULL : it->second;
    }
    
    void URDF::getFrames(std::vector<Frame*> &frames) const
    {
	std::vector<Frame*> localFrames;
	for (std::map<std::string, Frame*>::const_iterator i = m_frames.begin() ; i != m_frames.end() ; i++)
	    localFrames.push_back(i->second);
	std::sort(localFrames.begin(), localFrames.end(), SortByName<Frame>());
	frames.insert(frames.end(), localFrames.begin(), localFrames.end());
    }
    
    unsigned int URDF::getDisjointPartCount(void) const
    {
	return m_linkRoots.size();
    }
    
    URDF::Link* URDF::getDisjointPart(unsigned int index) const
    {
	if (index < m_linkRoots.size())
	    return m_linkRoots[index];
	else
	    return NULL;
    }
    
    bool URDF::isRoot(const Link* link) const
    {
	for (unsigned int i = 0 ; i < m_linkRoots.size() ; ++i)
	    if (link == m_linkRoots[i])
		return true;
	return false;
    }
    
    void URDF::getGroupNames(std::vector<std::string> &groups) const
    {
	std::vector<std::string> localGroups;
	for (std::map<std::string, Group*>::const_iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	    localGroups.push_back(i->first);
	std::sort(localGroups.begin(), localGroups.end());
	groups.insert(groups.end(), localGroups.begin(), localGroups.end());
    }
    
    URDF::Group* URDF::getGroup(const std::string &name) const
    {
	std::map<std::string, Group*>::const_iterator it = m_groups.find(name);
	return (it == m_groups.end()) ? NULL : it->second;
    }
    
    void URDF::getGroups(std::vector<Group*> &groups) const
    {
	/* To maintain the same ordering as getGroupNames, we do this in a slightly more cumbersome way */
	std::vector<std::string> names;
	getGroupNames(names);
	for (unsigned int i = 0 ; i < names.size() ; ++i)
	    groups.push_back(getGroup(names[i]));
    }
    
    URDF::Chain* URDF::getChain(const std::string &name) const
    {
	std::map<std::string, Chain*>::const_iterator it = m_chains.find(name);
	return (it == m_chains.end()) ? NULL : it->second;
    }
    
    void URDF::getChains(std::vector<Chain*> &chains) const
    {
	std::vector<Chain*> localChains;
	for (std::map<std::string, Chain*>::const_iterator i = m_chains.begin() ; i != m_chains.end() ; i++)
	    localChains.push_back(i->second);
	std::sort(localChains.begin(), localChains.end(), SortByName<Chain>());
	chains.insert(chains.end(), localChains.begin(), localChains.end());
    }    

    const URDF::Map& URDF::getMap(void) const
    {
	return m_data;
    }
    
    void URDF::Map::add(const std::string &type, const std::string &name, const std::string &key, const std::string &value)
    {
	if (!m_data[type][name][key].str)
	    m_data[type][name][key].str = new std::string();
	*(m_data[type][name][key].str) = value;
    }
    
    void URDF::Map::add(const std::string &type, const std::string &name, const std::string &key, const TiXmlElement *value)
    {
	m_data[type][name][key].xml = value;
    }
    
    bool URDF::Map::hasDefault(const std::string &key) const
    {
	std::map<std::string, std::string> m = getMapTagValues("", "");
	return m.find(key) != m.end();
    }
    
    std::string URDF::Map::getDefaultValue(const std::string &key) const
    {
	return getMapTagValues("", "")[key];
    }
    
    const TiXmlElement* URDF::Map::getDefaultXML(const std::string &key) const
    {
	return getMapTagXML("", "")[key];
    }
    
    void URDF::Map::getMapTagFlags(std::vector<std::string> &flags) const
    {
	for (std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator i = m_data.begin() ; i != m_data.end() ; ++i)
	    flags.push_back(i->first);
    }
    
    void URDF::Map::getMapTagNames(const std::string &flag, std::vector<std::string> &names) const
    {
	std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator pos = m_data.find(flag);
	if (pos != m_data.end())
	{
	    for (std::map<std::string, std::map<std::string, Element > >::const_iterator i = pos->second.begin() ; i != pos->second.end() ; ++i)
		names.push_back(i->first);
	}
    }
    std::map<std::string, std::string> URDF::Map::getMapTagValues(const std::string &flag, const std::string &name) const
    {    
	std::map<std::string, std::string> result;
	
	std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator pos = m_data.find(flag);
	if (pos != m_data.end())
	{
	    std::map<std::string, std::map<std::string, Element > >::const_iterator m = pos->second.find(name);
	    if (m != pos->second.end())
	    {
		for (std::map<std::string, Element>::const_iterator it = m->second.begin() ; it != m->second.end() ; it++)
		    if (it->second.str)
			result[it->first] = *(it->second.str);
	    }
	}
	return result;
    }
    
    std::map<std::string, const TiXmlElement*> URDF::Map::getMapTagXML(const std::string &flag, const std::string &name) const
    {    
	std::map<std::string, const TiXmlElement*> result;
	
	std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator pos = m_data.find(flag);
	if (pos != m_data.end())
	{
	    std::map<std::string, std::map<std::string, Element > >::const_iterator m = pos->second.find(name);
	    if (m != pos->second.end())
	    {
		for (std::map<std::string, Element>::const_iterator it = m->second.begin() ; it != m->second.end() ; it++)
		    if (it->second.xml)
			result[it->first] = it->second.xml;
	    }
	}
	return result;
    }
    
    bool URDF::containsCycle(unsigned int index) const
    {
	if (index >= m_linkRoots.size())
	    return false;
	
	std::map<Link*, bool> seen;
	
	std::queue<Link*> queue;
	queue.push(m_linkRoots[index]);
	
	while (!queue.empty())
	{
	    Link* link = queue.front();
	    queue.pop();
	    seen[link] = true;
	    for (unsigned int i = 0 ; i < link->children.size() ; ++i) {
		if (seen.find(link->children[i]) != seen.end())
		    return true;
		else
		    queue.push(link->children[i]);
	    }
	}
	
	return false;
    }
    
    void URDF::print(std::ostream &out) const
    {
	out << std::endl << "List of root links in robot '"<< m_name << "' (" << m_linkRoots.size() << ") :" << std::endl;
	for (unsigned int i = 0 ; i < m_linkRoots.size() ; ++i)
	    m_linkRoots[i]->print(out, "  ");
	out << std::endl << "Frames:" << std::endl;
	for (std::map<std::string, Frame*>::const_iterator i = m_frames.begin() ; i != m_frames.end() ; i++)
	    i->second->print(out, "  ");
	out << std::endl << "Groups:" << std::endl;
	for (std::map<std::string, Group*>::const_iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	    i->second->print(out, "  ");
	out << std::endl << "Chains:" << std::endl;
	for (std::map<std::string, Chain*>::const_iterator i = m_chains.begin() ; i != m_chains.end() ; i++)
	    i->second->print(out, "  ");
	out << std::endl << "Data types:" << std::endl;
	m_data.print(out, "  ");
    }

    void URDF::Group::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Group [" << name << "]:" << std::endl;
	out << indent << "  - links: ";
	for (unsigned int i = 0 ; i < linkNames.size() ; ++i)
	    out << linkNames[i] << " ";
	out << std::endl;
	out << indent << "  - frames: ";
	for (unsigned int i = 0 ; i < frameNames.size() ; ++i)
	    out << frameNames[i] << " ";
	out << std::endl;
	out << indent << "  - flags: ";
	for (unsigned int i = 0 ; i < flags.size() ; ++i)
	    out << flags[i] << " ";
	out << std::endl;
	data.print(out, indent + "  ");
    }

    void URDF::Chain::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Chain [" << name << "]:" << std::endl;
	out << indent << "  - links: ";
	for (unsigned int i = 0 ; i < links.size() ; ++i)
	    out << links[i]->name << " ";
	out << std::endl;
    }
    
    void URDF::Map::print(std::ostream &out, std::string indent) const
    {
	for (std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator i = m_data.begin() ; i != m_data.end() ; ++i)
	{
	    out << indent << "Data flagged as '" << i->first << "':" << std::endl;
	    for (std::map<std::string, std::map<std::string, Element > >::const_iterator j = i->second.begin() ; j != i->second.end() ; ++j)
	    {
		out << indent << "  [" << j->first << "]" << std::endl;
		for (std::map<std::string, Element>::const_iterator k = j->second.begin() ; k != j->second.end() ; ++k)
		    out << indent << "    " << k->first << " = " << (k->second.str != NULL ? *(k->second.str) : "") << (k->second.xml != NULL ? " [XML]" : "") << std::endl;
	    }
	}
    }
    
    void URDF::Frame::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Frame [" << name << "]:" << std::endl;
	out << indent << "  - type: " << type << std::endl;
	out << indent << "  - rpy: (" <<  rpy[0] << ", " << rpy[1] << ", " << rpy[2] << ")" << std::endl;
	out << indent << "  - xyz: (" <<  xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
	out << indent << "  - link: " << (link ? link->name : "") << std::endl;

	out << indent << "  - groups: ";
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    out << groups[i]->name << " ( ";
	    for (unsigned int j = 0 ; j < groups[i]->flags.size() ; ++j)
		out << groups[i]->flags[j] << " ";
	    out << ") ";
	}
	out << std::endl;
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Geometry::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Geometry [" << name << "]:" << std::endl;
	out << indent << "  - type: " << type << std::endl;
	if (shape)
	    switch (type)
	    {
	    case SPHERE:
		out << indent << "  - radius: " << static_cast<Sphere*>(shape)->radius << std::endl;
		break;
	    case CYLINDER:
		out << indent << "  - radius: " << static_cast<Cylinder*>(shape)->radius << std::endl;
		out << indent << "  - length: " << static_cast<Cylinder*>(shape)->length << std::endl;
		break;
	    case BOX:
		{
		    const double *size = static_cast<Box*>(shape)->size;
		    out << indent << "  - size: (" <<  size[0] << ", " << size[1] << ", " << size[2] << ")" << std::endl;
		}
		break;
	    case MESH:
		out << indent << "  - filename: " << static_cast<Mesh*>(shape)->filename << std::endl;
		if (isSet.find("scale")->second)
		{
		    const double *scale = static_cast<Mesh*>(shape)->scale;
		    out << indent << "  - scale: (" <<  scale[0] << ", " << scale[1] << ", " << scale[2] << ")" << std::endl;
		}
		break;
	    default:
		break;
	    }
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Joint::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Joint [" << name << "]:" << std::endl;
	out << indent << "  - type: " << type << std::endl;
	out << indent << "  - axis: (" <<  axis[0] << ", " << axis[1] << ", " << axis[2] << ")" << std::endl;
	out << indent << "  - anchor: (" <<  anchor[0] << ", " << anchor[1] << ", " << anchor[2] << ")" << std::endl;
	out << indent << "  - limit: (" <<  limit[0] << ", " << limit[1] << ")" << std::endl;
	out << indent << "  - effortLimit: " << effortLimit << std::endl;
	out << indent << "  - velocityLimit: " << velocityLimit << std::endl;
	out << indent << "  - calibration: " << calibration << std::endl;
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Collision::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Collision [" << name << "]:" << std::endl;
	out << indent << "  - verbose: " << (verbose ? "Yes" : "No") << std::endl;
	out << indent << "  - rpy: (" <<  rpy[0] << ", " << rpy[1] << ", " << rpy[2] << ")" << std::endl;
	out << indent << "  - xyz: (" <<  xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
	geometry->print(out, indent + "  ");
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Inertial::print(std::ostream &out, std::string indent) const
    {	
	out << indent << "Inertial [" << name << "]:" << std::endl;
	out << indent << "  - mass: " << mass << std::endl;
	out << indent << "  - com: (" <<  com[0] << ", " << com[1] << ", " << com[2] << ")" << std::endl;
	out << indent << "  - inertia: (" <<  inertia[0] << ", " << inertia[1] << ", " << inertia[2] << ", " << inertia[3] << ", " << inertia[4] << ", " << inertia[5] << ")" << std::endl;
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Visual::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Visual [" << name << "]:" << std::endl;
	out << indent << "  - rpy: (" <<  rpy[0] << ", " << rpy[1] << ", " << rpy[2] << ")" << std::endl;
	out << indent << "  - xyz: (" <<  xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
	geometry->print(out, indent + "  ");
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Link [" << name << "]:" << std::endl;
	out << indent << "  - parent link: " << parentName << std::endl;
	out << indent << "  - rpy: (" <<  rpy[0] << ", " << rpy[1] << ", " << rpy[2] << ")" << std::endl;
	out << indent << "  - xyz: (" <<  xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
	joint->print(out, indent+ "  ");
	collision->print(out, indent+ "  ");
	inertial->print(out, indent+ "  ");
	visual->print(out, indent+ "  ");
	
	out << indent << "  - groups: ";
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{      
	    out << groups[i]->name <<  " ( ";
	    for (unsigned int j = 0 ; j < groups[i]->flags.size() ; ++j)
		out << groups[i]->flags[j] << " ";
	    out << ") ";
	}
	out << std::endl;
	
	out << indent << "  - children links: ";
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	    out << children[i]->name << " ";
	out << std::endl;

	for (unsigned int i = 0 ; i < children.size() ; ++i)
	    children[i]->print(out, indent + "  ");
	data.print(out, indent + "  ");
    }
    
    void URDF::Sensor::print(std::ostream &out, std::string indent) const
    {
	out << indent << "Sensor:" << std::endl;
	out << indent << "  - type: " << type << std::endl;
	out << indent << "  - calibration: " << calibration << std::endl;
	Link::print(out, indent + "  ");
    }
    
    bool URDF::Link::canSense(void) const
    {
	return false;
    }
    
    bool URDF::Sensor::canSense(void) const
    {
	return true;
    }
    
    bool URDF::Link::insideGroup(Group *group) const
    {
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	    if (groups[i] == group)
		return true;
	return false;
    }

    bool URDF::Link::insideGroup(const std::string &group) const
    {
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	    if (groups[i]->name == group)
		return true;
	return false;
    }
    
    bool URDF::Group::empty(void) const
    {
	return links.empty() && frames.empty();
    }
    
    bool URDF::Group::hasFlag(const std::string &flag) const
    {
	for (unsigned int i = 0 ; i < flags.size() ; ++i)
	    if (flags[i] == flag)
		return true;
	return false;	  
    }    
    
    bool URDF::Group::isRoot(const Link* link) const
    {
	for (unsigned int i = 0 ; i < linkRoots.size() ; ++i)
	    if (linkRoots[i] == link)
		return true;
	return false;    
    }
    
    void URDF::errorMessage(const std::string &msg) const
    {
	if (m_verbose)
	    std::cerr << msg << std::endl;
	m_errorCount++;
    }
    
    void URDF::errorLocation(const TiXmlNode* node) const
    {
	if (!m_verbose)
	    return;
	
	if (!m_location.empty())
	{
	    std::cerr << "  ... at " << m_location;
	    if (!node)
		std::cerr << std::endl;
	}
	
	if (node)
	{
	    /* find the document the node is part of */
	    const TiXmlNode* doc = node;
	    while (doc && doc->Type() != TiXmlNode::DOCUMENT)
		doc = doc->Parent();
	    const char *filename = doc ? reinterpret_cast<const char*>(doc->GetUserData()) : NULL;
	    std::cerr << (m_location.empty() ? "  ..." : ",") << " line " << node->Row() << ", column " << node->Column();
	    if (filename)
		std::cerr << " (" << filename << ")";
	    std::cerr << std::endl;
	}
    }
    
    void URDF::unknownNode(const TiXmlNode* node)
    {
	if (m_rememberUnknownTags)
	    m_unknownTags.push_back(node);
	else
	    switch (node->Type())
	    {
	    case TiXmlNode::ELEMENT:
		errorMessage("Ignoring element node '" + node->ValueStr() + "'");
		errorLocation(node);  
		break;
	    case TiXmlNode::TEXT:
		errorMessage("Ignoring text node with content '" + node->ValueStr() + "'");
		errorLocation(node);  
		break;
	    case TiXmlNode::COMMENT:
	    case TiXmlNode::DECLARATION:
		break;            
	    case TiXmlNode::UNKNOWN:
	    default:
		errorMessage("Ignoring unknown node '" + node->ValueStr() + "'");
		errorLocation(node);  
		break;
	    }
    }
    
    void URDF::getActuators(std::vector<const TiXmlElement*> &actuators) const
    {
	actuators = m_actuators;
    }
    
    void URDF::getTransmissions(std::vector<const TiXmlElement*> &transmissions) const
    {
	transmissions = m_transmissions;
    }
    
    void URDF::getControllers(std::vector<const TiXmlElement*> &controllers) const
    {
	controllers = m_controllers;
    }
    
    void URDF::getUnknownTags(std::vector<const TiXmlNode*> &unknownTags) const
    {
	unknownTags = m_unknownTags;
    }
    
    void URDF::getChildrenAndAttributes(const TiXmlNode *node, std::vector<const TiXmlNode*> &children, std::vector<const TiXmlAttribute*> &attributes) const
    {
	for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
	{
	    if (child->Type() == TiXmlNode::ELEMENT && child->ValueStr() == "insert_const_block")
	    {
		const char *nm = child->ToElement()->Attribute("name");
		if (nm)
		{
		    std::string name = string_utils::trim(nm);		    
		    std::map<std::string, const TiXmlNode*>::const_iterator pos = m_constBlocks.find(name);
		    if (pos == m_constBlocks.end())
		    {
			errorMessage("Constant block '" + name + "' is not defined");
			errorLocation(child);
		    }
		    else
			getChildrenAndAttributes(pos->second, children, attributes);
		}
		else
		{
		    errorMessage("Undefined name when referencing a constant block");
		    errorLocation(child);
		}
	    }
	    else
		children.push_back(child);
	}
	
	if (node->Type() == TiXmlNode::ELEMENT && node->ValueStr() != "const_block")
	    for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
		attributes.push_back(attr);	
    }
    
    void URDF::defaultConstants(void)
    {
	m_constants["M_PI"] = "3.14159265358979323846";
    }
    
    void URDF::clearDocs(void)
    {
	/* first, clear datastructures that may be pointing to xml elements */
	
	m_constants.clear();
	m_constBlocks.clear();
	m_actuators.clear();
	m_controllers.clear();
	m_transmissions.clear();	
	m_unknownTags.clear();

	/* clear memory allocated for loaded documents */
	for (unsigned int i = 0 ; i < m_docs.size() ; ++i)
	{
	    char *filename = reinterpret_cast<char*>(m_docs[i]->GetUserData());
	    if (filename)
		free(filename);
	    delete m_docs[i];
	}	
	m_docs.clear();
    }
    
    void URDF::clearTemporaryData(void)
    {
	m_collision.clear();
	m_joints.clear();
	m_inertial.clear();
	m_visual.clear();
	m_geoms.clear();
	m_stage2.clear();
    }
    
    bool URDF::loadStream(std::istream &is)
    {
	if (!is.good())
	    return false;
	
	is.seekg(0, std::ios::end);
	std::streampos length = is.tellg();
	is.seekg(0, std::ios::beg);
	if (length >= 0)
	{
	    char *buffer = new char[length];
	    is.read(buffer, length);
	    bool result = (is.gcount() == length) ? loadString(buffer) : false;
	    delete[] buffer;
	    return result;
	}
	else
	    return false;
    }
    
    bool URDF::loadString(const char *data)
    {
	clear();
	bool result = false;
	
	TiXmlDocument *doc = new TiXmlDocument();
	doc->SetUserData(NULL);	
	m_docs.push_back(doc);
	if (doc->Parse(data))
	{
	    defaultConstants();
	    result = parse(dynamic_cast<const TiXmlNode*>(doc));
	}
	else
	    errorMessage(doc->ErrorDesc());
	
	return result;
    }  
    
    bool URDF::loadFile(FILE *file)
    {
	clear();
	bool result = false;
	
	TiXmlDocument *doc = new TiXmlDocument();
	doc->SetUserData(NULL);	
	m_docs.push_back(doc);
	if (doc->LoadFile(file))
	{
	    defaultConstants();
	    result = parse(dynamic_cast<const TiXmlNode*>(doc));
	}
	else
	    errorMessage(doc->ErrorDesc());
	
	return result;
    }
    
    bool URDF::loadFile(const char *filename)
    {
	clear();
	bool result = false;
	m_source = filename;
	
	TiXmlDocument *doc = new TiXmlDocument(filename);
	doc->SetUserData(filename ? reinterpret_cast<void*>(strdup(filename)) : NULL);	
	m_docs.push_back(doc);
	if (doc->LoadFile())
	{
	    addPath(filename);
	    defaultConstants();
	    result = parse(dynamic_cast<const TiXmlNode*>(doc));
	}
	else
	    errorMessage(doc->ErrorDesc());
	
	return result;
    }
    
    void URDF::addPath(const char *filename)
    {
	if (!filename)
	    return;
	
	std::string name = filename;
	std::string::size_type pos = name.find_last_of("/\\");
	if (pos != std::string::npos)
	{
	    char sep = name[pos];
	    name.erase(pos);
	    m_paths.push_back(name + sep);
	}    
    }
    
    char* URDF::findFile(const char *filename)
    {
	std::string fnm = string_utils::trim(filename);
	for (unsigned int i = 0 ; i < m_paths.size() ; ++i)
	{
	    std::string name = m_paths[i] + fnm;
	    std::fstream fin;
	    fin.open(name.c_str(), std::ios::in);
	    bool good = fin.is_open();
	    fin.close();
	    if (good)
		return strdup(name.c_str());        
	}
	return NULL;
    }
    
    std::string URDF::extractName(std::vector<const TiXmlAttribute*> &attributes, const std::string &defaultName) const
    { 
	std::string name = defaultName;
	for (unsigned int i = 0 ; i < attributes.size() ; ++i)
	{
	    if (strcmp(attributes[i]->Name(), "name") == 0)
	    {
		name = string_utils::trim(attributes[i]->ValueStr());
		attributes.erase(attributes.begin() + i);
		break;
	    }
	}
	return name;
    }
    
    struct getConstantData
    {
	getConstantData(void)
	{
	    m = NULL;
	    errorCount = 0;
	}
	
	std::map<std::string, std::string> *m;
	std::map<std::string, bool>         s;
	
	unsigned int                        errorCount;
	std::vector<std::string>            errorMsg;
    };
    
    static double getConstant(void *data, std::string &name)
    {
	getConstantData *d = reinterpret_cast<getConstantData*>(data);
	std::map<std::string, std::string> *m = d->m;
	assert(m);
	if (m->find(name) == m->end())
	{
	    d->errorMsg.push_back("Request for undefined constant: '" + name + "'");
	    d->errorCount++;
	    return 0.0;
	}
	else
	{
	    if (meval::ContainsOperators((*m)[name]))
	    {
		std::map<std::string, bool>::iterator pos = d->s.find((*m)[name]);
		if (pos != d->s.end() && pos->second == true)
		{
		    d->errorMsg.push_back("Recursive definition of constant '" + name + "'");
		    d->errorCount++;
		    return 0.0;
		}
		d->s[(*m)[name]] = true;
		double result = meval::EvaluateMathExpression((*m)[name], &getConstant, data);
		d->s[(*m)[name]] = false;
		return result;
	    }
	    else
		return atof((*m)[name].c_str());
	}
    }
    
    double URDF::getConstantValue(const std::string &name, bool *error) const
    {
	getConstantData data;
	data.m = const_cast<std::map<std::string, std::string>*>(&m_constants);
	double result = meval::EvaluateMathExpression(name, &getConstant, reinterpret_cast<void*>(&data));
	for (unsigned int k = 0 ; k < data.errorMsg.size() ; ++k)
	    errorMessage(data.errorMsg[k]);
	if (data.errorCount)
	    m_errorCount += data.errorCount;
	if (error)
	    *error = data.errorCount > 0;
	return result;
    }
    
    std::string URDF::getConstantString(const std::string &name, bool *error) const
    {
	std::map<std::string, std::string>::const_iterator pos = m_constants.find(name);
	if (error)
	    *error = pos == m_constants.end();
	if (pos != m_constants.end())
	    return pos->second;
	return "";
    }
    
    unsigned int URDF::loadDoubleValues(const TiXmlNode *node, unsigned int count, double *vals, const char *attrName, bool warn)
    {
	if (attrName)
	{
	    if (node && node->Type() == TiXmlNode::ELEMENT)
	    {
		const char* value = node->ToElement()->Attribute(attrName);
		if (value)
		    return loadDoubleValues(value, count, vals, node);
		else
		{
		    if (warn)
		    {
			errorMessage("Attribute " + std::string(attrName) + " is missing");			
			errorLocation(node);
		    }		    
		    return 0;
		}		
	    }
	}
	else
	{
	    if (node && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		node = node->FirstChild();
	    if (node && node->Type() == TiXmlNode::TEXT)
		return loadDoubleValues(node->ValueStr(), count, vals, node);	    
	}	
	return 0;
    }
    
    unsigned int URDF::loadDoubleValues(const std::string &data, unsigned int count, double *vals, const TiXmlNode *node)
    {
	std::stringstream ss(data);
	unsigned int read = 0;
	
	for (unsigned int i = 0 ; ss.good() && i < count ; ++i)
	{
	    std::string value;
	    bool err;
	    ss >> value;
	    vals[i] = getConstantValue(value, &err);
	    if (err)
		errorLocation(node);
	    read++;
	}

	if (ss.good())
	{
	    std::string extra;
	    ss >> extra;
	    extra = string_utils::trim(extra);
	    if (!extra.empty())
	    {
		errorMessage("More data available (" + string_utils::convert2str(read) + " read, rest is ignored): '" + data + "'");
		errorLocation(node);
	    }	    
	}
	
	if (read != count)
	{
	    errorMessage("Not all values were read: '" + data + "'");
	    errorLocation(node);
	}  
	
	return read;
    }
    
    unsigned int URDF::loadBoolValues(const TiXmlNode *node, unsigned int count, bool *vals, const char *attrName, bool warn)
    {
    	if (attrName)
	{
	    if (node && node->Type() == TiXmlNode::ELEMENT)
	    {
		const char* value = node->ToElement()->Attribute(attrName);
		if (value)
		    return loadBoolValues(value, count, vals, node);
		else
		{
		    if (warn)
		    {
			errorMessage("Attribute " + std::string(attrName) + " is missing");			
			errorLocation(node);
		    }		    
		    return 0;
		}
	    }
	}
	else
	{
	    if (node && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		node = node->FirstChild();
	    if (node && node->Type() == TiXmlNode::TEXT)
		return loadBoolValues(node->ValueStr(), count, vals, node);
	}
	return 0;
    }
    
    unsigned int URDF::loadBoolValues(const std::string& data, unsigned int count, bool *vals, const TiXmlNode *node)
    {
	std::stringstream ss(data);
	unsigned int read = 0;
	
	for (unsigned int i = 0 ; ss.good() && i < count ; ++i)
	{
	    std::string value;
	    ss >> value;
	    value = string_utils::tolower(value);
	    vals[i] = (value == "true" || value == "yes" || value == "1");
	    read++;
	}

	if (ss.good())
	{
	    std::string extra;
	    ss >> extra;
	    extra = string_utils::trim(extra);
	    if (!extra.empty())
	    {	
		errorMessage("More data available (" + string_utils::convert2str(read) + " read, rest is ignored): '" + data + "'");
		errorLocation(node);
	    }	    
	}
	
	if (read != count)
	{
	    errorMessage("Not all values were read: '" + data + "'");
	    errorLocation(node);  
	}
	
	return read;
    }
    
    void URDF::loadFrame(const TiXmlNode *node)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, "");    
	Frame *frame = (m_frames.find(name) != m_frames.end()) ? m_frames[name] : new Frame();
	frame->name = name;
	if (frame->name.empty())
	{
	    errorMessage("No frame name given");
	    errorLocation(node);
	}
	else
	    MARK_SET(node, frame, name);
	
	m_frames[frame->name] = frame;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "origin")
		{
		    if (loadDoubleValues(node, 3, frame->rpy, "rpy"))
			MARK_SET(node, frame, rpy);
		    if (loadDoubleValues(node, 3, frame->xyz, "xyz"))
			MARK_SET(node, frame, xyz);
		}
		else if (node->ValueStr() == "parent")
		{
		    const char *nm = node->ToElement()->Attribute("name");
		    if (nm)
		    {
			frame->linkName = string_utils::trim(nm);
			MARK_SET(node, frame, parent);
			if (frame->type == Frame::CHILD)
			    errorMessage("Frame '" + frame->name + "' can only have either a child or a parent link");
			frame->type = Frame::PARENT;
		    }
		    else
		    {
			errorMessage("No name defined for parent");			
			errorLocation(node);
		    }		    
		}
		else if (node->ValueStr() == "child")
		{
		    const char *nm = node->ToElement()->Attribute("name");
		    if (nm)
		    {
			frame->linkName = string_utils::trim(nm);
			MARK_SET(node, frame, child);
			if (frame->type == Frame::PARENT)
			    errorMessage("Frame '" + frame->name + "' can only have either a child or a parent link");
			frame->type = Frame::CHILD;
		    }
		    else
		    {
			errorMessage("No name defined for child");			
			errorLocation(node);
		    }	
		}
		else
		    if (node->ValueStr() == "map")
			loadMap(node, &frame->data);
		    else
			unknownNode(node);
	    }
	    else
		unknownNode(node);
	}    
    }
    
    void URDF::loadJoint(const TiXmlNode *node, const std::string& defaultName, Link::Joint *joint)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (joint && !name.empty())
	    MARK_SET(node, joint, name);
	
	if (!joint)
	{
	    if (m_joints.find(name) == m_joints.end())
	    {
		errorMessage("Attempting to add information to an undefined joint: '" + name + "'");
		errorLocation(node);		
		return;
	    }
	    else
		joint = m_joints[name];
	}
	
	joint->name = name;
	m_joints[name] = joint;
	
	for (unsigned int i = 0 ; i < attributes.size() ; ++i)
	{
	    const TiXmlAttribute *attr = attributes[i];
	    if (strcmp(attr->Name(), "type") == 0)
	    {
		if (attr->ValueStr() == "fixed")
		    joint->type = Link::Joint::FIXED;
		else if (attr->ValueStr() == "revolute")
		    joint->type = Link::Joint::REVOLUTE;
		else if (attr->ValueStr() == "prismatic")
		    joint->type = Link::Joint::PRISMATIC;
		else if (attr->ValueStr() == "floating")
		    joint->type = Link::Joint::FLOATING;
		else if (attr->ValueStr() == "planar")
		    joint->type = Link::Joint::PLANAR;
		else
		{
		    errorMessage("Unknown joint type: '" + attr->ValueStr() + "'");
		    errorLocation(node);
		}
		MARK_SET(node, joint, type);
	    }
	}
	
	bool free = joint->type == Link::Joint::PLANAR || joint->type == Link::Joint::FLOATING;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "axis" && !free)
		{
		    if (loadDoubleValues(node, 3, joint->axis, "xyz", true))
			MARK_SET(node, joint, axis);
		}		
		else if (node->ValueStr() == "anchor" && !free)
		{
		    if (loadDoubleValues(node, 3, joint->anchor, "xyz", true))
			MARK_SET(node, joint, anchor);
		}
		else if (node->ValueStr() == "joint_properties" && !free)
		{
		    if (loadDoubleValues(node, 1, &(joint->damping), "damping", true))
			MARK_SET(node, joint, damping);
		    if (loadDoubleValues(node, 1, &(joint->friction), "friction", true))
			MARK_SET(node, joint, friction);
		}		
		else if (node->ValueStr() == "limit" && !free)
		{
		    int vmin = loadDoubleValues(node, 1, joint->limit + 0, "min");
		    int vmax = loadDoubleValues(node, 1, joint->limit + 1, "max");
		    if (vmin ^ vmax)
		    {
			errorMessage("Only one bound given to joint limits");
			errorLocation(node);
		    }
		    if (vmin && vmax)
			MARK_SET(node, joint, limit);
		    
		    if (loadDoubleValues(node, 1, &joint->effortLimit, "effort"))
			MARK_SET(node, joint, effortLimit);
		    if (loadDoubleValues(node, 1, &joint->velocityLimit, "velocity"))
			MARK_SET(node, joint, velocityLimit);
		}
		else if (node->ValueStr() == "safety_limit_min" && !free)
		{
		    if (loadDoubleValues(node, 1, joint->safetyLength + 0, "safety_length", true))
			MARK_SET(node, joint, safetyLengthMin);
		}
		else if (node->ValueStr() == "safety_limit_max" && !free)
		{
		    if (loadDoubleValues(node, 1, joint->safetyLength + 1, "safety_length", true))
			MARK_SET(node, joint, safetyLengthMax);
		}
		else if (node->ValueStr() == "calibration")
		{
		    const char *vals = node->ToElement()->Attribute("values");
		    if (vals)
		    {
			joint->calibration = vals;
			MARK_SET(node, joint, calibration);
		    }
		    else
		    {
			errorMessage("Calibration values missing");			
			errorLocation(node);
		    }
		}	
		else if (node->ValueStr() == "map")
		    loadMap(node, &joint->data);
		else
		    unknownNode(node);
	    }
	    else
		unknownNode(node);
	}
	
    }
    
    void URDF::loadGeometry(const TiXmlNode *node, const std::string &defaultName, Link::Geometry *geometry)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (geometry && !name.empty())
	    MARK_SET(node, geometry, name);

	if (!geometry)
	{
	    if (m_geoms.find(name) == m_geoms.end())
	    {
		errorMessage("Attempting to add information to an undefined geometry: '" + name + "'");
		errorLocation(node);		
		return;
	    }
	    else
		geometry = m_geoms[name];
	}
	
	geometry->name = name;
	m_geoms[name] = geometry;

	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "mesh")
		{
		    geometry->type = Link::Geometry::MESH;
		    Link::Geometry::Mesh* mesh = new Link::Geometry::Mesh();
		    geometry->shape = mesh;
		    const char *nm = node->ToElement()->Attribute("filename");
		    if (nm)
		    {
			mesh->filename = nm;
			MARK_SET(node, geometry, filename);
		    }
		    if (loadDoubleValues(node, 3, mesh->scale, "scale"))
			MARK_SET(node, geometry, scale);
		}
		else if (node->ValueStr() == "box")
		{
		    geometry->type = Link::Geometry::BOX;
		    Link::Geometry::Box* box = new Link::Geometry::Box();
		    geometry->shape = box;
		    if (loadDoubleValues(node, 3, box->size, "size", true))
			MARK_SET(node, geometry, size);
		}
		else if (node->ValueStr() == "sphere")
		{
		    geometry->type = Link::Geometry::SPHERE;
		    Link::Geometry::Sphere* sphere = new Link::Geometry::Sphere();
		    geometry->shape = sphere;
		    if (loadDoubleValues(node, 1, &sphere->radius, "radius", true))
			MARK_SET(node, geometry, radius);
		}
		else if (node->ValueStr() == "cylinder")
		{
		    geometry->type = Link::Geometry::CYLINDER;
		    Link::Geometry::Cylinder* cylinder = new Link::Geometry::Cylinder();
		    geometry->shape = cylinder;
		    if (loadDoubleValues(node, 1, &cylinder->radius, "radius", true))
			MARK_SET(node, geometry, radius);
		    if (loadDoubleValues(node, 1, &cylinder->length, "length", true))
			MARK_SET(node, geometry, length);
		}
		else if (node->ValueStr() == "map")
		    loadMap(node, &geometry->data);	
		else                
		    unknownNode(node);
	    }
	    else
		unknownNode(node);
	}
    }
    
    void URDF::loadCollision(const TiXmlNode *node, const std::string &defaultName, Link::Collision *collision)
    {  
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (collision && !name.empty())
	    MARK_SET(node, collision, name);
	
	if (!collision)
	{
	    if (m_collision.find(name) == m_collision.end())
	    {
		errorMessage("Attempting to add information to an undefined collision: '" + name + "'");
		errorLocation(node);
		return;
	    }
	    else
		collision = m_collision[name];
	}
	
	collision->name = name;
	m_collision[name] = collision;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "origin")
		{
		    if (loadDoubleValues(node, 3, collision->rpy, "rpy"))
			MARK_SET(node, collision, rpy);		    			
		    if (loadDoubleValues(node, 3, collision->xyz, "xyz"))
			MARK_SET(node, collision, xyz);		    
		}
		else if (node->ValueStr() == "verbose")
		{
		    loadBoolValues(node, 1, &collision->verbose, "value", true);
		    MARK_SET(node, collision, verbose);
		}		
		else if (node->ValueStr() == "geometry")
		{
		    if (collision->geometry == NULL)
			collision->geometry = new Link::Geometry();
		    loadGeometry(node, name + "_geom", collision->geometry);
		    MARK_SET(node, collision, geometry);		    
		}		
		else if (node->ValueStr() == "map")
		    loadMap(node, &collision->data);
		else
		    unknownNode(node);
	    }
	    else
		unknownNode(node);
	}    
    }
    
    void URDF::loadVisual(const TiXmlNode *node, const std::string &defaultName, Link::Visual *visual)
    {  
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (visual && !name.empty())
	    MARK_SET(node, visual, name);

	if (!visual)
	{
	    if (m_visual.find(name) == m_visual.end())
	    {
		errorMessage("Attempting to add information to an undefined visual: '" + name + "'");
		errorLocation(node);
		return;
	    }
	    else
		visual = m_visual[name];
	}
	
	visual->name = name;
	m_visual[name] = visual;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "origin")
		{
		    if (loadDoubleValues(node, 3, visual->rpy, "rpy"))
			MARK_SET(node, visual, rpy);
		    if (loadDoubleValues(node, 3, visual->xyz, "xyz"))
			MARK_SET(node, visual, xyz);
		}
		else if (node->ValueStr() == "geometry")
		{
		    if (visual->geometry == NULL)
			visual->geometry = new Link::Geometry();
		    loadGeometry(node, name + "_geom", visual->geometry);
		    MARK_SET(node, visual, geometry);
		}
		else if (node->ValueStr() == "map")
		    loadMap(node, &visual->data);
		else
		    unknownNode(node);
	    }
	    else
		unknownNode(node);
	}   
    }
    
    void URDF::loadInertial(const TiXmlNode *node, const std::string &defaultName, Link::Inertial *inertial)
    { 
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (inertial && !name.empty())
	    MARK_SET(node, inertial, name);
	
	if (!inertial)
	{
	    if (m_inertial.find(name) == m_inertial.end())
	    {
		errorMessage("Attempting to add information to an undefined inertial component: '" + name + "'");
		errorLocation(node);
		return;
	    }
	    else
		inertial = m_inertial[name];
	}
	
	inertial->name = name;
	m_inertial[name] = inertial;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "mass")
		{
		    if (loadDoubleValues(node, 1, &inertial->mass, "value"), true)
			MARK_SET(node, inertial, mass);
		}		
		else if (node->ValueStr() == "com")
		{
		    if (loadDoubleValues(node, 3, inertial->com, "xyz"), true)
			MARK_SET(node, inertial, com);
		}
		else if (node->ValueStr() == "inertia")
		{
		    /* Ixx Ixy Ixz Iyy Iyz Izz */
		    if (node->ToElement()->Attribute("Ixx"))
			loadDoubleValues(node, 1, inertial->inertia + 0, "Ixx", true);
		    else
			loadDoubleValues(node, 1, inertial->inertia + 0, "ixx", true);
		    if (node->ToElement()->Attribute("Ixy"))
			loadDoubleValues(node, 1, inertial->inertia + 1, "Ixy", true);
		    else
			loadDoubleValues(node, 1, inertial->inertia + 1, "ixy", true);
		    if (node->ToElement()->Attribute("Ixz"))
			loadDoubleValues(node, 1, inertial->inertia + 2, "Ixz", true);
		    else		    
			loadDoubleValues(node, 1, inertial->inertia + 2, "ixz", true);
		    if (node->ToElement()->Attribute("Iyy"))
			loadDoubleValues(node, 1, inertial->inertia + 3, "Iyy", true);
		    else
			loadDoubleValues(node, 1, inertial->inertia + 3, "iyy", true);
		    if (node->ToElement()->Attribute("Iyz"))
			loadDoubleValues(node, 1, inertial->inertia + 4, "Iyz", true);
		    else
			loadDoubleValues(node, 1, inertial->inertia + 4, "iyz", true);
		    if (node->ToElement()->Attribute("Izz"))
			loadDoubleValues(node, 1, inertial->inertia + 5, "Izz", true);
		    else
			loadDoubleValues(node, 1, inertial->inertia + 5, "izz", true);
		    MARK_SET(node, inertial, inertia);
		}		
		else if (node->ValueStr() == "map")
		    loadMap(node, &inertial->data);
		else
		    unknownNode(node);
	    }
	    else
		unknownNode(node);
	}
    }
    
    void URDF::loadLink(const TiXmlNode *node)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, "");    
	Link *link = (m_links.find(name) != m_links.end()) ? m_links[name] : new Link();
	link->name = name;
	if (link->name.empty())
	{
	    errorMessage("No link name given");
	    errorLocation(node);
	}
	else
	    MARK_SET(node, link, name);
	m_links[link->name] = link;
	
	if (link->canSense())
	    errorMessage("Link '" + link->name + "' was already defined as a sensor");

	std::string currentLocation = m_location;
	m_location = "link '" + name + "'";
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "parent")
		{
		    const char *nm = node->ToElement()->Attribute("name");
		    if (nm)
		    {
			link->parentName = string_utils::trim(nm);
			MARK_SET(node, link, parent);
		    }
		    else
		    {
			errorMessage("Parent name is not defined");
			errorLocation(node);
		    }		    
		}
		else if (node->ValueStr() == "origin")
		{
		    if (loadDoubleValues(node, 3, link->rpy, "rpy"))
			MARK_SET(node, link, rpy);			
		    if (loadDoubleValues(node, 3, link->xyz, "xyz"))
			MARK_SET(node, link, xyz);
		}		
		else if (node->ValueStr() == "joint")
		{
		    if (link->joint == NULL)
			link->joint = new Link::Joint();
		    loadJoint(node, name + "_joint", link->joint);
		    MARK_SET(node, link, joint);
		}		
		else if (node->ValueStr() == "collision")
		{
		    if (link->collision == NULL)
			link->collision = new Link::Collision();
		    loadCollision(node, name + "_collision", link->collision);
		    MARK_SET(node, link, collision);
		}
		else if (node->ValueStr() == "inertial")
		{
		    if (link->inertial == NULL)
			link->inertial = new Link::Inertial();
		    loadInertial(node, name + "_inertial", link->inertial);
		    MARK_SET(node, link, inertial);
		}		
		else if (node->ValueStr() == "visual")
		{
		    if (link->visual == NULL)
			link->visual = new Link::Visual();
		    loadVisual(node, name + "_visual", link->visual);
		    MARK_SET(node, link, visual);
		}		
		else if (node->ValueStr() == "map")
		    loadMap(node, &link->data);
		else
		    unknownNode(node);
	    }
	    else
		unknownNode(node);
	}
	
	m_location = currentLocation;  
    }
    
    void URDF::loadSensor(const TiXmlNode *node)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, "");    
	Sensor *sensor = (m_links.find(name) != m_links.end()) ? dynamic_cast<Sensor*>(m_links[name]) : new Sensor();
	if (!sensor)
	{
	    errorMessage("Link with name '" + name + "' has already been defined");
	    sensor = new Sensor();
	}
	
	sensor->name = name;
	if (sensor->name.empty())
	    errorMessage("No sensor name given");
	else
	    MARK_SET(node, sensor, name);
	if (m_links.find(sensor->name) != m_links.end())
	    errorMessage("Sensor '" + sensor->name + "' redefined");
	m_links[sensor->name] = dynamic_cast<Link*>(sensor);
	
	std::string currentLocation = m_location;
	m_location = "sensor '" + name + "'";
	
	for (unsigned int i = 0 ; i < attributes.size() ; ++i)
	{
	    const TiXmlAttribute *attr = attributes[i];
	    if (strcmp(attr->Name(), "type") == 0)
	    {
		if (attr->ValueStr() == "camera")
		    sensor->type = Sensor::CAMERA;
		else if (attr->ValueStr() == "laser")
		    sensor->type = Sensor::LASER;
		else if (attr->ValueStr() == "stereocamera")
		    sensor->type = Sensor::STEREO_CAMERA;
		else
		{
		    errorMessage("Unknown sensor type: '" + attr->ValueStr() + "'");
		    errorLocation(node);
		}
		MARK_SET(node, sensor, type);
	    }
	}
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "parent")
		{
		    const char *nm = node->ToElement()->Attribute("name");		    
		    if (nm)
		    {
			sensor->parentName = string_utils::trim(nm);
			MARK_SET(node, sensor, parent);
		    }
		    else
		    {
			errorMessage("Parent name is not defined");
			errorLocation(node);
		    }		    
		}
		else if (node->ValueStr() == "origin")
		{
		    if (loadDoubleValues(node, 3, sensor->rpy, "rpy"))
			MARK_SET(node, sensor, rpy);
		    if (loadDoubleValues(node, 3, sensor->xyz, "xyz"))
			MARK_SET(node, sensor, xyz);
		}		
		else if (node->ValueStr() == "joint")
		{
		    if (sensor->joint == NULL)
			sensor->joint = new Sensor::Joint();
		    loadJoint(node, name + "_joint", sensor->joint);
		    MARK_SET(node, sensor, joint);
		}
		else if (node->ValueStr() == "collision")
		{
		    if (sensor->collision == NULL)
			sensor->collision = new Sensor::Collision();
		    loadCollision(node, name + "_collision", sensor->collision);
		    MARK_SET(node, sensor, collision);
		}		
		else if (node->ValueStr() == "inertial")
		{
		    if (sensor->inertial == NULL)
			sensor->inertial = new Sensor::Inertial();
		    loadInertial(node, name + "_inertial", sensor->inertial);
		    MARK_SET(node, sensor, inertial);
		}		
		else if (node->ValueStr() == "visual")
		{
		    if (sensor->visual == NULL)
			sensor->visual = new Sensor::Visual();
		    loadVisual(node, name + "_visual", sensor->visual);
		    MARK_SET(node, sensor, visual);
		}		
		else if (node->ValueStr() == "map")
		    loadMap(node, &sensor->data);
		else if (node->ValueStr() == "calibration")
		{
		    const char *filename = node->ToElement()->Attribute("filename");
		    if (filename)
		    {
			sensor->calibration = string_utils::trim(filename);		    
			MARK_SET(node, sensor, calibration);
		    }
		    else
		    {
			errorMessage("Calibration filename not set");
			errorLocation(node);			
		    }
		}		
		else
		    unknownNode(node); 
	    }
	    else
		unknownNode(node);
	}
	
	m_location = currentLocation;  
    }
    
    bool URDF::replaceIncludes(TiXmlElement *elem)
    {
	if (elem->ValueStr() == "include" && elem->FirstChild() && elem->FirstChild()->Type() == TiXmlNode::TEXT)
        {
            char* filename = findFile(elem->FirstChild()->Value());
	    bool change = false;
	    if (filename)
            {
                TiXmlDocument *doc = new TiXmlDocument(filename);
                if (doc->LoadFile())
                {
                    addPath(filename);
                    TiXmlNode *parent = elem->Parent();
                    if (parent)
		    {
			parent->ReplaceChild(dynamic_cast<TiXmlNode*>(elem), *doc->RootElement())->ToElement();
			change = true;
		    }
		}
                else
                    errorMessage("Unable to load " + std::string(filename));
		delete doc;
                free(filename);
	    }
            else
                errorMessage("Unable to find " + elem->FirstChild()->ValueStr());
	    if (change)
		return true;
        }
	
	bool restart = true;
	while (restart)
	{
	    restart = false;
	    for (TiXmlNode *child = elem->FirstChild() ; child ; child = child->NextSibling())
		if (child->Type() == TiXmlNode::ELEMENT)
		    if (replaceIncludes(child->ToElement()))
		    {
			restart = true;
			break;
		    }
	}
	return false;
	
    }

    void URDF::loadMap(const TiXmlNode *node, Map *data)
    {
	std::string name;
	std::string flag;
	
	for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
	{
	    if (strcmp(attr->Name(), "name") == 0)
		name = string_utils::trim(attr->ValueStr());
	    else
		if (strcmp(attr->Name(), "flag") == 0)
		    flag = string_utils::trim(attr->ValueStr());
	}
	
	for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
	    if (child->Type() == TiXmlNode::ELEMENT && child->ValueStr() == "elem")
	    {
		const char *key = child->ToElement()->Attribute("key");
		const char *value = child->ToElement()->Attribute("value");
		if (key)
		{	    
		    std::string valStr;
		    if (value)
			valStr = value;
		    else
		    {
			if (child->FirstChild() &&  child->FirstChild()->Type() == TiXmlNode::TEXT)
			    valStr = child->FirstChild()->ValueStr();
		    }
		    data->add(flag, name, key, valStr);
		}
		else
		{
		    errorMessage("No element key defined");
		    errorLocation(child);
		}		
	    }
	    else if (child->Type() == TiXmlNode::ELEMENT && child->ValueStr() == "verbatim")
	    {
		std::string key;
		bool includes = false;
		for (const TiXmlAttribute *attr = child->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
		{
		    if (strcmp(attr->Name(), "key") == 0)
			key = attr->ValueStr();
		    if (strcmp(attr->Name(), "includes") == 0)
			loadBoolValues(attr->ValueStr(), 1, &includes, child);
		}
		if (includes)
		    replaceIncludes(const_cast<TiXmlElement*>(child->ToElement()));
		data->add(flag, name, key, child->ToElement());
	    }
	    else
		unknownNode(child);
    }
    
    void URDF::linkDatastructure(void)
    {
	
	/* compute the proper pointers for parent nodes and children */
	for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	{
	    if (i->second->parentName.empty() || i->second->parentName == "world")
	    {
		m_linkRoots.push_back(i->second);
		continue;
	    }
	    if (i->second->joint->type == Link::Joint::FLOATING || i->second->joint->type == Link::Joint::PLANAR)
		errorMessage("Link '" + i->second->name + "' uses a free joint (floating or planar) but its parent is not the environment!");
	    if (m_links.find(i->second->parentName) == m_links.end())
		errorMessage("Parent of link '" + i->second->name + "' is undefined: '" + i->second->parentName + "'");
	    else
	    {
		Link *parent =  m_links[i->second->parentName];
		i->second->parent = parent;
		parent->children.push_back(i->second);
	    }
	}
	
	/* compute the pointers to links inside frames */
	for (std::map<std::string, Frame*>::iterator i = m_frames.begin() ; i != m_frames.end() ; i++)
	{
	    if (m_links.find(i->second->linkName) != m_links.end())
		i->second->link = m_links[i->second->linkName];
	    else
		errorMessage("Frame '" + i->first + "' refers to unknown link ('" + i->second->linkName + "')");	    
	}
	
	/* for each group, compute the pointers to the links they contain, and for every link,
	 * compute the list of pointers to the groups they are part of 
	 * do the same for frames */
	for (std::map<std::string, Group*>::iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	{
	    std::sort(i->second->linkNames.begin(), i->second->linkNames.end());
	    bool repeat = false;
	    for (unsigned int j = 0 ; j < i->second->linkNames.size() ; ++j)
	    {
		if (repeat)
		{
		    repeat = false;
		    j--;
		}
		
		if (m_links.find(i->second->linkNames[j]) == m_links.end())
		{
		    if (m_frames.find(i->second->linkNames[j]) == m_frames.end())
		    {
			errorMessage("Group '" + i->first + "': '" + i->second->linkNames[j] + "' is not defined as a link or frame");
			i->second->linkNames.erase(i->second->linkNames.begin() + j);
			repeat = true;
			continue;
		    }
		    else
		    {
			/* name is a frame */
			i->second->frameNames.push_back(i->second->linkNames[j]);
			Frame* f = m_frames[i->second->linkNames[j]];
			f->groups.push_back(i->second);
			i->second->frames.push_back(f);
		    }			    
		}
		else
		{
		    /* name is a link */
		    if (m_frames.find(i->second->linkNames[j]) != m_frames.end())
			errorMessage("Name '" + i->second->linkNames[j] + "' is used both for a link and a frame; defaulting to link");
		    
		    Link* l = m_links[i->second->linkNames[j]];
		    l->groups.push_back(i->second);
		    i->second->links.push_back(l);
		}
	    }
	    
	    /* remove the link names that are in fact frame names */
	    for (unsigned int j = 0 ; j < i->second->frameNames.size() ; ++j)
		for (unsigned int k = 0 ; k < i->second->linkNames.size() ; ++k)
		    if (i->second->linkNames[k] == i->second->frameNames[j])
		    {
			i->second->linkNames.erase(i->second->linkNames.begin() + k);
			break;
		    }
	}
	
	/* sort the links by name to reduce variance in the output of the parser */
	std::sort(m_linkRoots.begin(), m_linkRoots.end(), SortByName<Link>());
	for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	    std::sort(i->second->children.begin(), i->second->children.end(), SortByName<Link>());
	
	/* for every group, find the set of links that are roots in this group (their parent is not in the group) */
	for (std::map<std::string, Group*>::iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	{
	    for (unsigned int j = 0 ; j < i->second->links.size() ; ++j)
	    {
		Link *parent = i->second->links[j]->parent;
		bool outside = true;
		if (parent)
		    for (unsigned int k = 0 ; k < parent->groups.size() ; ++k)
			if (parent->groups[k] == i->second)
			{
			    outside = false;
			    break;
			}
		if (outside)
		    i->second->linkRoots.push_back(i->second->links[j]);
	    }
	    std::sort(i->second->linkRoots.begin(), i->second->linkRoots.end(), SortByName<Link>());
	}
	
	/* construct inGroup for every link */
	std::vector<std::string> grps;
	getGroupNames(grps);
	std::map<std::string, unsigned int> grpmap;
	for (unsigned int i = 0 ; i < grps.size() ; ++i)
	    grpmap[grps[i]] = i;
	
	for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	{
	    i->second->inGroup.resize(grps.size(), false);
	    for (unsigned int j = 0 ; j < i->second->groups.size() ; ++j)
		i->second->inGroup[grpmap[i->second->groups[j]->name]] = true;
	}

	/* construct the pointers for chains */
	for (std::map<std::string, Chain*>::iterator i = m_chains.begin() ; i != m_chains.end() ; i++)
	{
	    std::map<std::string, Link*>::const_iterator r = m_links.find(i->second->root);
	    std::map<std::string, Link*>::const_iterator t = m_links.find(i->second->tip);
	    if (r != m_links.end() && t  != m_links.end())
	    {
		std::vector<Link*> &clinks = i->second->links;
		clinks.push_back(r->second);
		clinks.push_back(t->second);
		
		while (clinks[0] != clinks[1]->parent && clinks[1]->parent != NULL)
		    clinks.insert(clinks.begin() + 1, clinks[1]->parent);
		
		if (clinks[0] != clinks[1]->parent)
		    errorMessage("Cannot reach link '" + t->first + "' from link '" + r->first + "' in chain '" + i->first + "'");
	    }
	    else
	    {
		if (r == m_links.end())
		    errorMessage("Root of chain '" + i->first + "' is not a link ('" + r->first + "')");
		if (t == m_links.end())
		    errorMessage("Tip of chain '" + i->first + "' is not a link ('" + t->first + "')");
	    }
	}
    }
    
    

    bool URDF::parse(const TiXmlNode *node)
    {
	if (!node) return false;
	
	int type =  node->Type();
	switch (type)
	{
	case TiXmlNode::DOCUMENT:
	    if (dynamic_cast<const TiXmlDocument*>(node)->RootElement()->ValueStr() != "robot")
		errorMessage("File '" + m_source + "' does not start with the <robot> tag");
	    
	    /* stage 1: extract templates, constants, groups */
	    parse(dynamic_cast<const TiXmlNode*>(dynamic_cast<const TiXmlDocument*>(node)->RootElement()));
	    
	    /* stage 2: parse the rest of the data (that depends on templates & constants) */
	    {		
		std::vector<const TiXmlElement*> m_stage3;
		for (unsigned int i = 0 ; i < m_stage2.size() ; ++i)
		{
		    const TiXmlElement *elem = m_stage2[i]->ToElement(); 
		    if (!elem)
			errorMessage("Non-element node found in second stage of parsing. This should NOT happen");
		    else
		    {
			std::string name = elem->ValueStr();
			
			if (name == "link")
			    loadLink(m_stage2[i]);
			else if (name == "sensor")
			    loadSensor(m_stage2[i]);
			else if (name == "actuator")
			    m_actuators.push_back(elem);
			else if (name == "transmission")
			    m_transmissions.push_back(elem);
			else if (name == "controller")
			    m_controllers.push_back(elem);
			else
			    m_stage3.push_back(elem);			
		    }
		}

		for (unsigned int i = 0 ; i < m_stage3.size() ; ++i)
		{
		    std::string name = m_stage3[i]->ValueStr();
		    
		    if (name == "frame")
			loadFrame(m_stage3[i]);
		    else if (name == "joint")
			loadJoint(m_stage3[i], "", NULL);
		    else if (name == "geometry")
			loadGeometry(m_stage3[i], "", NULL);
		    else if (name == "collision")
			loadCollision(m_stage3[i], "", NULL);
		    else if (name == "visual")
			loadVisual(m_stage3[i], "", NULL);
		    else if (name == "inertial")
			loadInertial(m_stage3[i], "", NULL);
		    else
			unknownNode(m_stage3[i]);
		}		
	    }
	    
	    /* stage 4: 'link' datastructures -- provide easy access pointers */
	    linkDatastructure();
	    
	    /* clear temporary data */
	    clearTemporaryData();
	    
	    break;
	case TiXmlNode::ELEMENT:
	    if (node->ValueStr() == "robot")
	    {
		const char *name = node->ToElement()->Attribute("name");
		if (name)
		{
		    std::string nameStr = string_utils::trim(name);
		    if (!m_name.empty() && nameStr != m_name)
			errorMessage("Loading a file with contradicting robot name: '" + m_name + "' - '" + name + "'");
		    m_name = nameStr;
		}
		for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
		    parse(child);
	    }
	    else
		if (node->ValueStr() == "include")
		{
		    if (node->Type() == TiXmlNode::ELEMENT && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		    {
			char* filename = findFile(node->FirstChild()->Value());
			if (filename)
			{
			    TiXmlDocument *doc = new TiXmlDocument(filename);
			    doc->SetUserData(reinterpret_cast<void*>(filename));
			    m_docs.push_back(doc);
			    if (doc->LoadFile())
			    {
				addPath(filename);
				if (doc->RootElement()->ValueStr() != "robot")
				    errorMessage("Included file '" + std::string(filename) + "' does not start with the <robot> tag");
				
				parse(dynamic_cast<const TiXmlNode*>(doc->RootElement()));
			    }
			    else
				errorMessage("Unable to load " + std::string(filename));
			}
			else
			    errorMessage("Unable to find " + node->FirstChild()->ValueStr());
		    }
		    else 
			unknownNode(node);
		}
		else if (node->ValueStr() == "const")
		{       
		    const char *name = node->ToElement()->Attribute("name");
		    const char *value = node->ToElement()->Attribute("value");
		    
		    if (!node->NoChildren())
		    {
			errorMessage("Constant '" + std::string(name) + "' appears to contain tags. This should not be the case.");
			errorLocation(node);
		    }
		    if (name && value)
			m_constants[string_utils::trim(name)] = value;
		}
		else
		    if (node->ValueStr() == "const_block")
		    {
			const char *name = node->ToElement()->Attribute("name");
			if (name)
			    m_constBlocks[string_utils::trim(name)] = node;
			else
			{
			    errorMessage("Undefined name for constant block");
			    errorLocation(node);
			}
		    }
		    else if (node->ValueStr() == "group")
		    {
			std::string group;
			std::string flags;
			
			for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
			{
			    if (strcmp(attr->Name(), "name") == 0)
				group = string_utils::trim(attr->ValueStr());
			    else
				if (strcmp(attr->Name(), "flags") == 0)
				    flags = attr->ValueStr();
			}
			Group *g = NULL;
			if (m_groups.find(group) == m_groups.end())
			{
			    g = new Group();
			    g->name = group;
			    m_groups[group] = g;
			}
			else
			    g = m_groups[group];
			
			std::stringstream ssflags(flags);
			while (ssflags.good())
			{
			    std::string flag;
			    ssflags >> flag;
			    g->flags.push_back(flag);	      
			}
			
			for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
			{
			    if (child->Type() == TiXmlNode::TEXT)
			    {
				std::stringstream ss(child->ValueStr());
				while (ss.good())
				{
				    std::string value; ss >> value;
				    g->linkNames.push_back(value);
				}				
			    }
			    else
			    {
				if (child->Type() == TiXmlNode::ELEMENT && child->ValueStr() == "map")
				    loadMap(child, &g->data);
				else
				    unknownNode(child);
			    }
			}
			
			if (g->linkNames.empty())
			{
			    errorMessage("Group '" + g->name + "' is empty. Not adding to list of groups.");
			    m_groups.erase(m_groups.find(g->name));
			    delete g;
			}			
		    }
		    else if (node->ValueStr() == "chain")
		    {
			std::string name;
			std::string root;
			std::string tip;
			
			for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
			{
			    if (strcmp(attr->Name(), "name") == 0)
				name = string_utils::trim(attr->ValueStr());
			    else
			    {
				if (strcmp(attr->Name(), "root") == 0)
				    root = attr->ValueStr();
				else
				    if (strcmp(attr->Name(), "tip") == 0)
					tip = attr->ValueStr();
			    }
			}
			if (name.empty())
			    name = root + "_2_" + tip;
			
			if (m_chains.find(name) != m_chains.end())
			    errorMessage("Chain '" + name + "' was previously defined. Not adding again.");
			else
			{
			    Chain *c = new Chain();
			    c->name = name;
			    c->root = root;
			    c->tip  = tip;
			    m_chains[name] = c;			    
			}
		    }
		    else if (node->ValueStr() == "map")
			loadMap(node, &m_data);
		    else
			m_stage2.push_back(node);
	    break;
	default:
	    unknownNode(node);
	}
	
	return true;
    }
    
    void URDF::sanityCheck(void) const
    {
	std::vector<Link*> links;
	getLinks(links);
	for (unsigned int i = 0 ; i < links.size() ; ++i)
	{
	    if (links[i]->isSet["joint"])
	    {
		Link::Joint *joint = links[i]->joint;
		if (joint->type == Link::Joint::UNKNOWN)
		    errorMessage("Joint '" + joint->name + "' in link '" + links[i]->name + "' is of unknown type");
		if (joint->type == Link::Joint::REVOLUTE || joint->type == Link::Joint::PRISMATIC)
		{
		    if (joint->axis[0] == 0.0 && joint->axis[1] == 0.0 && joint->axis[2] == 0.0)
			errorMessage("Joint '" + joint->name + "' in link '" + links[i]->name + "' does not seem to have its axis properly set");
		    if ((joint->isSet["limit"] || joint->type == Link::Joint::PRISMATIC))
		    {
			if (joint->limit[0] == 0.0 && joint->limit[1] == 0.0)
			    errorMessage("Joint '" + joint->name + "' in link '" + links[i]->name + "' does not seem to have its limits properly set (they are both 0)");
			else
			    if (joint->limit[0] > joint->limit[1])
				errorMessage("Joint '" + joint->name + "' in link '" + links[i]->name + "' does not seem to have its limits properly set (max < min)");
			    else
				if (joint->limit[0] + joint->safetyLength[0] > joint->limit[1] - joint->safetyLength[1])
				    errorMessage("Joint '" + joint->name + "' in link '" + links[i]->name + "' does not seem to have its limits properly set (max < min after safety length imposed)");
		    }
		}
	    }
	    else
		errorMessage("Link " + links[i]->name + " does not have its <joint> set");
	    
	    if (links[i]->isSet["collision"])
	    {
		if (links[i]->collision->isSet["geometry"])
		{
		    Link::Geometry *cgeom = links[i]->collision->geometry;
		    if (cgeom->type == Link::Geometry::UNKNOWN)
			errorMessage("Collision geometry '" + cgeom->name + "' in link '" + links[i]->name + "' is of unknown type");
		    else 
		    {
			if (cgeom->type == Link::Geometry::SPHERE)
			    if (static_cast<Link::Geometry::Sphere*>(cgeom->shape)->radius <= 0.0)
				errorMessage("Collision geometry '" + cgeom->name + "' in link '" + links[i]->name + "' does not seem to have its size properly set");
			if (cgeom->type == Link::Geometry::CYLINDER)
			    if (static_cast<Link::Geometry::Cylinder*>(cgeom->shape)->radius <= 0.0 || static_cast<Link::Geometry::Cylinder*>(cgeom->shape)->length <= 0.0)
				errorMessage("Collision geometry '" + cgeom->name + "' in link '" + links[i]->name + "' does not seem to have its size properly set");
			if (cgeom->type == Link::Geometry::BOX)
			{
			    const double *size = static_cast<Link::Geometry::Box*>(cgeom->shape)->size;
			    if (size[0] <= 0.0 || size[1] <= 0.0 || size[2] <= 0.0)
				errorMessage("Collision geometry '" + cgeom->name + "' in link '" + links[i]->name + "' does not seem to have its size properly set");
			}			
		    }
		}
		else
		    errorMessage("Collision " + links[i]->collision->name + " does not have its <geometry> set");
	    }
	    else
		errorMessage("Link " + links[i]->name + " does not have its <collision> set");
	    
	    if (links[i]->isSet["visual"])
	    {
		if (links[i]->visual->isSet["geometry"])
		{
		    Link::Geometry *vgeom = links[i]->visual->geometry;
		    if (vgeom->type == Link::Geometry::UNKNOWN)
			errorMessage("Visual geometry '" + vgeom->name + "' in link '" + links[i]->name + "' is of unknown type");
		    else
		    {
			if (vgeom->type == Link::Geometry::SPHERE)
			    if (static_cast<Link::Geometry::Sphere*>(vgeom->shape)->radius <= 0.0)
				errorMessage("Visual geometry '" + vgeom->name + "' in link '" + links[i]->name + "' does not seem to have its size properly set");
			if (vgeom->type == Link::Geometry::CYLINDER)
			    if (static_cast<Link::Geometry::Cylinder*>(vgeom->shape)->radius <= 0.0 || static_cast<Link::Geometry::Cylinder*>(vgeom->shape)->length <= 0.0)
				errorMessage("Visual geometry '" + vgeom->name + "' in link '" + links[i]->name + "' does not seem to have its size properly set");
			if (vgeom->type == Link::Geometry::BOX)
			{
			    const double *size = static_cast<Link::Geometry::Box*>(vgeom->shape)->size;
			    if (size[0] <= 0.0 || size[1] <= 0.0 || size[2] <= 0.0)
				errorMessage("Visual geometry '" + vgeom->name + "' in link '" + links[i]->name + "' does not seem to have its size properly set");
			}
		    }
		}
		else
		    errorMessage("Visual " + links[i]->visual->name + " does not have its <geometry> set");  
	    }
	    
	    if (!links[i]->isSet["inertial"])
		errorMessage("Link " + links[i]->name + " does not have its <inertial> set");	
	}
    }
    
    
} // namespace robot_desc
