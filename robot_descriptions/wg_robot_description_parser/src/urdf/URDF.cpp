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

#include <urdf/URDF.h>
#include <math_utils/MathExpression.h>
#include <cstring>
#include <fstream>
#include <sstream>
#include <queue>

namespace robot_desc {
    
    /** Macro to mark the fact a certain member variable was set. Also
	prints a warning if the same member was set multiple times. */
#define MARK_SET(owner, variable)					\
    {									\
	if (owner->isSet[#variable]) {					\
	    fprintf(stderr, "'%s' already set\n", #variable);		\
	    errorLocation(); }						\
	else								\
	    owner->isSet[#variable] = true;				\
    }
    

    /** Operator for sorting objects by name */
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
	for (std::map<std::string, Actuator*>::iterator i = m_actuators.begin() ; i != m_actuators.end() ; i++)
	    delete i->second;
	for (std::map<std::string, Transmission*>::iterator i = m_transmissions.begin() ; i != m_transmissions.end() ; i++)
	    delete i->second;
	for (std::map<std::string, Frame*>::iterator i = m_frames.begin() ; i != m_frames.end() ; i++)
	    delete i->second;
    }
    
    void URDF::clear(void)
    {
	freeMemory();
	m_name.clear();
	m_source.clear();
	m_location.clear();
	m_links.clear();
	m_actuators.clear();
	m_transmissions.clear();
	m_frames.clear();
	m_groups.clear();
	m_paths.clear();
	m_paths.push_back("");
	m_linkRoots.clear();
	clearTemporaryData();
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
    
    URDF::Actuator* URDF::getActuator(const std::string &name) const
    {
	std::map<std::string, Actuator*>::const_iterator it = m_actuators.find(name);
	return it == m_actuators.end() ? NULL : it->second;
    }

    void URDF::getActuators(std::vector<Actuator*> &actuators) const
    {
	std::vector<Actuator*> localActuators;
	for (std::map<std::string, Actuator*>::const_iterator i = m_actuators.begin() ; i != m_actuators.end() ; i++)
	    localActuators.push_back(i->second);
	std::sort(localActuators.begin(), localActuators.end(), SortByName<Actuator>());
	actuators.insert(actuators.end(), localActuators.begin(), localActuators.end());
    }

    URDF::Transmission* URDF::getTransmission(const std::string &name) const
    {
	std::map<std::string, Transmission*>::const_iterator it = m_transmissions.find(name);
	return it == m_transmissions.end() ? NULL : it->second;
    }
    
    void URDF::getTransmissions(std::vector<Transmission*> &transmissions) const
    {
	std::vector<Transmission*> localTransmissions;
	for (std::map<std::string, Transmission*>::const_iterator i = m_transmissions.begin() ; i != m_transmissions.end() ; i++)
	    localTransmissions.push_back(i->second);
	std::sort(localTransmissions.begin(), localTransmissions.end(), SortByName<Transmission>());
	transmissions.insert(transmissions.end(), localTransmissions.begin(), localTransmissions.end());
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
    
    const URDF::Data& URDF::getData(void) const
    {
	return m_data;
    }
    
    void URDF::Data::add(const std::string &type, const std::string &name, const std::string &key, const std::string &value)
    {
	m_data[type][name][key].str = value;
    }
    
    void URDF::Data::add(const std::string &type, const std::string &name, const std::string &key, const TiXmlElement *value)
    {
	m_data[type][name][key].xml = value;
    }
    
    bool URDF::Data::hasDefault(const std::string &key) const
    {
	std::map<std::string, std::string> m = getDataTagValues("", "");
	return m.find(key) != m.end();
    }
    
    std::string URDF::Data::getDefaultValue(const std::string &key) const
    {
	return getDataTagValues("", "")[key];
    }
    
    const TiXmlElement* URDF::Data::getDefaultXML(const std::string &key) const
    {
	return getDataTagXML("", "")[key];
    }
    
    void URDF::Data::getDataTagTypes(std::vector<std::string> &types) const
    {
	for (std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator i = m_data.begin() ; i != m_data.end() ; ++i)
	    types.push_back(i->first);
    }
    
    void URDF::Data::getDataTagNames(const std::string &type, std::vector<std::string> &names) const
    {
	std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator pos = m_data.find(type);
	if (pos != m_data.end())
	{
	    for (std::map<std::string, std::map<std::string, Element > >::const_iterator i = pos->second.begin() ; i != pos->second.end() ; ++i)
		names.push_back(i->first);
	}
    }
    std::map<std::string, std::string> URDF::Data::getDataTagValues(const std::string &type, const std::string &name) const
    {    
	std::map<std::string, std::string> result;
	
	std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator pos = m_data.find(type);
	if (pos != m_data.end())
	{
	    std::map<std::string, std::map<std::string, Element > >::const_iterator m = pos->second.find(name);
	    if (m != pos->second.end())
	    {
		for (std::map<std::string, Element>::const_iterator it = m->second.begin() ; it != m->second.end() ; it++)
		    result[it->first] = it->second.str;            
	    }
	}
	return result;
    }
    
    std::map<std::string, const TiXmlElement*> URDF::Data::getDataTagXML(const std::string &type, const std::string &name) const
    {    
	std::map<std::string, const TiXmlElement*> result;
	
	std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator pos = m_data.find(type);
	if (pos != m_data.end())
	{
	    std::map<std::string, std::map<std::string, Element > >::const_iterator m = pos->second.find(name);
	    if (m != pos->second.end())
	    {
		for (std::map<std::string, Element>::const_iterator it = m->second.begin() ; it != m->second.end() ; it++)
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
    
    void URDF::print(FILE *out) const
    {
	fprintf(out, "\nList of root links in robot '%s' (%u):\n", m_name.c_str(), m_linkRoots.size());
	for (unsigned int i = 0 ; i < m_linkRoots.size() ; ++i)
	    m_linkRoots[i]->print(out, "  ");
	fprintf(out, "\n");
	fprintf(out, "Frames:\n");
	for (std::map<std::string, Frame*>::const_iterator i = m_frames.begin() ; i != m_frames.end() ; i++)
	    i->second->print(out, "  ");
	fprintf(out, "\n");
	fprintf(out, "Actuators:\n");
	for (std::map<std::string, Actuator*>::const_iterator i = m_actuators.begin() ; i != m_actuators.end() ; i++)
	    i->second->print(out, "  "); 
	fprintf(out, "\n");   
	fprintf(out, "Transmissions:\n");
	for (std::map<std::string, Transmission*>::const_iterator i = m_transmissions.begin() ; i != m_transmissions.end() ; i++)
	    i->second->print(out, "  ");
	fprintf(out, "\n");
	fprintf(out, "Data types:\n");
	m_data.print(out, "  ");
    }
    
    void URDF::Data::print(FILE *out, std::string indent) const
    {
	for (std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator i = m_data.begin() ; i != m_data.end() ; ++i)
	{
	    fprintf(out, "%sData of type '%s':\n", indent.c_str(), i->first.c_str());
	    for (std::map<std::string, std::map<std::string, Element > >::const_iterator j = i->second.begin() ; j != i->second.end() ; ++j)
	    {
		fprintf(out, "%s  [%s]\n", indent.c_str(), j->first.c_str());
		for (std::map<std::string, Element>::const_iterator k = j->second.begin() ; k != j->second.end() ; ++k)
		    fprintf(out, "%s    %s = %s%s\n", indent.c_str(), k->first.c_str(), k->second.str.c_str(), k->second.xml != NULL ? " [XML]" : "");
	    }
	}
    }
   
    void URDF::Frame::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sFrame [%s]:\n", indent.c_str(), name.c_str());
	fprintf(out, "%s  - type: %d\n", indent.c_str(), (int)type);
	fprintf(out, "%s  - rpy: (%f, %f, %f)\n", indent.c_str(), rpy[0], rpy[1], rpy[2]);
	fprintf(out, "%s  - xyz: (%f, %f, %f)\n", indent.c_str(), xyz[0], xyz[1], xyz[2]);
	fprintf(out, "%s  - link: %s\n", indent.c_str(), link ? link->name.c_str() : "");
	fprintf(out, "%s  - groups: ", indent.c_str());
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{      
	    fprintf(out, "%s ( ", groups[i]->name.c_str());
	    for (unsigned int j = 0 ; j < groups[i]->flags.size() ; ++j)
		fprintf(out, "%s ", groups[i]->flags[j].c_str());
	    fprintf(out, ") ");
	}
	data.print(out, indent + "  ");
    }
    
    void URDF::Transmission::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sTransmission [%s]:\n", indent.c_str(), name.c_str());
	data.print(out, indent + "  ");
    }
    
    void URDF::Actuator::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sActuator [%s]:\n", indent.c_str(), name.c_str());
	data.print(out, indent + "  ");
    }
     
    void URDF::Link::Geometry::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sGeometry [%s]:\n", indent.c_str(), name.c_str());
	fprintf(out, "%s  - type: %d\n", indent.c_str(), (int)type);
	fprintf(out, "%s  - size: ( ", indent.c_str());
	for (int i = 0 ; i < nsize ; ++i)
	    fprintf(out, "%f ", size[i]);
	fprintf(out, ")\n");
	fprintf(out, "%s  - filename: %s\n", indent.c_str(), filename.c_str());
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Joint::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sJoint [%s]:\n", indent.c_str(), name.c_str());
	fprintf(out, "%s  - type: %d\n", indent.c_str(), (int)type);
	fprintf(out, "%s  - axis: (%f, %f, %f)\n", indent.c_str(), axis[0], axis[1], axis[2]);
	fprintf(out, "%s  - anchor: (%f, %f, %f)\n", indent.c_str(), anchor[0], anchor[1], anchor[2]);
	fprintf(out, "%s  - limit: (%f, %f)\n", indent.c_str(), limit[0], limit[1]);
	fprintf(out, "%s  - calibration: %s\n", indent.c_str(), calibration.c_str());
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Collision::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sCollision [%s]:\n", indent.c_str(), name.c_str());
	fprintf(out, "%s  - verbose: %s\n", indent.c_str(), verbose ? "Yes" : "No");
	fprintf(out, "%s  - material: %s\n", indent.c_str(), material.c_str());
	fprintf(out, "%s  - rpy: (%f, %f, %f)\n", indent.c_str(), rpy[0], rpy[1], rpy[2]);
	fprintf(out, "%s  - xyz: (%f, %f, %f)\n", indent.c_str(), xyz[0], xyz[1], xyz[2]);
	geometry->print(out, indent + "  ");
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Inertial::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sInertial [%s]:\n", indent.c_str(), name.c_str());
	fprintf(out, "%s  - mass: %f\n", indent.c_str(), mass);
	fprintf(out, "%s  - com: (%f, %f, %f)\n", indent.c_str(), com[0], com[1], com[2]);
	fprintf(out, "%s  - inertia: (%f, %f, %f, %f, %f, %f)\n", indent.c_str(), 
		inertia[0], inertia[1], inertia[2], inertia[3], inertia[4],  inertia[5]);
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::Visual::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sVisual [%s]:\n", indent.c_str(), name.c_str());
	fprintf(out, "%s  - material: %s\n", indent.c_str(), material.c_str());
	fprintf(out, "%s  - scale: (%f, %f, %f)\n", indent.c_str(), scale[0], scale[1], scale[2]);
	fprintf(out, "%s  - rpy: (%f, %f, %f)\n", indent.c_str(), rpy[0], rpy[1], rpy[2]);
	fprintf(out, "%s  - xyz: (%f, %f, %f)\n", indent.c_str(), xyz[0], xyz[1], xyz[2]);
	geometry->print(out, indent + "  ");
	data.print(out, indent + "  ");
    }
    
    void URDF::Link::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sLink [%s]:\n", indent.c_str(), name.c_str());
	fprintf(out, "%s  - parent link: %s\n", indent.c_str(), parentName.c_str());
	fprintf(out, "%s  - rpy: (%f, %f, %f)\n", indent.c_str(), rpy[0], rpy[1], rpy[2]);
	fprintf(out, "%s  - xyz: (%f, %f, %f)\n", indent.c_str(), xyz[0], xyz[1], xyz[2]);
	joint->print(out, indent+ "  ");
	collision->print(out, indent+ "  ");
	inertial->print(out, indent+ "  ");
	visual->print(out, indent+ "  ");
	fprintf(out, "%s  - groups: ", indent.c_str());
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{      
	    fprintf(out, "%s ( ", groups[i]->name.c_str());
	    for (unsigned int j = 0 ; j < groups[i]->flags.size() ; ++j)
		fprintf(out, "%s ", groups[i]->flags[j].c_str());
	    fprintf(out, ") ");
	}
	fprintf(out, "\n");
	fprintf(out, "%s  - children links: ", indent.c_str());
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	    fprintf(out, "%s ", children[i]->name.c_str());
	fprintf(out, "\n");
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	    children[i]->print(out, indent + "  ");
	data.print(out, indent + "  ");
    }
    
    void URDF::Sensor::print(FILE *out, std::string indent) const
    {
	fprintf(out, "%sSensor:\n", indent.c_str());
	fprintf(out, "%s  - type: %d\n", indent.c_str(), (int)type);
	fprintf(out, "%s  - calibration: %s\n", indent.c_str(), calibration.c_str());
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
    
    void URDF::errorLocation(void) const
    {
	if (!m_location.empty())
	    fprintf(stderr, "  ... at %s\n", m_location.c_str());  
    }
    
    void URDF::ignoreNode(const TiXmlNode* node)
    {
	switch (node->Type())
	{
	case TiXmlNode::ELEMENT:
	    fprintf(stderr, "Ignoring element node '%s'\n", node->Value());
	    errorLocation();  
	    break;
	case TiXmlNode::TEXT:
	    fprintf(stderr, "Ignoring text node with content '%s'\n", node->Value());
	    errorLocation();  
	    break;
	case TiXmlNode::COMMENT:
	case TiXmlNode::DECLARATION:
	    break;            
	case TiXmlNode::UNKNOWN:
	default:
	    fprintf(stderr, "Ignoring unknown node '%s'\n", node->Value());
	    errorLocation();  
	    break;
	}
    }
    
    void URDF::getChildrenAndAttributes(const TiXmlNode *node, std::vector<const TiXmlNode*> &children, std::vector<const TiXmlAttribute*> &attributes) const
    {
	for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
	    children.push_back(child);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
	    {
		if (strcmp(attr->Name(), "clone") == 0)
		{
		    std::map<std::string, const TiXmlNode*>::const_iterator pos = m_templates.find(attr->ValueStr());
		    if (pos == m_templates.end())
		    {
			fprintf(stderr, "Template '%s' is not defined\n", attr->Value());
			errorLocation();
		    }	
		    else
			getChildrenAndAttributes(pos->second, children, attributes);
		}
		else
		    attributes.push_back(attr);
	    }
	}
    }
    
    void URDF::defaultConstants(void)
    {
	m_constants["M_PI"] = "3.14159265358979323846";
    }
    
    void URDF::clearDocs(void)
    {
	/* clear memory allocated for loaded documents */
	for (unsigned int i = 0 ; i < m_docs.size() ; ++i)
	    delete m_docs[i];
	m_docs.clear();
    }
    
    void URDF::clearTemporaryData(void)
    {	
	m_collision.clear();
	m_joints.clear();
	m_inertial.clear();
	m_visual.clear();
	m_geoms.clear();
	
	m_constants.clear();
	m_templates.clear();
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
	m_docs.push_back(doc);
	if (doc->Parse(data))
	{
	    defaultConstants();
	    result = parse(dynamic_cast<const TiXmlNode*>(doc));
	}
	else
	    fprintf(stderr, "%s\n", doc->ErrorDesc());
	
	return result;
    }  
    
    bool URDF::loadFile(FILE *file)
    {
	clear();
	bool result = false;
	
	TiXmlDocument *doc = new TiXmlDocument();
	m_docs.push_back(doc);
	if (doc->LoadFile(file))
	{
	    defaultConstants();
	    result = parse(dynamic_cast<const TiXmlNode*>(doc));
	}
	else
	    fprintf(stderr, "%s\n", doc->ErrorDesc());
	
	return result;
    }
    
    bool URDF::loadFile(const char *filename)
    {
	clear();
	bool result = false;
	m_source = filename;
	
	TiXmlDocument *doc = new TiXmlDocument(filename);
	m_docs.push_back(doc);
	if (doc->LoadFile())
	{
	    addPath(filename);
	    defaultConstants();
	    result = parse(dynamic_cast<const TiXmlNode*>(doc));
	}
	else
	    fprintf(stderr, "%s\n", doc->ErrorDesc());
	
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
	for (unsigned int i = 0 ; i < m_paths.size() ; ++i)
	{
	    std::string name = m_paths[i] + filename;
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
		name = attributes[i]->ValueStr();
		attributes.erase(attributes.begin() + i);
		break;
	    }
	}
	return name;
    }
    
    struct getConstantData
    {
	std::map<std::string, std::string> *m;
	std::map<std::string, bool>         s;
    };
    
    static double getConstant(void *data, std::string &name)
    {
	getConstantData *d = reinterpret_cast<getConstantData*>(data);
	std::map<std::string, std::string> *m = d->m;
	if (m->find(name) == m->end())
	{
	    fprintf(stderr, "Request for undefined constant: '%s'\n", name.c_str());
	    return 0.0;
	}
	else
	{
	    if (meval::ContainsOperators((*m)[name]))
	    {
		std::map<std::string, bool>::iterator pos = d->s.find((*m)[name]);
		if (pos != d->s.end() && pos->second == true)
		{
		    fprintf(stderr, "Recursive definition of constant '%s'\n", name.c_str());
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
    
    unsigned int URDF::loadDoubleValues(const TiXmlNode *node, unsigned int count, double *vals)
    {
	if (node && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
	    node = node->FirstChild();
	else
	    return 0;
	
	std::stringstream ss(node->ValueStr());
	unsigned int read = 0;
	
	for (unsigned int i = 0 ; ss.good() && i < count ; ++i)
	{
	    std::string value;
	    ss >> value;
	    getConstantData data;
	    data.m = &m_constants;
	    vals[i] = meval::EvaluateMathExpression(value, &getConstant, reinterpret_cast<void*>(&data));
	    read++;
	}

	if (ss.good())
	{
	    std::string extra = ss.str();
	    while (!extra.empty())
	    {
		char last = extra[extra.size() - 1];
		if (last == ' ' || last == '\n' || last == '\t')
		    extra.erase(extra.size() - 1);
		else
		    break;	  
	    }
	    if (!extra.empty())
	    {
		fprintf(stderr, "More data available (%u read, rest is ignored): '%s'\n", read, node->Value());		
		errorLocation();		
	    }	    
	}
	
	if (read != count)
	{
	    fprintf(stderr, "Not all values were read: '%s'\n", node->Value());
	    errorLocation();
	}  
	
	return read;
    }
    
    unsigned int URDF::loadBoolValues(const TiXmlNode *node, unsigned int count, bool *vals)
    {
	if (node && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
	    node = node->FirstChild();
	else
	    return 0;
	
	std::stringstream ss(node->ValueStr());
	unsigned int read = 0;
	
	for (unsigned int i = 0 ; ss.good() && i < count ; ++i)
	{
	    std::string value;
	    ss >> value;
	    const unsigned int length = value.length();
	    for(unsigned int j = 0 ; j != length ; ++j)
		value[j] = std::tolower(value[j]);        
	    vals[i] = (value == "true" || value == "yes" || value == "1");
	    read++;
	}

	if (ss.good())
	{
	    std::string extra = ss.str();
	    while (!extra.empty())
	    {
		char last = extra[extra.size() - 1];
		if (last == ' ' || last == '\n' || last == '\t')
		    extra.erase(extra.size() - 1);
		else
		    break;	  
	    }
	    if (!extra.empty())
	    {
		fprintf(stderr, "More data available (%u read, rest is ignored): '%s'\n", read, node->Value());		
		errorLocation();		
	    }	    
	}
	
	if (read != count)
	{
	    fprintf(stderr, "Not all values were read: '%s'\n", node->Value());
	    errorLocation();  
	}
	
	return read;
    }
    
    void URDF::loadTransmission(const TiXmlNode *node)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, "");    
	Transmission *transmission = (m_transmissions.find(name) != m_transmissions.end()) ? m_transmissions[name] : new Transmission();
	transmission->name = name;
	if (transmission->name.empty())
	    fprintf(stderr, "No transmission name given\n");
	else
	    MARK_SET(transmission, name);
	
	m_transmissions[transmission->name] = transmission;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "data")
		    loadData(node, &transmission->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
	}    
    }
    
    void URDF::loadActuator(const TiXmlNode *node)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, "");    
	Actuator *actuator = (m_actuators.find(name) != m_actuators.end()) ? m_actuators[name] : new Actuator();
	actuator->name = name;
	if (actuator->name.empty())
	    fprintf(stderr, "No actuator name given\n");
	else
	    MARK_SET(actuator, name);
	
	m_actuators[actuator->name] = actuator;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "data")
		    loadData(node, &actuator->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
	}    
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
	    fprintf(stderr, "No frame name given\n");
	else
	    MARK_SET(frame, name);
	
	m_frames[frame->name] = frame;
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "rpy")
		{
		    loadDoubleValues(node, 3, frame->rpy);
		    MARK_SET(frame, rpy);		    
		}		
		else if (node->ValueStr() == "xyz")
		{
		    loadDoubleValues(node, 3, frame->xyz);
		    MARK_SET(frame, xyz);
		}
		else if (node->ValueStr() == "parent" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    frame->linkName = node->FirstChild()->ValueStr();
		    MARK_SET(frame, parent);
		    if (frame->type == Frame::CHILD)
			fprintf(stderr, "Frame '%s' can only have either a child or a parent link\n", frame->name.c_str());
		    frame->type = Frame::PARENT;
		}
		else if (node->ValueStr() == "child" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    frame->linkName = node->FirstChild()->ValueStr();
		    MARK_SET(frame, child);
		    if (frame->type == Frame::PARENT)
			fprintf(stderr, "Frame '%s' can only have either a child or a parent link\n", frame->name.c_str());
		    frame->type = Frame::CHILD;
		}
		else
		if (node->ValueStr() == "data")
		    loadData(node, &frame->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
	}    
    }
    
    void URDF::loadJoint(const TiXmlNode *node, const std::string& defaultName, Link::Joint *joint)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (joint && !name.empty())
	    MARK_SET(joint, name);
	
	if (!joint)
	{
	    if (m_joints.find(name) == m_joints.end())
	    {
		fprintf(stderr, "Attempting to add information to an undefined joint: '%s'\n", name.c_str());
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
		    fprintf(stderr, "Unknown joint type: '%s'\n", attr->Value());
		    errorLocation();
		}
		MARK_SET(joint, type);
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
		    loadDoubleValues(node, 3, joint->axis);
		    MARK_SET(joint, axis);
		}		
		else if (node->ValueStr() == "anchor" && !free)
		{
		    loadDoubleValues(node, 3, joint->anchor);
		    MARK_SET(joint, anchor);
		}		
		else if (node->ValueStr() == "limit" && !free)
		{
		    loadDoubleValues(node, 2, joint->limit);
		    MARK_SET(joint, limit);
		}		
		else if (node->ValueStr() == "calibration" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    joint->calibration = node->FirstChild()->ValueStr();
		    MARK_SET(joint, calibration);		    
		}	
		else if (node->ValueStr() == "data")
		    loadData(node, &joint->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
	}
	
    }
    
    void URDF::loadGeometry(const TiXmlNode *node, const std::string &defaultName, Link::Geometry *geometry)
    {
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (geometry && !name.empty())
	    MARK_SET(geometry, name);

	if (!geometry)
	{
	    if (m_geoms.find(name) == m_geoms.end())
	    {
		fprintf(stderr, "Attempting to add information to an undefined geometry: '%s'\n", name.c_str());
		return;
	    }
	    else
		geometry = m_geoms[name];
	}
	
	geometry->name = name;
	m_geoms[name] = geometry;
	
	for (unsigned int i = 0 ; i < attributes.size() ; ++i)
	{
	    const TiXmlAttribute *attr = attributes[i];
	    if (strcmp(attr->Name(), "type") == 0)
	    {
		if (attr->ValueStr() == "box")
		{
		    geometry->type = Link::Geometry::BOX;
		    geometry->nsize = 3;	  
		}      
		else if (attr->ValueStr() == "cylinder")
		{
		    geometry->type = Link::Geometry::CYLINDER;
		    geometry->nsize = 2;	  
		}
		else if (attr->ValueStr() == "sphere")
		{
		    geometry->type = Link::Geometry::SPHERE;
		    geometry->nsize = 1;	  
		}
		else if (attr->ValueStr() == "mesh")
		{
		    geometry->type = Link::Geometry::MESH;
		    geometry->nsize = 3;
		}      
		else
		{
		    fprintf(stderr, "Unknown geometry type: '%s'\n", attr->Value());
		    errorLocation();
		}
		MARK_SET(geometry, type);
	    }
	}
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "size")
		{
		    if (geometry->nsize > 0)
			loadDoubleValues(node, geometry->nsize, geometry->size);
		    else
			ignoreNode(node);
		    MARK_SET(geometry, size);
		}
		else if (node->ValueStr() == "data")
		    loadData(node, &geometry->data);
		else if (node->ValueStr() == "filename" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    geometry->filename = node->FirstChild()->ValueStr();
		    MARK_SET(geometry, filename);
		}		
		else                
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
	}
    }
    
    void URDF::loadCollision(const TiXmlNode *node, const std::string &defaultName, Link::Collision *collision)
    {  
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (collision && !name.empty())
	    MARK_SET(collision, name);
	
	if (!collision)
	{
	    if (m_collision.find(name) == m_collision.end())
	    {
		fprintf(stderr, "Attempting to add information to an undefined collision: '%s'\n", name.c_str());
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
		if (node->ValueStr() == "rpy")
		{
		    loadDoubleValues(node, 3, collision->rpy);
		    MARK_SET(collision, rpy);		    
		}		
		else if (node->ValueStr() == "xyz")
		{
		    loadDoubleValues(node, 3, collision->xyz);
		    MARK_SET(collision, xyz);
		}		
		else if (node->ValueStr() == "verbose")
		{
		    loadBoolValues(node, 1, &collision->verbose);
		    MARK_SET(collision, verbose);
		}		
		else if (node->ValueStr() == "material" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    collision->material = node->FirstChild()->ValueStr();
		    MARK_SET(collision, material);
		}		
		else if (node->ValueStr() == "geometry")
		{
		    loadGeometry(node, name + "_geom", collision->geometry);
		    MARK_SET(collision, geometry);		    
		}		
		else if (node->ValueStr() == "data")
		    loadData(node, &collision->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
	}    
    }
    
    void URDF::loadVisual(const TiXmlNode *node, const std::string &defaultName, Link::Visual *visual)
    {  
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (visual && !name.empty())
	    MARK_SET(visual, name);

	if (!visual)
	{
	    if (m_visual.find(name) == m_visual.end())
	    {
		fprintf(stderr, "Attempting to add information to an undefined visual: '%s'\n", name.c_str());
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
		if (node->ValueStr() == "rpy")
		{
		    loadDoubleValues(node, 3, visual->rpy);
		    MARK_SET(visual, rpy);
		}
		else if (node->ValueStr() == "xyz")
		{
		    loadDoubleValues(node, 3, visual->xyz);
		    MARK_SET(visual, xyz);		    
		}		
		else if (node->ValueStr() == "scale")
		{
		    loadDoubleValues(node, 3, visual->scale);
		    MARK_SET(visual, scale);
		}		
		else if (node->ValueStr() == "material" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    visual->material = node->FirstChild()->ValueStr();
		    MARK_SET(visual, material);
		}		
		else if (node->ValueStr() == "geometry")
		{
		    loadGeometry(node, name + "_geom", visual->geometry);
		    MARK_SET(visual, geometry);
		}
		else if (node->ValueStr() == "data")
		    loadData(node, &visual->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
	}   
    }
    
    void URDF::loadInertial(const TiXmlNode *node, const std::string &defaultName, Link::Inertial *inertial)
    { 
	std::vector<const TiXmlNode*> children;
	std::vector<const TiXmlAttribute*> attributes;
	getChildrenAndAttributes(node, children, attributes);
	
	std::string name = extractName(attributes, defaultName);
	if (inertial && !name.empty())
	    MARK_SET(inertial, name);
	
	if (!inertial)
	{
	    if (m_inertial.find(name) == m_inertial.end())
	    {
		fprintf(stderr, "Attempting to add information to an undefined inertial component: '%s'\n", name.c_str());
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
		    loadDoubleValues(node, 1, &inertial->mass);
		    MARK_SET(inertial, mass);		    
		}		
		else if (node->ValueStr() == "com")
		{
		    loadDoubleValues(node, 3, inertial->com);
		    MARK_SET(inertial, com);
		}
		else if (node->ValueStr() == "inertia")
		{
		    loadDoubleValues(node, 6, inertial->inertia);
		    MARK_SET(inertial, inertia);		    
		}		
		else if (node->ValueStr() == "data")
		    loadData(node, &inertial->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
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
	    fprintf(stderr, "No link name given\n");
	else
	    MARK_SET(link, name);
	m_links[link->name] = link;
	
	if (link->canSense())
	    fprintf(stderr, "Link '%s' was already defined as a sensor\n", link->name.c_str());

	std::string currentLocation = m_location;
	m_location = "link '" + name + "'";
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "parent" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    link->parentName = node->FirstChild()->ValueStr();
		    MARK_SET(link, parent);
		}		
		else if (node->ValueStr() == "rpy")
		{
		    loadDoubleValues(node, 3, link->rpy);
		    MARK_SET(link, rpy);
		}		
		else if (node->ValueStr() == "xyz")
		{
		    loadDoubleValues(node, 3, link->xyz);
		    MARK_SET(link, xyz);
		}		
		else if (node->ValueStr() == "joint")
		{
		    loadJoint(node, name + "_joint", link->joint);
		    MARK_SET(link, joint);
		}		
		else if (node->ValueStr() == "collision")
		{
		    loadCollision(node, name + "_collision", link->collision);
		    MARK_SET(link, collision);
		}
		else if (node->ValueStr() == "inertial")
		{
		    loadInertial(node, name + "_inertial", link->inertial);
		    MARK_SET(link, inertial);
		}		
		else if (node->ValueStr() == "visual")
		{
		    loadVisual(node, name + "_visual", link->visual);
		    MARK_SET(link, visual);
		}		
		else if (node->ValueStr() == "data")
		    loadData(node, &link->data);
		else
		    ignoreNode(node);
	    }
	    else
		ignoreNode(node);
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
	    fprintf(stderr, "Link with name '%s' has already been defined\n", name.c_str());
	    sensor = new Sensor();
	}
	
	sensor->name = name;
	if (sensor->name.empty())
	    fprintf(stderr, "No sensor name given\n");
	else
	    MARK_SET(sensor, name);
	if (m_links.find(sensor->name) != m_links.end())
	    fprintf(stderr, "Sensor '%s' redefined\n", sensor->name.c_str());
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
		    fprintf(stderr, "Unknown sensor type: '%s'\n", attr->Value());
		    errorLocation();
		}
		MARK_SET(sensor, type);
	    }
	}
	
	for (unsigned int i = 0 ; i < children.size() ; ++i)
	{
	    const TiXmlNode *node = children[i];
	    if (node->Type() == TiXmlNode::ELEMENT)
	    {
		if (node->ValueStr() == "parent" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    sensor->parentName = node->FirstChild()->ValueStr();
		    MARK_SET(sensor, parent);		    
		}		
		else if (node->ValueStr() == "rpy")
		{
		    loadDoubleValues(node, 3, sensor->rpy);
		    MARK_SET(sensor, rpy);		    
		}		
		else if (node->ValueStr() == "xyz")
		{
		    loadDoubleValues(node, 3, sensor->xyz);
		    MARK_SET(sensor, xyz);		    
		}		
		else if (node->ValueStr() == "joint")
		{
		    loadJoint(node, name + "_joint", sensor->joint);
		    MARK_SET(sensor, joint);
		}
		else if (node->ValueStr() == "collision")
		{
		    loadCollision(node, name + "_collision", sensor->collision);
		    MARK_SET(sensor, collision);
		}		
		else if (node->ValueStr() == "inertial")
		{
		    loadInertial(node, name + "_inertial", sensor->inertial);
		    MARK_SET(sensor, inertial);
		}		
		else if (node->ValueStr() == "visual")
		{
		    loadVisual(node, name + "_visual", sensor->visual);
		    MARK_SET(sensor, visual);
		}		
		else if (node->ValueStr() == "data")
		    loadData(node, &sensor->data);
		else if (node->ValueStr() == "calibration" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		{
		    sensor->calibration =  node->FirstChild()->ValueStr();
		    MARK_SET(sensor, calibration);
		}		
		else
		    ignoreNode(node); 
	    }
	    else
		ignoreNode(node);
	}
	
	m_location = currentLocation;  
    }
    
    void URDF::loadData(const TiXmlNode *node, Data *data)
    {
	std::string name;
	std::string type;
	
	for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
	{
	    if (strcmp(attr->Name(), "name") == 0)
		name = attr->ValueStr();
	    else
		if (strcmp(attr->Name(), "type") == 0)
		    type = attr->ValueStr();
	}
	
	for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
	    if (child->Type() == TiXmlNode::ELEMENT && child->FirstChild() && child->FirstChild()->Type() == TiXmlNode::TEXT)
		data->add(type, name, child->ValueStr(), child->FirstChild()->ValueStr());
	    else if (child->Type() == TiXmlNode::ELEMENT && !child->FirstChild())
		data->add(type, name, child->ValueStr(), "");
	    else if (child->Type() == TiXmlNode::ELEMENT && child->ValueStr() == "xml")
	    {
		std::string key;
		for (const TiXmlAttribute *attr = child->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
		{
		    if (strcmp(attr->Name(), "key") == 0)
			key = attr->ValueStr();
		}
		data->add(type, name, key, child->ToElement());
	    }
	    else
		ignoreNode(child);
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
		fprintf(stderr, "Link '%s' uses a free joint (floating or planar) but its parent is not the environment!\n", i->second->name.c_str());
	    if (m_links.find(i->second->parentName) == m_links.end())
		fprintf(stderr, "Parent of link '%s' is undefined: '%s'\n", i->second->name.c_str(),
			i->second->parentName.c_str());
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
		fprintf(stderr, "Frame '%s' refers to unknown link ('%s')\n", i->first.c_str(), i->second->linkName.c_str());		    
	}
	
	/* for each group, compute the pointers to the links they contain, and for every link,
	 * compute the list of pointers to the groups they are part of 
	 * do the same for frames */
	for (std::map<std::string, Group*>::iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	{
	    std::sort(i->second->linkNames.begin(), i->second->linkNames.end());
	    
	    for (unsigned int j = 0 ; j < i->second->linkNames.size() ; ++j)
		if (m_links.find(i->second->linkNames[j]) == m_links.end())
		{
		    if (m_frames.find(i->second->linkNames[j]) == m_frames.end())
			fprintf(stderr, "Group '%s': '%s' is not defined as a link or frame\n", i->first.c_str(), i->second->linkNames[j].c_str());
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
			fprintf(stderr, "Name '%s' is used both for a link and a frame\n", i->second->linkNames[j].c_str());
		    
		    Link* l = m_links[i->second->linkNames[j]];
		    l->groups.push_back(i->second);
		    i->second->links.push_back(l);
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
    }
    	    
    bool URDF::parse(const TiXmlNode *node)
    {
	if (!node) return false;
	
	int type =  node->Type();
	switch (type)
	{
	case TiXmlNode::DOCUMENT:
	    if (dynamic_cast<const TiXmlDocument*>(node)->RootElement()->ValueStr() != "robot")
		fprintf(stderr, "File '%s' does not start with the <robot> tag\n", m_source.c_str());
	    
	    /* stage 1: extract templates, constants, groups */
	    parse(dynamic_cast<const TiXmlNode*>(dynamic_cast<const TiXmlDocument*>(node)->RootElement()));
	    
	    /* stage 2: parse the rest of the data (that depends on templates & constants) */
	    {		
		for (unsigned int i = 0 ; i < m_stage2.size() ; ++i)
		{
		    const TiXmlElement *elem = m_stage2[i]->ToElement(); 
		    if (!elem)
			fprintf(stderr, "Non-element node found in second stage of parsing. This should NOT happen\n");
		    else
		    {
			std::string name = elem->ValueStr();
			
			if (name == "link")
			    loadLink(m_stage2[i]);
			else if (name == "sensor")
			    loadSensor(m_stage2[i]);
			else if (name == "frame")
			    loadFrame(m_stage2[i]);
			else if (name == "actuator")
			    loadActuator(m_stage2[i]);
			else if (name == "transmission")
			    loadTransmission(m_stage2[i]);
			else if (name == "joint")
			    loadJoint(m_stage2[i], "", NULL);
			else if (name == "geometry")
			    loadGeometry(m_stage2[i], "", NULL);
			else if (name == "collision")
			    loadCollision(m_stage2[i], "", NULL);
			else if (name == "visual")
			    loadVisual(m_stage2[i], "", NULL);
			else if (name == "inertial")
			    loadInertial(m_stage2[i], "", NULL);
			else
			    ignoreNode(m_stage2[i]);
		    }
		}
	    }
	    
	    /* stage 3: 'link' datastructures -- provide easy access pointers */
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
		    std::string nameStr = name;
		    if (!m_name.empty() && nameStr != m_name)
			fprintf(stderr, "Loading a file with contradicting robot name: '%s' - '%s'\n", m_name.c_str(), name);
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
			    m_docs.push_back(doc);
			    if (doc->LoadFile())
			    {
				addPath(filename);
				if (doc->RootElement()->ValueStr() != "robot")
				    fprintf(stderr, "Included file '%s' does not start with the <robot> tag\n", filename);
				parse(dynamic_cast<const TiXmlNode*>(doc->RootElement()));
			    }
			    else
				fprintf(stderr, "Unable to load %s\n", filename);
			    free(filename);
			}
			else
			    fprintf(stderr, "Unable to find %s\n", node->FirstChild()->Value());                    
		    }
		    else 
			ignoreNode(node);
		}
		else if (node->ValueStr() == "constants")
		{        
		    for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
			if (child->Type() == TiXmlNode::ELEMENT && child->FirstChild() && child->FirstChild()->Type() == TiXmlNode::TEXT)
			    m_constants[child->ValueStr()] = child->FirstChild()->ValueStr();
			else
			    ignoreNode(child);
		}
		else
		    if (node->ValueStr() == "templates")
		    {
			for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
			    if (child->Type() == TiXmlNode::ELEMENT && child->ValueStr() == "define")
			    {
				const char *name = child->ToElement()->Attribute("template");
				if (name)
				    m_templates[name] = child;
				else
				    ignoreNode(child);
			    }
			    else
				ignoreNode(child);
		    }
		    else if (node->ValueStr() == "group" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		    {
			std::string group;
			std::string flags;
			
			for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
			{
			    if (strcmp(attr->Name(), "name") == 0)
				group = attr->ValueStr();
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
			
			std::stringstream ss(node->FirstChild()->ValueStr());
			while (ss.good())
			{
			    std::string value; ss >> value;
			    g->linkNames.push_back(value);
			}
			
			if (g->linkNames.empty())
			{
			    fprintf(stderr, "Group '%s' is empty. Not adding to list of groups.\n", g->name.c_str());
			    m_groups.erase(m_groups.find(g->name));
			    delete g;
			}			
		    }
		    else if (node->ValueStr() == "data")
			loadData(node, &m_data);
		    else
			m_stage2.push_back(node);
	    break;
	default:
	    ignoreNode(node);
	}
	
	return true;
    }
    
    void URDF::sanityCheck(void) const
    {
	std::vector<Link*> links;
	getLinks(links);
	for (unsigned int i = 0 ; i < links.size() ; ++i)
	{
	    
	    Link::Joint *joint = links[i]->joint;
	    if (joint->type == Link::Joint::UNKNOWN)
		fprintf(stderr, "Joint '%s' in link '%s' is of unknown type\n", joint->name.c_str(), links[i]->name.c_str());
	    if (joint->type == Link::Joint::REVOLUTE || joint->type == Link::Joint::PRISMATIC)
	    {
		if (joint->axis[0] == 0.0 && joint->axis[1] == 0.0 && joint->axis[2] == 0.0)
		    fprintf(stderr, "Joint '%s' in link '%s' does not seem to have its axis properly set\n", joint->name.c_str(), links[i]->name.c_str());
		if ((joint->isSet["limit"] || joint->type == Link::Joint::PRISMATIC) && joint->limit[0] == 0.0 && joint->limit[1] == 0.0)
		    fprintf(stderr, "Joint '%s' in link '%s' does not seem to have its limits properly set\n", joint->name.c_str(), links[i]->name.c_str());
	    }
	    
	    Link::Geometry *cgeom = links[i]->collision->geometry;
	    if (cgeom->type == Link::Geometry::UNKNOWN)
		fprintf(stderr, "Collision geometry '%s' in link '%s' is of unknown type\n", cgeom->name.c_str(), links[i]->name.c_str());
	    if (cgeom->type != Link::Geometry::UNKNOWN && cgeom->type != Link::Geometry::MESH)
	    {
		int nzero = 0;
		for (int k = 0 ; k < cgeom->nsize ; ++k)
		    nzero += cgeom->size[k] == 0.0 ? 1 : 0;
		if (nzero > 0)
		    fprintf(stderr, "Collision geometry '%s' in link '%s' does not seem to have its size properly set\n", cgeom->name.c_str(), links[i]->name.c_str());
	    }
	    
	    Link::Geometry *vgeom = links[i]->visual->geometry;
	    if (vgeom->type == Link::Geometry::UNKNOWN)
		fprintf(stderr, "Visual geometry '%s' in link '%s' is of unknown type\n", vgeom->name.c_str(), links[i]->name.c_str());
	    if (vgeom->type != Link::Geometry::UNKNOWN && vgeom->type != Link::Geometry::MESH)
	    {
		int nzero = 0;
		for (int k = 0 ; k < vgeom->nsize ; ++k)
		    nzero += vgeom->size[k] == 0.0 ? 1 : 0;
		if (nzero > 0)
		    fprintf(stderr, "Visual geometry '%s' in link '%s' does not seem to have its size properly set\n", vgeom->name.c_str(), links[i]->name.c_str());
	    }	
	}
    }
    
    
} // namespace robot_desc
