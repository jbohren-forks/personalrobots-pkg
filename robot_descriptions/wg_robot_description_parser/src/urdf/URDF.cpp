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

void robot_desc::URDF::freeMemory(void)
{
    clearDocs();
    
    for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	delete i->second;
    for (std::map<std::string, Group*>::iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	delete i->second;
}

void robot_desc::URDF::clear(void)
{
    freeMemory();
    m_name   = "";
    m_source = "";
    m_links.clear();
    m_groups.clear();
    m_paths.clear();
    m_paths.push_back("");
    m_linkRoots.clear();
    m_collision.clear();
    m_joints.clear();
    m_inertial.clear();
    m_visual.clear();
    m_geoms.clear();
    m_actuators.clear();
}

const std::string& robot_desc::URDF::getRobotName(void) const
{
    return m_name;
}

void robot_desc::URDF::getLinks(std::vector<Link*> &links) const
{
    for (std::map<std::string, Link*>::const_iterator i = m_links.begin() ; i != m_links.end() ; i++)
	links.push_back(i->second);
}

unsigned int robot_desc::URDF::getDisjointPartCount(void) const
{
    return m_linkRoots.size();
}

robot_desc::URDF::Link* robot_desc::URDF::getDisjointPart(unsigned int index) const
{
    if (index < m_linkRoots.size())
	return m_linkRoots[index];
    else
	return NULL;
}

void robot_desc::URDF::getGroupNames(std::vector<std::string> &groups) const
{
    for (std::map<std::string, Group*>::const_iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
	groups.push_back(i->first);
}

robot_desc::URDF::Group* robot_desc::URDF::getGroup(const std::string &name) const
{
    std::map<std::string, Group*>::const_iterator it = m_groups.find(name);
    return (it == m_groups.end()) ? NULL : it->second;
}

const robot_desc::URDF::Data& robot_desc::URDF::getData(void) const
{
    return m_data;
}

void robot_desc::URDF::Data::add(const std::string &type, const std::string &name, const std::string &key, const std::string &value)
{
    m_data[type][name][key].str = value;
}

void robot_desc::URDF::Data::add(const std::string &type, const std::string &name, const std::string &key, const TiXmlElement *value)
{
    m_data[type][name][key].xml = value;
}

bool robot_desc::URDF::Data::hasDefault(const std::string &key) const
{
    std::map<std::string, std::string> m = getDataTagValues("", "");
    return m.find(key) != m.end();
}

std::string robot_desc::URDF::Data::getDefaultValue(const std::string &key) const
{
    return getDataTagValues("", "")[key];
}

const TiXmlElement* robot_desc::URDF::Data::getDefaultXML(const std::string &key) const
{
    return getDataTagXML("", "")[key];
}

void robot_desc::URDF::Data::getDataTagTypes(std::vector<std::string> &types) const
{
    for (std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator i = m_data.begin() ; i != m_data.end() ; ++i)
	types.push_back(i->first);
}

void robot_desc::URDF::Data::getDataTagNames(const std::string &type, std::vector<std::string> &names) const
{
    std::map<std::string, std::map<std::string, std::map<std::string, Element > > >::const_iterator pos = m_data.find(type);
    if (pos != m_data.end())
    {
	for (std::map<std::string, std::map<std::string, Element > >::const_iterator i = pos->second.begin() ; i != pos->second.end() ; ++i)
	    names.push_back(i->first);
    }
}
std::map<std::string, std::string> robot_desc::URDF::Data::getDataTagValues(const std::string &type, const std::string &name) const
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

std::map<std::string, const TiXmlElement*> robot_desc::URDF::Data::getDataTagXML(const std::string &type, const std::string &name) const
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

bool robot_desc::URDF::containsCycle(unsigned int index) const
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
	for (unsigned int i = 0 ; i < link->children.size() ; ++i)
	    if (seen.find(link->children[i]) != seen.end())
		return true;
	    else
		queue.push(link->children[i]);
    }

    return false;
}

void robot_desc::URDF::print(FILE *out) const
{
    fprintf(out, "\nList of root links in robot '%s' (%u):\n", m_name.c_str(), m_linkRoots.size());
    for (unsigned int i = 0 ; i < m_linkRoots.size() ; ++i)
	m_linkRoots[i]->print(out, "  ");
    fprintf(out, "\n");
    fprintf(out, "Data types:\n");
    m_data.print(out, "  ");
}

void robot_desc::URDF::Data::print(FILE *out, std::string indent) const
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

void robot_desc::URDF::Link::Geometry::print(FILE *out, std::string indent) const
{
    fprintf(out, "%sGeometry [%s]:\n", indent.c_str(), name.c_str());
    fprintf(out, "%s  - type: %d\n", indent.c_str(), (int)type);
    fprintf(out, "%s  - size: (%f, %f, %f)\n", indent.c_str(), size[0], size[1], size[2]);
    fprintf(out, "%s  - filename: %s\n", indent.c_str(), filename.c_str());
    data.print(out, indent + "  ");
}

void robot_desc::URDF::Link::Actuator::print(FILE *out, std::string indent) const
{
    fprintf(out, "%sActuator [%s]:\n", indent.c_str(), name.c_str());
    data.print(out, indent + "  ");
}

void robot_desc::URDF::Link::Joint::print(FILE *out, std::string indent) const
{
    fprintf(out, "%sJoint [%s]:\n", indent.c_str(), name.c_str());
    fprintf(out, "%s  - type: %d\n", indent.c_str(), (int)type);
    fprintf(out, "%s  - axis: (%f, %f, %f)\n", indent.c_str(), axis[0], axis[1], axis[2]);
    fprintf(out, "%s  - anchor: (%f, %f, %f)\n", indent.c_str(), anchor[0], anchor[1], anchor[2]);
    fprintf(out, "%s  - limit: (%f, %f)\n", indent.c_str(), limit[0], limit[1]);
    fprintf(out, "%s  - calibration: (%f, %f)\n", indent.c_str(), calibration[0], calibration[1]);
    for (unsigned int i = 0 ; i < actuators.size() ; ++i)
	actuators[i]->print(out, indent + "  ");
    data.print(out, indent + "  ");
}

void robot_desc::URDF::Link::Collision::print(FILE *out, std::string indent) const
{
    fprintf(out, "%sCollision [%s]:\n", indent.c_str(), name.c_str());
    fprintf(out, "%s  - verbose: %s\n", indent.c_str(), verbose ? "Yes" : "No");
    fprintf(out, "%s  - material: %s\n", indent.c_str(), material.c_str());
    fprintf(out, "%s  - rpy: (%f, %f, %f)\n", indent.c_str(), rpy[0], rpy[1], rpy[2]);
    fprintf(out, "%s  - xyz: (%f, %f, %f)\n", indent.c_str(), xyz[0], xyz[1], xyz[2]);
    geometry->print(out, indent + "  ");
    data.print(out, indent + "  ");
}

void robot_desc::URDF::Link::Inertial::print(FILE *out, std::string indent) const
{
    fprintf(out, "%sInertial [%s]:\n", indent.c_str(), name.c_str());
    fprintf(out, "%s  - mass: %f\n", indent.c_str(), mass);
    fprintf(out, "%s  - com: (%f, %f, %f)\n", indent.c_str(), com[0], com[1], com[2]);
    fprintf(out, "%s  - inertia: (%f, %f, %f, %f, %f, %f)\n", indent.c_str(), 
	    inertia[0], inertia[1], inertia[2], inertia[3], inertia[4],  inertia[5]);
    data.print(out, indent + "  ");
}

void robot_desc::URDF::Link::Visual::print(FILE *out, std::string indent) const
{
    fprintf(out, "%sVisual [%s]:\n", indent.c_str(), name.c_str());
    fprintf(out, "%s  - material: %s\n", indent.c_str(), material.c_str());
    fprintf(out, "%s  - scale: (%f, %f, %f)\n", indent.c_str(), scale[0], scale[1], scale[2]);
    fprintf(out, "%s  - rpy: (%f, %f, %f)\n", indent.c_str(), rpy[0], rpy[1], rpy[2]);
    fprintf(out, "%s  - xyz: (%f, %f, %f)\n", indent.c_str(), xyz[0], xyz[1], xyz[2]);
    geometry->print(out, indent + "  ");
    data.print(out, indent + "  ");
}

void robot_desc::URDF::Link::print(FILE *out, std::string indent) const
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
	fprintf(out, "%s (%s) ", groups[i]->name.c_str(), groups[i]->flags.c_str());
    fprintf(out, "\n");
    fprintf(out, "%s  - children links: ", indent.c_str());
    for (unsigned int i = 0 ; i < children.size() ; ++i)
	fprintf(out, "%s ", children[i]->name.c_str());
    fprintf(out, "\n");
    for (unsigned int i = 0 ; i < children.size() ; ++i)
	children[i]->print(out, indent + "  ");
    data.print(out, indent + "  ");
}

void robot_desc::URDF::Sensor::print(FILE *out, std::string indent) const
{
    fprintf(out, "%sSensor:\n", indent.c_str());
    fprintf(out, "%s  - type: %d\n", indent.c_str(), (int)type);
    fprintf(out, "%s  - calibration: %s\n", indent.c_str(), calibration.c_str());
    Link::print(out, indent + "  ");
}

bool robot_desc::URDF::Link::canSense(void) const
{
    return false;
}

bool robot_desc::URDF::Sensor::canSense(void) const
{
    return true;
}

void robot_desc::URDF::ignoreNode(const TiXmlNode* node)
{
    switch (node->Type())
    {
    case TiXmlNode::ELEMENT:
	fprintf(stderr, "Ignoring element node '%s'\n", node->Value());
	break;
    case TiXmlNode::TEXT:
	fprintf(stderr, "Ignoring text node with content '%s'\n", node->Value());
	break;
    case TiXmlNode::COMMENT:
    case TiXmlNode::DECLARATION:
	break;	    
    case TiXmlNode::UNKNOWN:
    default:
	fprintf(stderr, "Ignoring unknown node '%s'\n", node->Value());
	break;
    }
}

void robot_desc::URDF::getChildrenAndAttributes(const TiXmlNode *node, std::vector<const TiXmlNode*> &children, std::vector<const TiXmlAttribute*> &attributes) const
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
		    fprintf(stderr, "Template '%s' is not defined\n", attr->Value());
		else
		    getChildrenAndAttributes(pos->second, children, attributes);
	    }
	    else
		attributes.push_back(attr);
	}
    }
}

void robot_desc::URDF::defaultConstants(void)
{
    m_constants["M_PI"] = "3.14159265358979323846";
}

void robot_desc::URDF::clearDocs(void)
{
    /* clear memory allocated for loaded documents */
    for (unsigned int i = 0 ; i < m_docs.size() ; ++i)
	delete m_docs[i];
    m_docs.clear();
}

bool robot_desc::URDF::loadStream(std::istream &is)
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

bool robot_desc::URDF::loadString(const char *data)
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

bool robot_desc::URDF::loadFile(FILE *file)
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

bool robot_desc::URDF::loadFile(const char *filename)
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

void robot_desc::URDF::addPath(const char *filename)
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

char* robot_desc::URDF::findFile(const char *filename)
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

std::string robot_desc::URDF::extractName(std::vector<const TiXmlAttribute*> &attributes, const std::string &defaultName)
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

unsigned int robot_desc::URDF::loadDoubleValues(const TiXmlNode *node, unsigned int count, double *vals)
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

    if (read != count)
	fprintf(stderr, "Not all values were read: '%s'\n", node->Value());
    
    return read;
}

unsigned int robot_desc::URDF::loadBoolValues(const TiXmlNode *node, unsigned int count, bool *vals)
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

    if (read != count)
	fprintf(stderr, "Not all values were read: '%s'\n", node->Value());
    
    return read;
}

void robot_desc::URDF::loadActuator(const TiXmlNode *node, const std::string &defaultName, Link::Actuator *actuator)
{
    std::vector<const TiXmlNode*> children;
    std::vector<const TiXmlAttribute*> attributes;
    getChildrenAndAttributes(node, children, attributes);
    
    std::string name = extractName(attributes, defaultName);
    
    if (!actuator)
    {
	if (m_actuators.find(name) == m_actuators.end())
	{
	    fprintf(stderr, "Attempting to add information to an undefined actuator: '%s'\n", name.c_str());
	    return;
	}
	else
	    actuator = m_actuators[name];
    }
    
    actuator->name = name;
    m_actuators[name] = actuator;
    
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

void robot_desc::URDF::loadJoint(const TiXmlNode *node, const std::string& defaultName, Link::Joint *joint)
{
    std::vector<const TiXmlNode*> children;
    std::vector<const TiXmlAttribute*> attributes;
    getChildrenAndAttributes(node, children, attributes);
    
    std::string name = extractName(attributes, defaultName);
    
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
	    else
	    if (attr->ValueStr() == "revolute")
		joint->type = Link::Joint::REVOLUTE;
	    else
  	    if (attr->ValueStr() == "prismatic")
		joint->type = Link::Joint::PRISMATIC;
	    else
	    if (attr->ValueStr() == "floating")
		joint->type = Link::Joint::FLOATING;
	    else
		fprintf(stderr, "Unknown sensor type: '%s'\n", attr->Value());
	}
    }
    
    for (unsigned int i = 0 ; i < children.size() ; ++i)
    {
	const TiXmlNode *node = children[i];
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "axis")
		loadDoubleValues(node, 3, joint->axis);
	    else
	    if (node->ValueStr() == "anchor")
		loadDoubleValues(node, 3, joint->anchor);
	    else
	    if (node->ValueStr() == "limit")
		loadDoubleValues(node, 2, joint->limit);
	    else
	    if (node->ValueStr() == "calibration")
		loadDoubleValues(node, 2, joint->calibration);
	    else
	    if (node->ValueStr() == "data")
		loadData(node, &joint->data);
	    else
	    if (node->ValueStr() == "actuator")
	    {
		Link::Actuator *actuator = new Link::Actuator();
		joint->actuators.push_back(actuator);
		loadActuator(node, name + "_actuator", actuator);
	    }
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }
    
}

void robot_desc::URDF::loadGeometry(const TiXmlNode *node, const std::string &defaultName, Link::Geometry *geometry)
{
    std::vector<const TiXmlNode*> children;
    std::vector<const TiXmlAttribute*> attributes;
    getChildrenAndAttributes(node, children, attributes);

    std::string name = extractName(attributes, defaultName);
    
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
		geometry->type = Link::Geometry::BOX;
	    else
  	    if (attr->ValueStr() == "cylinder")
		geometry->type = Link::Geometry::CYLINDER;
	    else
	    if (attr->ValueStr() == "sphere")
		geometry->type = Link::Geometry::SPHERE;
	    else
	    if (attr->ValueStr() == "mesh")
		geometry->type = Link::Geometry::MESH;
	    else
		fprintf(stderr, "Unknown sensor type: '%s'\n", attr->Value());
	}
    }

    for (unsigned int i = 0 ; i < children.size() ; ++i)
    {
	const TiXmlNode *node = children[i];
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "size")
	    {
		switch (geometry->type)
		{
		case Link::Geometry::BOX:
		    loadDoubleValues(node, 3, geometry->size);
		    break;
		case Link::Geometry::CYLINDER:
		    loadDoubleValues(node, 2, geometry->size);
		    break;
		case Link::Geometry::SPHERE:
		    loadDoubleValues(node, 1, geometry->size);
		    break;
		default:
		    ignoreNode(node);
		    break;
		}
	    }
	    else
	    if (node->ValueStr() == "data")
		loadData(node, &geometry->data);
	    else
	    if (node->ValueStr() == "filename" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		geometry->filename = node->FirstChild()->ValueStr();
	    else		
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }
}

void robot_desc::URDF::loadCollision(const TiXmlNode *node, const std::string &defaultName, Link::Collision *collision)
{  
    std::vector<const TiXmlNode*> children;
    std::vector<const TiXmlAttribute*> attributes;
    getChildrenAndAttributes(node, children, attributes);

    std::string name = extractName(attributes, defaultName);
    
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
		loadDoubleValues(node, 3, collision->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadDoubleValues(node, 3, collision->xyz);
	    else
	    if (node->ValueStr() == "verbose")
		loadBoolValues(node, 1, &collision->verbose);
	    else
	    if (node->ValueStr() == "material" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		collision->material = node->FirstChild()->ValueStr();
	    else
	    if (node->ValueStr() == "data")
		loadData(node, &collision->data);
	    else
	    if (node->ValueStr() == "geometry")
		loadGeometry(node, name + "_geom", collision->geometry);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }    
}

void robot_desc::URDF::loadVisual(const TiXmlNode *node, const std::string &defaultName, Link::Visual *visual)
{  
    std::vector<const TiXmlNode*> children;
    std::vector<const TiXmlAttribute*> attributes;
    getChildrenAndAttributes(node, children, attributes);

    std::string name = extractName(attributes, defaultName);
    
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
		loadDoubleValues(node, 3, visual->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadDoubleValues(node, 3, visual->xyz);
	    else
	    if (node->ValueStr() == "scale")
		loadDoubleValues(node, 3, visual->scale);
	    else
	    if (node->ValueStr() == "material" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		visual->material = node->FirstChild()->ValueStr();
	    else
	    if (node->ValueStr() == "data")
		loadData(node, &visual->data);
	    else
	    if (node->ValueStr() == "geometry")
		loadGeometry(node, name + "_geom", visual->geometry);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }   
}

void robot_desc::URDF::loadInertial(const TiXmlNode *node, const std::string &defaultName, Link::Inertial *inertial)
{ 
    std::vector<const TiXmlNode*> children;
    std::vector<const TiXmlAttribute*> attributes;
    getChildrenAndAttributes(node, children, attributes);
    
    std::string name = extractName(attributes, defaultName);
    
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
		loadDoubleValues(node, 1, &inertial->mass);
	    else
	    if (node->ValueStr() == "com")
		loadDoubleValues(node, 3, inertial->com);
	    else
	    if (node->ValueStr() == "inertia")
		loadDoubleValues(node, 6, inertial->inertia);
	    else
	    if (node->ValueStr() == "data")
		loadData(node, &inertial->data);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }
}

void robot_desc::URDF::loadLink(const TiXmlNode *node)
{
    std::vector<const TiXmlNode*> children;
    std::vector<const TiXmlAttribute*> attributes;
    getChildrenAndAttributes(node, children, attributes);

    std::string name = extractName(attributes, "");    
    Link *link = (m_links.find(name) != m_links.end()) ? m_links[name] : new Link();
    link->name = name;
    if (link->name == "")
	fprintf(stderr, "No link name given\n");
    m_links[link->name] = link;
    
    for (unsigned int i = 0 ; i < children.size() ; ++i)
    {
	const TiXmlNode *node = children[i];
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "parent" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		link->parentName = node->FirstChild()->ValueStr();
	    else
	    if (node->ValueStr() == "rpy")
		loadDoubleValues(node, 3, link->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadDoubleValues(node, 3, link->xyz);
	    else
	    if (node->ValueStr() == "joint")
		loadJoint(node, name + "_joint", link->joint);
	    else
	    if (node->ValueStr() == "collision")
		loadCollision(node, name + "_collision", link->collision);
	    else
	    if (node->ValueStr() == "inertial")
		loadInertial(node, name + "_inertial", link->inertial);
	    else
	    if (node->ValueStr() == "visual")
		loadVisual(node, name + "_visual", link->visual);
	    else
	    if (node->ValueStr() == "data")
		loadData(node, &link->data);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }
}

void robot_desc::URDF::loadSensor(const TiXmlNode *node)
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
    if (sensor->name == "")
	fprintf(stderr, "No sensor name given\n");
    
    if (m_links.find(sensor->name) != m_links.end())
	fprintf(stderr, "Sensor '%s' redefined\n", sensor->name.c_str());
    m_links[sensor->name] = dynamic_cast<Link*>(sensor);
    
    for (unsigned int i = 0 ; i < attributes.size() ; ++i)
    {
	const TiXmlAttribute *attr = attributes[i];
	if (strcmp(attr->Name(), "type") == 0)
	{
	    if (attr->ValueStr() == "camera")
		sensor->type = Sensor::CAMERA;
	    else
	    if (attr->ValueStr() == "laser")
		sensor->type = Sensor::LASER;
	    else
	    if (attr->ValueStr() == "stereocamera")
		sensor->type = Sensor::STEREO_CAMERA;
	    else
		fprintf(stderr, "Unknown sensor type: '%s'\n", attr->Value());
	}
    }

    for (unsigned int i = 0 ; i < children.size() ; ++i)
    {
	const TiXmlNode *node = children[i];
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "parent" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		sensor->parentName = node->FirstChild()->ValueStr();
	    else
	    if (node->ValueStr() == "rpy")
		loadDoubleValues(node, 3, sensor->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadDoubleValues(node, 3, sensor->xyz);
	    else
	    if (node->ValueStr() == "joint")
		loadJoint(node, name + "_joint", sensor->joint);
	    else
	    if (node->ValueStr() == "collision")
		loadCollision(node, name + "_collision", sensor->collision);
	    else
	    if (node->ValueStr() == "inertial")
		loadInertial(node, name + "_inertial", sensor->inertial);
	    else
	    if (node->ValueStr() == "visual")
		loadVisual(node, name + "_visual", sensor->visual);
	    else
	    if (node->ValueStr() == "data")
		loadData(node, &sensor->data);
	    else
	    if (node->ValueStr() == "calibration" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		sensor->calibration =  node->FirstChild()->ValueStr();
	    else
		ignoreNode(node); 
	}
	else
	    ignoreNode(node);
    }
}

void robot_desc::URDF::loadData(const TiXmlNode *node, Data *data)
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
	else
	    if (child->Type() == TiXmlNode::ELEMENT && !child->FirstChild())
		data->add(type, name, child->ValueStr(), "");
	    else
		if (child->Type() == TiXmlNode::ELEMENT && child->ValueStr() == "xml")
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

bool robot_desc::URDF::parse(const TiXmlNode *node)
{
    if (!node) return false;
    
    int type =  node->Type();
    switch (type)
    {
    case TiXmlNode::DOCUMENT:
	parse(dynamic_cast<const TiXmlNode*>(dynamic_cast<const TiXmlDocument*>(node)->RootElement()));
	
	{
	    for (unsigned int i = 0 ; i < m_stage2.size() ; ++i)
	    {
		const TiXmlElement *elem = m_stage2[i]->ToElement(); 
		if (!elem)
		    fprintf(stderr, "Non-element node found in second stage of parsing\n");
		else
		{
		    std::string name = elem->ValueStr();
		    
		    if (name == "link")
			loadLink(m_stage2[i]);
		    else
		    if (name == "sensor")
			loadSensor(m_stage2[i]);
		    else
		    if (name == "joint")
			loadJoint(m_stage2[i], "", NULL);
		    else
		    if (name == "geometry")
			loadGeometry(m_stage2[i], "", NULL);
		    else
		    if (name == "collision")
			loadCollision(m_stage2[i], "", NULL);
		    else
		    if (name == "visual")
			loadVisual(m_stage2[i], "", NULL);
		    else
		    if (name == "inertial")
			loadInertial(m_stage2[i], "", NULL);
		    else
		    if (name == "actuator")
			loadInertial(m_stage2[i], "", NULL);
		    else
			ignoreNode(m_stage2[i]);
		}
	    }

	    /* compute the proper pointers for parent nodes and children */
	    for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	    {
		if (i->second->parentName.empty() || i->second->parentName == "world")
		{
		    m_linkRoots.push_back(i->second);
		    continue;
		}
		if (i->second->joint->type == Link::Joint::FLOATING)
		    fprintf(stderr, "Link '%s' uses a floating joint but its parent is not the environment!\n", i->second->name.c_str());
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
	    
	    /* for each group, compute the pointers to the links they contain, and for every link,
	     * compute the list of pointers to the groups they are part of */
	    for (std::map<std::string, Group*>::iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
		for (unsigned int j = 0 ; j < i->second->linkNames.size() ; ++j)
		    if (m_links.find(i->second->linkNames[j]) == m_links.end())
			fprintf(stderr, "Group '%s': link '%s' is undefined\n", i->first.c_str(), i->second->linkNames[j].c_str());
		    else
		    {
			Link* l = m_links[i->second->linkNames[j]];
			l->groups.push_back(i->second);
			i->second->links.push_back(l);
		    }
	    
	    /* for every group, find the set of links that are roots in this group (their parent is not in the group) */
	    for (std::map<std::string, Group*>::iterator i = m_groups.begin() ; i != m_groups.end() ; i++)
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
	}
	
	m_constants.clear();
	m_templates.clear();
	m_stage2.clear();
	
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
	    else
		if (node->ValueStr() == "constants")
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
		    else
			if (node->ValueStr() == "group" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
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
			    if (g->flags.empty())
				g->flags = flags;
			    else
				g->flags += " " + flags;
			    std::stringstream ss(node->FirstChild()->ValueStr());
			    while (ss.good())
			    {
				std::string value; ss >> value;
				g->linkNames.push_back(value);
			    }
			}
			else
			    if (node->ValueStr() == "data")
				loadData(node, &m_data);
			    else
				m_stage2.push_back(node);
	break;
    default:
	ignoreNode(node);
    }
    
    return true;
}
