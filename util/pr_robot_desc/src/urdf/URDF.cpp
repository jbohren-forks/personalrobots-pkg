#include "urdf/URDF.h"
#include "urdf/MathExpression.h"
#include <tinyxml-2.5.3/tinyxml.h>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>

// need to change this depending on OS (different on windows)
static const char PATH_SEPARATOR = '/';

void URDF::freeMemory(void)
{
    for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	delete i->second;
}

void URDF::clear(void)
{
    freeMemory();
    m_name   = "";
    m_source = "";
    m_links.clear();
    m_paths.clear();
    m_paths.push_back("");
}

void URDF::ignoreNode(const void *data)
{	
    const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(data);
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

void URDF::getChildrenAndAttributes(const void *data, std::vector<const void *> &children, std::vector<const void *> &attributes) const
{
    const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(data);
    for (const TiXmlNode *child = node->FirstChild() ; child ; child = child->NextSibling())
	children.push_back(child);
    if (node->Type() == TiXmlNode::ELEMENT)
    {
	for (const TiXmlAttribute *attr = node->ToElement()->FirstAttribute() ; attr ; attr = attr->Next())
	{
	    if (strcmp(attr->Name(), "clone") == 0)
	    {
		std::map<std::string, const void *>::const_iterator pos = m_templates.find(attr->ValueStr());
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

void URDF::defaultConstants(void)
{
    m_constants["M_PI"] = M_PI;
}

bool URDF::load(const char *filename)
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
    
    /* clear memory allocated for loaded documents */
    for (unsigned int i = 0 ; i < m_docs.size() ; ++i)
	delete reinterpret_cast<TiXmlDocument*>(m_docs[i]);
    m_docs.clear();
    
    return result;
}

void URDF::addPath(const char *filename)
{
    if (!filename)
	return;
    
    std::string name = filename;
    std::string::size_type pos = name.find_last_of(PATH_SEPARATOR);
    name.erase(pos);
    m_paths.push_back(name + PATH_SEPARATOR);
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

std::string URDF::extractName(std::vector<const void *> &attributes)
{ 
    std::string name;    
    for (unsigned int i = 0 ; i < attributes.size() ; ++i)
    {
	const TiXmlAttribute *attr = reinterpret_cast<const TiXmlAttribute*>(attributes[i]);
	if (strcmp(attr->Name(), "name") == 0)
	{
	    name = attr->ValueStr();
	    attributes.erase(attributes.begin() + i);
	    break;
	}
    }
    return name;
}

static double getConstant(void *data, std::string &name)
{
    std::map<std::string, std::string> *m = reinterpret_cast<std::map<std::string, std::string>*>(data);
    if (m->find(name) == m->end())
    {
	fprintf(stderr, "Request for undefined constant: '%s'\n", name.c_str());
	return 0.0;
    }
    else
        return atof((*m)[name].c_str());
}

unsigned int URDF::loadValues(const void *data, unsigned int count, double *vals)
{
    const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(data);
    if (node && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
	node = node->FirstChild();
    else
	return 0;
    
    std::stringstream ss(node->ValueStr());
    unsigned int read = 0;
    
    for (int i = 0 ; ss.good() && i < count ; ++i)
    {
	std::string value;
	ss >> value;
	vals[i] = EvaluateMathExpression(value, &getConstant, reinterpret_cast<void*>(&m_constants));
	read++;
    }

    if (read != count)
	fprintf(stderr, "Not all values were read: '%s'\n", node->Value());
    
    return read;
}

void URDF::loadActuator(const void *data, Link::Actuator *actuator)
{
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);
    
    std::string name = extractName(attributes);
    
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
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "motor" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		actuator->motor = node->FirstChild()->ValueStr(); 
	    else
	    if (node->ValueStr() == "ip" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		actuator->ip = node->FirstChild()->ValueStr(); 
	    else
	    if (node->ValueStr() == "port" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		actuator->port = atoi(node->FirstChild()->Value());
	    else
	    if (node->ValueStr() == "reduction")
		loadValues(node, 1, &actuator->reduction);
	    else
	    if (node->ValueStr() == "polymap")
		loadValues(node, 3, actuator->polymap);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }    
}

void URDF::loadJoint(const void *data, Link::Joint *joint)
{
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);
    
    std::string name = extractName(attributes);
    
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
	const TiXmlAttribute *attr = reinterpret_cast<const TiXmlAttribute*>(attributes[i]);
	if (strcmp(attr->Name(), "type") == 0)
	{
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
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "axis")
		loadValues(node, 3, joint->axis);
	    else
	    if (node->ValueStr() == "anchor")
		loadValues(node, 3, joint->anchor);
	    else
	    if (node->ValueStr() == "limit")
		loadValues(node, 2, joint->limit);
	    else
	    if (node->ValueStr() == "calibration")
		loadValues(node, 2, joint->calibration);
	    else
	    if (node->ValueStr() == "actuator")
	    {
		Link::Actuator *actuator = new Link::Actuator();
		joint->actuators.push_back(actuator);
		loadActuator(node, actuator);
	    }
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }
    
}

void URDF::loadGeometry(const void *data, Link::Geometry *geometry)
{
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);
    
    std::string name = extractName(attributes);
    
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
	const TiXmlAttribute *attr = reinterpret_cast<const TiXmlAttribute*>(attributes[i]);
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
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "size")
	    {
		switch (geometry->type)
		{
		case Link::Geometry::BOX:
		    loadValues(node, 3, geometry->size);
		    break;
		case Link::Geometry::CYLINDER:
		    loadValues(node, 2, geometry->size);
		    break;
		case Link::Geometry::SPHERE:
		    loadValues(node, 1, geometry->size);
		    break;
		default:
		    ignoreNode(node);
		    break;
		}
	    }
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

void URDF::loadCollision(const void *data, Link::Collision *collision)
{  
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);
    
    std::string name = extractName(attributes);
    
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
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "rpy")
		loadValues(node, 3, collision->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadValues(node, 3, collision->xyz);
	    else
	    if (node->ValueStr() == "material" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		collision->material = node->FirstChild()->ValueStr();
	    else		
	    if (node->ValueStr() == "geometry")
		loadGeometry(node, collision->geometry);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }    
}

void URDF::loadVisual(const void *data, Link::Visual *visual)
{  
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);
    
    std::string name = extractName(attributes);
    
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
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "rpy")
		loadValues(node, 3, visual->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadValues(node, 3, visual->xyz);
	    else
	    if (node->ValueStr() == "material" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		visual->material = node->FirstChild()->ValueStr();
	    else		
	    if (node->ValueStr() == "geometry")
		loadGeometry(node, visual->geometry);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }   
}

void URDF::loadInertial(const void *data, Link::Inertial *inertial)
{ 
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);
    
    std::string name = extractName(attributes);
    
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
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "mass")
		loadValues(node, 1, &inertial->mass);
	    else
	    if (node->ValueStr() == "com")
		loadValues(node, 3, inertial->com);
	    else
	    if (node->ValueStr() == "inertia")
		loadValues(node, 6, inertial->inertia);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }
}

void URDF::loadLink(const void *data)
{
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);
    
    std::string name = extractName(attributes);    
    Link *link = (m_links.find(name) != m_links.end()) ? m_links[name] : new Link();
    link->name = name;
    if (link->name == "")
	fprintf(stderr, "No link name given\n");
    m_links[link->name] = link;
    
    for (unsigned int i = 0 ; i < children.size() ; ++i)
    {
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "parent" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		link->parentName = node->FirstChild()->ValueStr();
	    else
	    if (node->ValueStr() == "rpy")
		loadValues(node, 3, link->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadValues(node, 3, link->xyz);
	    else
	    if (node->ValueStr() == "joint")
		loadJoint(node, link->joint);
	    else
	    if (node->ValueStr() == "collision")
		loadCollision(node, link->collision);
	    else
	    if (node->ValueStr() == "inertial")
		loadInertial(node, link->inertial);
	    else
	    if (node->ValueStr() == "visual")
		loadVisual(node, link->visual);
	    else
		ignoreNode(node);
	}
	else
	    ignoreNode(node);
    }
}

void URDF::loadSensor(const void *data)
{
    std::vector<const void*> children;
    std::vector<const void*> attributes;
    getChildrenAndAttributes(data, children, attributes);

    std::string name = extractName(attributes);    
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
	const TiXmlAttribute *attr = reinterpret_cast<const TiXmlAttribute*>(attributes[i]);
	if (strcmp(attr->Name(), "type") == 0)
	{
	    if (attr->ValueStr() == "camera")
		sensor->type = Sensor::CAMERA;
	    else
		fprintf(stderr, "Unknown sensor type: '%s'\n", attr->Value());
	}
    }

    for (unsigned int i = 0 ; i < children.size() ; ++i)
    {
	const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(children[i]);
	if (node->Type() == TiXmlNode::ELEMENT)
	{
	    if (node->ValueStr() == "parent" && node->FirstChild() && node->FirstChild()->Type() == TiXmlNode::TEXT)
		sensor->parentName = node->FirstChild()->ValueStr();
	    else
	    if (node->ValueStr() == "rpy")
		loadValues(node, 3, sensor->rpy);
	    else
	    if (node->ValueStr() == "xyz")
		loadValues(node, 3, sensor->xyz);
	    else
	    if (node->ValueStr() == "joint")
		loadJoint(node, sensor->joint);
	    else
	    if (node->ValueStr() == "collision")
		loadCollision(node, sensor->collision);
	    else
	    if (node->ValueStr() == "inertial")
		loadInertial(node, sensor->inertial);
	    else
	    if (node->ValueStr() == "visual")
		loadVisual(node, sensor->visual);
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

bool URDF::parse(const void *data)
{
    const TiXmlNode *node = reinterpret_cast<const TiXmlNode*>(data);
    if (!node) return false;
    
    int type =  node->Type();
    switch (type)
    {
    case TiXmlNode::DOCUMENT:
	parse(dynamic_cast<const TiXmlNode*>(dynamic_cast<const TiXmlDocument*>(node)->RootElement()));
	
	{
	    for (unsigned int i = 0 ; i < m_stage2.size() ; ++i)
	    {
		const TiXmlElement *elem = reinterpret_cast<const TiXmlNode*>(m_stage2[i])->ToElement(); 
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
			loadJoint(m_stage2[i], NULL);
		    else
		    if (name == "geometry")
			loadGeometry(m_stage2[i], NULL);
		    else
		    if (name == "collision")
			loadCollision(m_stage2[i], NULL);
		    else
		    if (name == "visual")
			loadVisual(m_stage2[i], NULL);
		    else
		    if (name == "inertial")
			loadInertial(m_stage2[i], NULL);
		    else
		    if (name == "actuator")
			loadInertial(m_stage2[i], NULL);
		    else
			ignoreNode(m_stage2[i]);
		}
	    }

	    /* compute the proper pointers for parent nodes and children */
	    for (std::map<std::string, Link*>::iterator i = m_links.begin() ; i != m_links.end() ; i++)
	    {
		if (i->second->parentName.empty() || i->second->parentName == "world")
		{
		    m_roots.push_back(i->second);
		    continue;
		}
		if (m_links.find(i->second->parentName) == m_links.end())
		    fprintf(stderr, "Parent of link '%s' is undefined: '%s'\n", i->second->name.c_str(),
			    i->second->parentName.c_str());
		else
		{
		    Link *parent =  m_links[i->second->parentName];
		    i->second->parent = parent;
		    parent->children.push_back(parent);
		}
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
			m_stage2.push_back(node);
	break;
    default:
	ignoreNode(node);
    }
    
    return true;
}
