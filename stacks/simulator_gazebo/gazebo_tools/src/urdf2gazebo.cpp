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

#include <gazebo_tools/urdf2gazebo.h>
#include "ros/node.h"
#include <sstream>

using namespace urdf2gazebo;

URDF2Gazebo::URDF2Gazebo()
{
  robot_model_name_ = "pr2_model";
}
URDF2Gazebo::URDF2Gazebo(std::string robot_model_name)
{
  robot_model_name_ = robot_model_name;
}

URDF2Gazebo::~URDF2Gazebo()
{
}


std::string URDF2Gazebo::vector32str(const urdf::Vector3 vector, double (*conv)(double) = NULL)
{
    std::stringstream ss;
    ss << (conv ? conv(vector.x) : vector.x);
    ss << " ";
    ss << (conv ? conv(vector.y) : vector.y);
    ss << " ";
    ss << (conv ? conv(vector.z) : vector.z);
    return ss.str();
}
std::string URDF2Gazebo::values2str(unsigned int count, const double *values, double (*conv)(double) = NULL)
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++)
    {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}

void URDF2Gazebo::setupTransform(btTransform &transform, urdf::Pose pose)
{
    btMatrix3x3 mat;
    mat.setRotation(btQuaternion(pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w));
    transform = btTransform(mat,btVector3(pose.position.x,pose.position.y,pose.position.z));
}

void URDF2Gazebo::addKeyValue(TiXmlElement *elem, const std::string& key, const std::string &value)
{
    TiXmlElement *ekey      = new TiXmlElement(key);
    TiXmlText    *text_ekey = new TiXmlText(value);
    ekey->LinkEndChild(text_ekey);    
    elem->LinkEndChild(ekey); 
}

void URDF2Gazebo::addTransform(TiXmlElement *elem, const::btTransform& transform)
{
    btVector3 pz = transform.getOrigin();
    double cpos[3] = { pz.x(), pz.y(), pz.z() };
    btMatrix3x3 mat = transform.getBasis();
    double crot[3];
    mat.getEulerZYX(crot[2],crot[1],crot[0]);
    
    /* set geometry transform */
    addKeyValue(elem, "xyz", values2str(3, cpos));
    addKeyValue(elem, "rpy", values2str(3, crot, rad2deg));  
}

std::string URDF2Gazebo::getGazeboValue(TiXmlElement* elem)
{
    std::string value_str;
    if (elem->Attribute("value"))
    {
      value_str = elem->Attribute("value");
    }
    else if (elem->FirstChild() &&  elem->FirstChild()->Type() == TiXmlNode::TEXT)
    {
      value_str = elem->FirstChild()->ValueStr();
    }
    return value_str;
}

void URDF2Gazebo::parseGazeboExtension(TiXmlDocument &urdf_in)
{
  ROS_DEBUG("parsing gazebo extension");
  TiXmlElement* robot_xml = urdf_in.FirstChildElement("robot");

  // Get all Gazebo extension elements
  for (TiXmlElement* gazebo_xml = robot_xml->FirstChildElement("gazebo"); gazebo_xml; gazebo_xml = gazebo_xml->NextSiblingElement("gazebo"))
  {
    GazeboExtension *gazebo;

    const char* ref = gazebo_xml->Attribute("reference");
    std::string ref_str;
    if (!ref)
    {
      ROS_DEBUG("parsing gazebo extension for robot, reference is not specified");
      // copy extensions for robot (outside of link/joint)
      ref_str = std::string(this->robot_model_name_);
    }
    else
    {
      ROS_DEBUG("parsing gazebo extension for %s",ref);
      // copy extensions for link/joint
      ref_str = std::string(ref);
    }

    if (gazebo_extensions_.find(ref_str) == gazebo_extensions_.end())
    {
        gazebo = new GazeboExtension();
        ROS_DEBUG("first time parsing, creating extension map %s",ref);
    }
    else
    {
        gazebo = (gazebo_extensions_.find(ref_str))->second;
        ROS_DEBUG("extension map %s exists, reuse",ref);
    }

    // begin parsing xml node
    for (TiXmlElement *child_elem = gazebo_xml->FirstChildElement(); child_elem; child_elem = child_elem->NextSiblingElement())
    {
      ROS_DEBUG("child element : %s ", child_elem->ValueStr().c_str());

      // material
      if (child_elem->ValueStr() == "material")
      {
          gazebo->material = getGazeboValue(child_elem);
          ROS_DEBUG("   material %s",gazebo->material.c_str());
      }
      else if (child_elem->ValueStr() == "turnGravityOff")
      {
        std::string value_str = getGazeboValue(child_elem);

        // default of turnGravityOff is false
        if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
          gazebo->turnGravityOff = true;
        else
          gazebo->turnGravityOff = false;

        ROS_DEBUG("   turnGravityOff %d",gazebo->turnGravityOff);

      }
      else if (child_elem->ValueStr() == "dampingFactor")
      {
          gazebo->is_dampingFactor = true;
          gazebo->dampingFactor = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   dampingFactor %f",gazebo->dampingFactor);
      }
      else if (child_elem->ValueStr() == "mu1")
      {
          gazebo->is_mu1 = true;
          gazebo->mu1 = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   mu1 %f",gazebo->mu1);
      }
      else if (child_elem->ValueStr() == "mu2")
      {
          gazebo->is_mu2 = true;
          gazebo->mu2 = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   mu2 %f",gazebo->mu2);
      }
      else if (child_elem->ValueStr() == "kp")
      {
          gazebo->is_kp = true;
          gazebo->kp = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   kp %f",gazebo->kp);
      }
      else if (child_elem->ValueStr() == "kd")
      {
          gazebo->is_kd = true;
          gazebo->kd = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   kd %f",gazebo->kd);
      }
      else if (child_elem->ValueStr() == "selfCollide")
      {
        std::string value_str = getGazeboValue(child_elem);

        // default of selfCollide is false
        if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
          gazebo->selfCollide = true;
        else
          gazebo->selfCollide = false;

        ROS_DEBUG("   selfCollide %d",gazebo->selfCollide);

      }
      else if (child_elem->ValueStr() == "laserRetro")
      {
          gazebo->is_laserRetro = true;
          gazebo->laserRetro = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   laserRetro %f",gazebo->laserRetro);
      }
      else if (child_elem->ValueStr() == "stopKp")
      {
          gazebo->is_stopKp = true;
          gazebo->stopKp = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   stopKp %f",gazebo->stopKp);
      }
      else if (child_elem->ValueStr() == "stopKd")
      {
          gazebo->is_stopKd = true;
          gazebo->stopKd = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   stopKd %f",gazebo->stopKd);
      }
      else if (child_elem->ValueStr() == "fudgeFactor")
      {
          gazebo->is_fudgeFactor = true;
          gazebo->fudgeFactor = atof(getGazeboValue(child_elem).c_str());
          ROS_DEBUG("   fudgeFactor %f",gazebo->fudgeFactor);
      }
      else if (child_elem->ValueStr() == "provideFeedback")
      {
          std::string value_str = getGazeboValue(child_elem);

          if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
            gazebo->provideFeedback = true;
          else
            gazebo->provideFeedback = false;

          ROS_DEBUG("   provideFeedback %f",gazebo->provideFeedback);
      }
      else
      {
          std::ostringstream stream;
          stream << *child_elem;

          // FIXME: how to copy?
          //TiXmlElement *block = new TiXmlElement(stream.str());
          TiXmlElement *block = new TiXmlElement(*child_elem);

          gazebo->copy_block.push_back(block);

          ROS_DEBUG("    copy block %s",stream.str().c_str());
      }
    }

    // insert into my map
    this->gazebo_extensions_.insert(std::make_pair( ref_str, gazebo ) );
  }
}

void URDF2Gazebo::insertGazeboExtensionGeom(TiXmlElement *elem,std::string link_name)
{
    for (std::map<std::string,GazeboExtension*>::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first == link_name)
      {
          ROS_DEBUG("geom: reference %s link name %s",gazebo_it->first.c_str(),link_name.c_str());
          // insert mu1, mu2, kp, kd for geom
          if (gazebo_it->second->is_mu1)
              addKeyValue(elem, "mu1", values2str(1, &gazebo_it->second->mu1) );
          if (gazebo_it->second->is_mu2)
              addKeyValue(elem, "mu2", values2str(1, &gazebo_it->second->mu2) );
          if (gazebo_it->second->is_kp)
              addKeyValue(elem, "kp", values2str(1, &gazebo_it->second->kp) );
          if (gazebo_it->second->is_kd)
              addKeyValue(elem, "kd", values2str(1, &gazebo_it->second->kd) );
          if (gazebo_it->second->is_laserRetro)
              addKeyValue(elem, "laserRetro", values2str(1, &gazebo_it->second->laserRetro) );
      }
    }
}
void URDF2Gazebo::insertGazeboExtensionVisual(TiXmlElement *elem,std::string link_name)
{
    for (std::map<std::string,GazeboExtension*>::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first == link_name)
      {
          ROS_DEBUG("visual: reference %s link name %s, material %s",gazebo_it->first.c_str(),link_name.c_str(),gazebo_it->second->material.c_str());
          // insert material block
          if (!gazebo_it->second->material.empty())
              addKeyValue(elem, "material", gazebo_it->second->material);
      }
    }
}
void URDF2Gazebo::insertGazeboExtensionBody(TiXmlElement *elem,std::string link_name)
{
    for (std::map<std::string,GazeboExtension*>::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first == link_name)
      {
          ROS_DEBUG("body: reference %s link name %s",gazebo_it->first.c_str(),link_name.c_str());
          // insert turnGravityOff
          if (gazebo_it->second->turnGravityOff)
              addKeyValue(elem, "turnGravityOff", "true");
          else
              addKeyValue(elem, "turnGravityOff", "false");
          // damping factor
          if (gazebo_it->second->is_dampingFactor)
              addKeyValue(elem, "dampingFactor", values2str(1, &gazebo_it->second->dampingFactor) );
          // selfCollide tag
          if (gazebo_it->second->selfCollide)
              addKeyValue(elem, "selfCollide", "true");
          else
              addKeyValue(elem, "selfCollide", "false");
          // insert copy block into body
          for (std::vector<TiXmlElement*>::iterator block_it = gazebo_it->second->copy_block.begin();
               block_it != gazebo_it->second->copy_block.end(); block_it++)
          {
              elem->LinkEndChild((*block_it)->Clone());
          }
      }
    }
}
void URDF2Gazebo::insertGazeboExtensionJoint(TiXmlElement *elem,std::string joint_name)
{
    for (std::map<std::string,GazeboExtension*>::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first == joint_name)
      {
          ROS_DEBUG("geom: reference %s joint name %s, stopKp %f",gazebo_it->first.c_str(),joint_name.c_str(),gazebo_it->second->stopKp);
          // insert stopKp, stopKd, fudgeFactor
          if (gazebo_it->second->is_stopKp)
              addKeyValue(elem, "stopKp", values2str(1, &gazebo_it->second->stopKp) );
          if (gazebo_it->second->is_stopKd)
              addKeyValue(elem, "stopKd", values2str(1, &gazebo_it->second->stopKd) );
          if (gazebo_it->second->is_fudgeFactor)
              addKeyValue(elem, "fudgeFactor", values2str(1, &gazebo_it->second->fudgeFactor) );

          // insert provideFeedback
          if (gazebo_it->second->provideFeedback)
              addKeyValue(elem, "provideFeedback", "true");
          else
              addKeyValue(elem, "provideFeedback", "false");
      }
    }
}
void URDF2Gazebo::insertGazeboExtensionRobot(TiXmlElement *elem)
{
    for (std::map<std::string,GazeboExtension*>::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first == this->robot_model_name_) // no reference
      {
          // insert copy block into robot
          for (std::vector<TiXmlElement*>::iterator block_it = gazebo_it->second->copy_block.begin();
               block_it != gazebo_it->second->copy_block.end(); block_it++)
          {
              std::ostringstream stream_in;
              stream_in << *(*block_it);
              ROS_DEBUG("robot: reference empty, copy block for robot %s\n%s",this->robot_model_name_.c_str(),stream_in.str().c_str());
              elem->LinkEndChild((*block_it)->Clone());
          }
      }
    }
}

std::string URDF2Gazebo::getGeometrySize(boost::shared_ptr<urdf::Geometry> geometry, int *sizeCount, double *sizeVals)
{
    std::string type;
    
    switch (geometry->type)
    {
    case urdf::Geometry::BOX:
        type = "box";
        *sizeCount = 3;
        {
            boost::shared_ptr<const urdf::Box> box;
            box = boost::dynamic_pointer_cast< const urdf::Box >(geometry);
            sizeVals[0] = box->dim.x;
            sizeVals[1] = box->dim.y;
            sizeVals[2] = box->dim.z;
        }
        break;
    case urdf::Geometry::CYLINDER:
        type = "cylinder";
        *sizeCount = 2;
        {
            boost::shared_ptr<const urdf::Cylinder> cylinder;
            cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(geometry);
            sizeVals[0] = cylinder->radius;
            sizeVals[1] = cylinder->length;
        }
        break;
    case urdf::Geometry::SPHERE:
        type = "sphere";
        *sizeCount = 1;
        {
          boost::shared_ptr<const urdf::Sphere> sphere;
          sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(geometry);
          sizeVals[0] = sphere->radius;
        }
        break;
    case urdf::Geometry::MESH:
        type = "trimesh";
        *sizeCount = 3;
        {
          boost::shared_ptr<const urdf::Mesh> mesh;
          mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(geometry);
          sizeVals[0] = mesh->scale.x;
          sizeVals[1] = mesh->scale.y;
          sizeVals[2] = mesh->scale.z;
        }
        break;
    default:
        *sizeCount = 0;
        printf("Unknown body type: %d in geometry\n", geometry->type);
        break;
    }
    
    return type;
}

std::string URDF2Gazebo::getGeometryBoundingBox(boost::shared_ptr<urdf::Geometry> geometry, double *sizeVals)
{
    std::string type;
    
    switch (geometry->type)
    {
    case urdf::Geometry::BOX:
        type = "box";
        {
            boost::shared_ptr<const urdf::Box> box;
            box = boost::dynamic_pointer_cast<const urdf::Box >(geometry);
            sizeVals[0] = box->dim.x;
            sizeVals[1] = box->dim.y;
            sizeVals[2] = box->dim.z;
        }
        break;
    case urdf::Geometry::CYLINDER:
        type = "cylinder";
        {
            boost::shared_ptr<const urdf::Cylinder> cylinder;
            cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(geometry);
            sizeVals[0] = cylinder->radius * 2;
            sizeVals[1] = cylinder->radius * 2;
            sizeVals[2] = cylinder->length;
        }
        break;
    case urdf::Geometry::SPHERE:
        type = "sphere";
        {
            boost::shared_ptr<const urdf::Sphere> sphere;
            sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(geometry);
            sizeVals[0] = sizeVals[1] = sizeVals[2] = sphere->radius * 2;
        }
        break;
    case urdf::Geometry::MESH:
        type = "trimesh";
        {
            boost::shared_ptr<const urdf::Mesh> mesh;
            mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(geometry);
            sizeVals[0] = mesh->scale.x;
            sizeVals[1] = mesh->scale.y;
            sizeVals[2] = mesh->scale.z;
        }
        break;
    default:
        sizeVals[0] = sizeVals[1] = sizeVals[2] = 0;
        printf("Unknown body type: %d in geometry\n", geometry->type);
        break;
    }
    
    return type;
}

void URDF2Gazebo::convertLink(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link, const btTransform &transform, bool enforce_limits)
{
    btTransform currentTransform = transform;
    std::string type;

    int linkGeomSize;
    double linkSize[3];
    if (link->collision)
      if (link->collision->geometry)
        type = getGeometrySize(link->collision->geometry, &linkGeomSize, linkSize);
    
    // This should be made smarter.
    if(!link->visual) 
    {
      printf("ignoring link without visual tag: %s\n", link->name.c_str());
      return;
    }

    if(!link->inertial)
    {
      printf("ignoring link without inertial tag: %s\n", link->name.c_str());
      return;
    }

    if(link->inertial->mass == 0.0) 
    {
      printf("ignoring link with zero mass: %s\n", link->name.c_str());
      return;
    }

    if (!type.empty())
    {
        /* create new body */
        TiXmlElement *elem     = new TiXmlElement("body:" + type);
        
        /* set body name */
        elem->SetAttribute("name", link->name);


        /* set mass properties */
        addKeyValue(elem, "massMatrix", "true");
        addKeyValue(elem, "mass", values2str(1, &link->inertial->mass));
        
        addKeyValue(elem, "ixx", values2str(1, &link->inertial->ixx));
        addKeyValue(elem, "ixy", values2str(1, &link->inertial->ixy));
        addKeyValue(elem, "ixz", values2str(1, &link->inertial->ixz));
        addKeyValue(elem, "iyy", values2str(1, &link->inertial->iyy));
        addKeyValue(elem, "iyz", values2str(1, &link->inertial->iyz));
        addKeyValue(elem, "izz", values2str(1, &link->inertial->izz));
        
        addKeyValue(elem, "cx", values2str(1, &link->inertial->origin.position.x));
        addKeyValue(elem, "cy", values2str(1, &link->inertial->origin.position.y));
        addKeyValue(elem, "cz", values2str(1, &link->inertial->origin.position.z));
        
        double roll,pitch,yaw;
        link->inertial->origin.rotation.getRPY(roll,pitch,yaw);
        if (roll != 0 || pitch != 0 || yaw != 0)
            ROS_ERROR("rotation of inertial frame is not supported\n");

        /* compute global transform */
        btTransform localTransform;
        // this is the transform from parent link to current link
        setupTransform(localTransform, link->parent_joint->parent_to_joint_origin_transform);
        currentTransform *= localTransform;
        addTransform(elem, currentTransform);
        
        /* begin create geometry node */
        TiXmlElement *geom     = new TiXmlElement("geom:" + type);
        {    
            /* set its name */
            geom->SetAttribute("name", link->name + "_geom");
            
            /* set transform */
            addKeyValue(geom, "xyz", vector32str(link->collision->origin.position));
            double rpy[3];
            link->collision->origin.rotation.getRPY(rpy[0],rpy[1],rpy[2]);
            addKeyValue(geom, "rpy", values2str(3, rpy, rad2deg));
            
            if (link->collision->geometry->type == urdf::Geometry::MESH)
            {  
                boost::shared_ptr<const urdf::Mesh> mesh;
                mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(link->collision->geometry);
                /* set mesh size or scale */
                addKeyValue(geom, "scale", vector32str(mesh->scale));

                /* set mesh file */
                // strip extension from filename
                std::string tmp_extension(".stl");
                std::string tmp_slash("/meshes/");
                int pos0 = mesh->filename.rfind(tmp_slash);
                int pos1 = mesh->filename.find(tmp_extension,0);
                std::string mesh_filename = mesh->filename;
                mesh_filename.replace(pos1,mesh->filename.size()-pos1+1,std::string(""));
                if (pos0 != mesh_filename.npos)
                    mesh_filename.replace(0,pos0+8,std::string(""));
                // add mesh filename
                addKeyValue(geom, "mesh", mesh_filename + ".mesh");
                
            }
            else
            {
                /* set geometry size */
                addKeyValue(geom, "size", values2str(linkGeomSize, linkSize));
            }
            
            /* set additional data from extensions */
            insertGazeboExtensionGeom(geom,link->name);
            
            /* begin create visual node */
            TiXmlElement *visual = new TiXmlElement("visual");
            {
                /* compute the visualisation transfrom */
                btTransform coll;
                setupTransform(coll, link->collision->origin);
                coll.inverse();
                
                btTransform vis;
                setupTransform(vis, link->visual->origin);
                coll = coll.inverseTimes(vis);
                
                addTransform(visual, coll);
                
                /* set geometry size */                
                
                if (link->visual->geometry->type == urdf::Geometry::MESH)
                {  
                    boost::shared_ptr<const urdf::Mesh>  mesh;
                    mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(link->visual->geometry);
                    /* set mesh size or scale */
                    /*
                    if (link->visual->geometry->isSet["size"])
                        addKeyValue(visual, "size", values2str(3, mesh->size));        
                    else
                        addKeyValue(visual, "scale", values2str(3, mesh->scale));        
                    */
                    addKeyValue(visual, "scale", vector32str(mesh->scale));

                    /* set mesh file */
                    if (mesh->filename.empty())
                        addKeyValue(visual, "mesh", "unit_" + type);
                    else
                    {
                        // strip extension from filename
                        std::string tmp_extension(".stl");
                        std::string tmp_slash("/meshes/");
                        int pos1 = mesh->filename.find(tmp_extension,0);
                        int pos0 = mesh->filename.rfind(tmp_slash);
                        std::string mesh_filename = std::string(mesh->filename.c_str());
                        mesh_filename.replace(pos1,mesh_filename.size()-pos1+1,std::string(""));
                        if (pos0 != mesh_filename.npos)
                            mesh_filename.replace(0,pos0+8,std::string(""));
                        // add mesh filename
                        addKeyValue(visual, "mesh", mesh_filename + ".mesh");
                    }
                    
                }
                else
                {
                    double visualSize[3];
                    std::string visual_geom_type = getGeometryBoundingBox(link->visual->geometry, visualSize);
                    addKeyValue(visual, "scale", values2str(3, visualSize));
                    addKeyValue(visual, "mesh", "unit_" + visual_geom_type);
                }
                
                /* copy gazebo extensions data */
                insertGazeboExtensionVisual(visual,link->name);

            }
            /* end create visual node */
            
            geom->LinkEndChild(visual);
        }
        /* end create geometry node */
        
        /* add geometry to body */
        elem->LinkEndChild(geom);      
        
        /* copy gazebo extensions data */
        insertGazeboExtensionBody(elem,link->name);
        
        /* add body to document */
        root->LinkEndChild(elem);
        
        /* compute the joint tag */
        bool fixed = false;
        std::string jtype;
        if (link->parent_joint != NULL)
        switch (link->parent_joint->type)
        {
        case urdf::Joint::CONTINUOUS:
        case urdf::Joint::REVOLUTE:
            jtype = "hinge";
            break;
        case urdf::Joint::PRISMATIC:
            jtype = "slider";
            break;
        case urdf::Joint::FLOATING:
        case urdf::Joint::PLANAR:
            break;
        case urdf::Joint::FIXED:
            jtype = "hinge";
            fixed = true;
            break;
        default:
            printf("Unknown joint type: %d in link '%s'\n", link->parent_joint->type, link->name.c_str());
            break;
        }
        
        if (!jtype.empty())
        {
            TiXmlElement *joint = new TiXmlElement("joint:" + jtype);
            joint->SetAttribute("name", link->parent_joint->name);
            
            addKeyValue(joint, "body1", link->name);
            addKeyValue(joint, "body2", link->parent_link->name);
            addKeyValue(joint, "anchor", link->name);
            
            if (fixed)
            {
                addKeyValue(joint, "lowStop", "0");
                addKeyValue(joint, "highStop", "0");
                addKeyValue(joint, "axis", "1 0 0");
            }
            else
            {
                btTransform currentTransformCopy( transform );
                currentTransformCopy.setOrigin( btVector3(0, 0, 0) );
                btVector3 rotatedJointAxis = currentTransformCopy * btVector3( link->parent_joint->axis.x,
                                                                               link->parent_joint->axis.y,
                                                                               link->parent_joint->axis.z );
                double rotatedJointAxisArray[3] = { rotatedJointAxis.x(), rotatedJointAxis.y(), rotatedJointAxis.z() };
                addKeyValue(joint, "axis", values2str(3, rotatedJointAxisArray));
 
                // Joint Anchor is deprecated
                // btTransform currentAnchorTransform( currentTransform ); 
                // currentAnchorTransform.setOrigin( btVector3(0, 0, 0) ); 
                // btVector3 jointAnchor = currentAnchorTransform * btVector3( link->parent_joint->anchor.x,
                //                                                             link->parent_joint->anchor.y,
                //                                                             link->parent_joint->anchor.z ); 
                // double tmpAnchor[3] =  { jointAnchor.x(), jointAnchor.y(), jointAnchor.z() };
                // addKeyValue(joint, "anchorOffset", values2str(3, tmpAnchor));
                
                if (enforce_limits && link->parent_joint->limits)
                {
                    if (jtype == "slider")
                    {
                        addKeyValue(joint, "lowStop",  values2str(1, &link->parent_joint->limits->lower       ));
                        addKeyValue(joint, "highStop", values2str(1, &link->parent_joint->limits->upper       ));
                    }
                    else if (link->parent_joint->type != urdf::Joint::CONTINUOUS)
                    {
                        double *lowstop  = &link->parent_joint->limits->lower;
                        double *highstop = &link->parent_joint->limits->upper;
                        // enforce ode bounds, this will need to be fixed
                        if (*lowstop > 0)
                        {
                          ROS_WARN("urdf2gazebo: limiting lowStop to <= 0 degrees");
                          *lowstop = 0.0;
                        }
                        if (*lowstop < -(M_PI)*0.9)
                        {
                          ROS_WARN("urdf2gazebo: limiting lowStop to >= -(180)*0.9 degrees");
                          *lowstop = -(M_PI)*0.9;
                        }
                        if (*highstop < 0)
                        {
                          ROS_WARN("urdf2gazebo: limiting highStop to >= 0 degrees");
                          *highstop = 0.0;
                        }
                        if (*highstop > (M_PI)*0.9)
                        {
                          ROS_WARN("urdf2gazebo: limiting highStop to <= (180)*0.9 degrees");
                          *highstop = (M_PI)*0.9;
                        }
                        addKeyValue(joint, "lowStop",  values2str(1, &link->parent_joint->limits->lower, rad2deg));
                        addKeyValue(joint, "highStop", values2str(1, &link->parent_joint->limits->upper, rad2deg));
                    }
                }
            }
            /* copy gazebo extensions data */
            insertGazeboExtensionJoint(joint,link->parent_joint->name);

            /* add joint to document */
            root->LinkEndChild(joint);
        }
    }
    
    for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
        convertLink(root, link->child_links[i], currentTransform, enforce_limits);
}

void URDF2Gazebo::convert(TiXmlDocument &urdf_in, TiXmlDocument &gazebo_xml_out, bool enforce_limits, urdf::Vector3 initial_xyz, urdf::Vector3 initial_rpy,bool xml_declaration)
{

    // copy model to a string
    std::ostringstream stream_in;
    stream_in << urdf_in;

    /* Create a RobotModel from string */
    urdf::Model robot_model;

    if (!robot_model.initString(stream_in.str().c_str()))
    {
        ROS_ERROR("Unable to load robot model from param server robot_description\n");  
        exit(2);
    }


    // add xml declaration if needed
    if (xml_declaration)
    {
      TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
      gazebo_xml_out.LinkEndChild(decl);
    }
    
    /* create root element and define needed namespaces */
    TiXmlElement *robot = new TiXmlElement("model:physical");
    robot->SetAttribute("xmlns:gazebo", "http://playerstage.sourceforge.net/gazebo/xmlschema/#gz");
    robot->SetAttribute("xmlns:model", "http://playerstage.sourceforge.net/gazebo/xmlschema/#model");
    robot->SetAttribute("xmlns:sensor", "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor");
    robot->SetAttribute("xmlns:body", "http://playerstage.sourceforge.net/gazebo/xmlschema/#body");
    robot->SetAttribute("xmlns:geom", "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom");
    robot->SetAttribute("xmlns:joint", "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint");
    robot->SetAttribute("xmlns:controller", "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller");
    robot->SetAttribute("xmlns:interface", "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface");
    robot->SetAttribute("xmlns:rendering", "http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering");
    robot->SetAttribute("xmlns:physics", "http://playerstage.sourceforge.net/gazebo/xmlschema/#physics");
    
    // Create a node to enclose the robot body
    robot->SetAttribute("name", robot_model_name_);
    
    /* set the transform for the whole model to identity */
    addKeyValue(robot, "xyz", vector32str(initial_xyz));
    addKeyValue(robot, "rpy", vector32str(initial_rpy));
    btTransform transform;
    transform.setIdentity();
    
    /* parse gazebo extension */
    parseGazeboExtension( urdf_in );

    /* start conversion */
    boost::shared_ptr<const urdf::Link> root_link = robot_model.getRoot();
    for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator child = root_link->child_links.begin(); child != root_link->child_links.end(); child++)
        convertLink(robot, (*child), transform, enforce_limits);
    
    /* find all data item types */
    insertGazeboExtensionRobot(robot);
    
    std::ostringstream stream_robot;
    stream_robot << *(robot);
    ROS_DEBUG("\n\n\nentire robot:\n%s",stream_robot.str().c_str());
    gazebo_xml_out.LinkEndChild(robot);
}










