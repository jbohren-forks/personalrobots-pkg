#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

# Given a set of parameters, generates the messages, service types, and
# classes to allow runtime reconfiguration. Documentation of a node's
# parameters is a handy byproduct.

import roslib; roslib.load_manifest("dynamic_reconfigure")
import roslib.packages
from string import Template
import os

class ParameterGenerator:
    minval = {
            'float32' : '-1./0.',
            'float64' : '-1./0.',
            'uint8' : '0',
            'uint16' : '0',
            'uint32' : '0',
            'uint64' : '0',
            'int8' : '1LL << 7',
            'int16' : '1LL << 15',
            'int32' : '1LL << 31',
            'int64' : '1LL << 63',
            'time' : 'ros::Time(0, 0)',
            'duration' : 'ros::Duration(1LL << 31, 1LL << 31)',
            'string' : '""',
            }
            
    maxval = {
            'float32' : '1./0.',
            'float64' : '1./0.',
            'uint8' : '-1LL',
            'uint16' : '-1LL',
            'uint32' : '-1LL',
            'uint64' : '-1LL',
            'int8' : '~(1LL << 7)',
            'int16' : '~(1LL << 15)',
            'int32' : '~(1LL << 31)',
            'int64' : '~(1LL << 63)',
            'time' : 'ros::Time(-1LL, -1LL)',
            'duration' : 'ros::Duration(~(1LL << 31), ~(1LL << 31))',
            'string' : '""',
            }
    
    defval = {
            'float32' : '0',
            'float64' : '0',
            'uint8' : '0',
            'uint16' : '0',
            'uint32' : '0',
            'uint64' : '0',
            'int8' : '0',
            'int16' : '0',
            'int32' : '0',
            'int64' : '0',
            'time' : 'ros::Time(0, 0)',
            'duration' : 'ros::Duration(0, 0)',
            'string' : '""',
            }
            
    def __init__(self, pkgname, name):
        self.parameters = []
        self.pkgname = pkgname
        self.pkgpath = roslib.packages.get_pkg_dir(pkgname)
        self.dynconfpath = roslib.packages.get_pkg_dir("dynamic_reconfigure")
        self.name = name

    def add(self, name, type, level, description, default = None, min = None, max = None):
        if min == None:
            min = self.minval[type]
        if max == None:
            max = self.maxval[type]
        if default == None:
            default = self.defaultval[type]
        self.parameters.append({
            'name' : name,
            'type' : type,
            'default' : default,
            'level' : level,
            'description' : description,
            'min' : min,
            'max' : max,
            })

    def mkdirabs(self, path):
        if os.path.isdir(path):
            pass
        elif os.path.isfile(path):
            raise OSError("Error creating directory %s, a file with the same name exists" %path)
        else:
            head, tail = os.path.split(path)
            if head and not os.path.isdir(head):
                self.mkdir(head)
            if tail:
                os.mkdir(path)

    def mkdir(self, path):
        path = os.path.join(self.pkgpath, path)
        self.mkdirabs(path)

    def generate(self):
        self.generateconfigmanipulator()
        self.generatemsg()
        self.generatesetsrv()
        self.generategetsrv()
        self.generatedoc()

    def generatedoc(self):
        self.mkdir("dox")
        f = open(os.path.join(self.pkgpath, "dox", self.name+".dox"), 'w')
        print >> f, "/**"
        print >> f, "\\subsection parameters ROS parameters"
        print >> f
        print >> f, "Reads and maintains the following parameters on the ROS server"
        print >> f
        for param in self.parameters:
          print >> f, Template("- \b \"~$name\" : \b [$type] $description").substitute(param)
        print >> f
        print >> f, "*/"
        f.close()

    def crepr(self, param, val):
        type = param["type"]
        if type == 'string':
            return '"'+param[val]+'"'
        if 'uint' in type:
            return str(param[val])+'ULL'
        if 'int' in type:
            return str(param[val])+'LL'
        if 'time' in type:
            return 'ros::Time('+str(param[val])+')'
        if 'duration' in type:
            return 'ros::Duration('+str(param[val])+')'
        if  'float' in types:
            return str(param[val])

    def joinlines(self, lines):
        return ''.join(['      '+line+'\n' for line in lines])


    def generateconfigmanipulator(self):
        # Read the configuration manipulator template.
        f = open(os.path.join(self.dynconfpath, "templates", "configmanipulator.h"))
        configmanipulator = f.read()
        f.close()
        
        # Write the configuration manipulator.
        self.mkdir(os.path.join("cfg", "cpp", self.pkgname))
        f = open(os.path.join(self.pkgpath, "cfg", "cpp", self.pkgname, self.name+"Manipulator.h"), 'w')
        readparam = []
        writeparam = []
        changelvl = []
        defminmax = []
        for param in self.parameters:
            defminmax.append(Template("min.$name = $min").substitute(param, min=self.crepr(param, "min")))
            defminmax.append(Template("max.$name = $max").substitute(param, min=self.crepr(param, "max")))
            defminmax.append(Template("defaults.$name = $max").substitute(param, min=self.crepr(param, "default")))
            changelvl.append(Template("if (config1.$name != config2.$name) changelvl |= $level;").substitute(param))
            writeparam.append(Template("nh.setParam(\"~$name\", config.$name)").substitute(param))
            readparam.append(Template("nh.getParam(\"~$name\", config.$name)").substitute(param))
        defminmax = self.joinlines(defminmax)
        changelvl = self.joinlines(changelvl)
        writeparam = self.joinlines(writeparam)
        readparam = self.joinlines(readparam)
        f.write(Template(configmanipulator).substitute(uname=self.name.upper(), name = self.name, 
            pkgname = self.pkgname, readparam = readparam, writeparam = writeparam, 
            changelvl = changelvl, defminmax = defminmax))
        f.close()

    def generatemsg(self):
        self.mkdir("msg")
        f = open(os.path.join(self.pkgpath, "msg", self.name+".msg"), 'w')
        print >> f, "# This is an autogerenated file. Please do not edit."
        for param in self.parameters:
            print >> f, Template("$type $name # $description").substitute(param)
        f.close()

    def generategetsrv(self):
        self.mkdir("srv")
        f = open(os.path.join(self.pkgpath, "srv", "Get"+self.name+".srv"), 'w')
        print >> f, "# This is an autogerenated file. Please do not edit."
        print >> f, "---" 
        print >> f, self.name, "config", "# Current configuration of node."
        print >> f, self.name, "defaults", "# Minimum values where appropriate."
        print >> f, self.name, "min", "# Minimum values where appropriate."
        print >> f, self.name, "max", "# Maximum values where appropriate."
        f.close()

    def generatesetsrv(self):
        self.mkdir("srv")
        f = open(os.path.join(self.pkgpath, "srv", "Set"+self.name+".srv"), 'w')
        print >> f, "# This is an autogerenated file. Please do not edit."
        print >> f, self.name, "config", "# Requested node configuration."
        print >> f, "---"        
        print >> f, self.name, "config", "# What the node's configuration was actually set to."
        f.close()
