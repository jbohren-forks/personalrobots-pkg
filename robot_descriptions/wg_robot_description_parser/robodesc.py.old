#!/usr/bin/env python
import os, sys
from xml.dom.minidom import parse, parseString
from xml.dom import Node 

class XmlParseException(Exception): pass

def getText(tag):
    return reduce(lambda x, y: x + y, [c.data for c in tag.childNodes if c.nodeType == Node.TEXT_NODE], "").strip()
def getTriple(tag):
    val = getText(tag).split(' ')
    if len(val) != 3:
        raise XmlParseException("Invalid <%s> text: %s"%(tag.tagName, getText(tag)))
    return val
def getDouble(tag):
    val = getText(tag).split(' ')
    if len(val) != 2:
        raise XmlParseException("Invalid <%s> text: %s"%(tag.tagName, getText(tag)))
    return val

## Xml-based object 
class _XObject(object):
    def __init__(self, tagName):
        self.tagName = tagName
        self.vprops  = [] #validation properties -- properties that must be not None
        self.aprops  = [] #attribute properties -- properties set from node attribute values
        self.textprops  = [] #text-based properties
        self.tripleprops = []
        self.doubleprops = []        
        
    def propStr(self):
        return ''
    def __str__(self):
        return "[%s%s]"%(self.tagName, self.propStr())
    
    def loadTag(self, tag, name):
        if name in self.textprops:
            setattr(self, name, getText(tag))
        elif name in self.tripleprops:
            setattr(self, name, getTriple(tag))
        elif name in self.doubleprops:
            setattr(self, name, getDouble(tag))
        raise XmlParseException("unknown tag <%s>"%name)
    def loadDom(self, tag):
        if tag.tagName != self.tagName:
            raise XmlParseException("Internal error: <%s> does not match <%s>"%(name, self.tagName))
        for p in self.aprops:
            val = tag.getAttribute('name')
            if val is None:
                raise XmlParseException("<%s> tag must have a '%s' attribute"%(self.tagName, p))
            setattr(self, p, val)
        for c in [c for c in tag.childNodes if c.nodeType == Node.ELEMENT_NODE]:
            self.loadTag(c, c.tagName)
        return self
    def validate(self):
        for p in self.vprops:
            val = getattr(self, p) 
            if val is None:
                raise XmlParseException("<%s> tag is missing a <%s> spec"%(self.tagName, p))
            if isinstance(val, _XObject):
                val.validate()

class Joint(_XObject):
    def __init__(self):
        super(Joint, self).__init__('joint')
        self.name = self.type = self.anchor = self.limit = self.axis = self.calibration = None
        self.vprops.append('anchor') #validation
        self.aprops.extend(['name', 'type'])
        self.textprops.append('anchor')
        self.tripleprops.append('axis')        
        self.doubleprops.append('limit', 'calibration')
        
    def propStr(self):
        return super(Joint, self).propStr() + ' name: %s type: %s anchor: %s'%(self.name, self.type, self.anchor)
    
class Geometry(_XObject):
    #TODO: spec is unfinished here
    def __init__(self):
        super(Geometry, self).__init__('geometry')
        self.name = self.type = None
        self.aprops.extend(['name', 'type'])
        
    def propStr(self):
        return super(Geometry, self).propStr() + ' name: %s type: %s'%(self.name, self.type)
    
class Inertial(_XObject):
    def __init__(self):
        super(Inertial, self).__init__('inertial')
        self.mass = self.com = self.inertia = None
        self.vprops.extend(['mass', 'com', 'inertia'])
        self.textprops.append('mass')
        self.tripleprops.append('com')        

    def propStr(self):
        return  super(Inertial, self).propStr() + ' mass: %s com: %s inertia: %s'%(self.mass, ' '.join(self.com), ' '.join(self.inertia))

    def loadTag(self, tag, name):
        if name == 'inertia':
            val = getText(tag).split(' ')
            if len(val) != 6:
                raise XmlParseException("Invalid <%s> text: %s"%(name, getText(tag)))
            self.inertia = val
        else:
            super(Inertial, self).loadTag(tag, name)
    
## Xml-based object with geometry properties
class _GObject(_XObject):
    def __init__(self, tagName):
        super(_GObject, self).__init__(tagName)
        self.xyz = self.rpy = self.material = None
        self.geometries = []
        self.vprops.extend(['xyz', 'rpy', 'material']) #validation props
        self.tripleprops.extend('rpy', 'xyz')
        self.textprops.append('material')
        
    def propStr(self):
        georepr = '\n'.join([str(g) for g in self.geometries])
        return  super(_GObject, self).propStr() + ' xyz: %s rpy: %s material: %s geometries: %s'%(' '.join(self.xyz), ' '.join(self.rpy), self.material, georepr)

    def validate(self):
        super(_GObject, self).validate()
        for g in self.geometries:
            g.validate()

    def loadTag(self, tag, name):
        if name == 'geometry':
            self.geometries.append(Geometry().loadDom(tag))
        else:
            super(_GObject, self).loadTag(tag, name) 

class Visual(_GObject):
    def __init__(self):
        super(Visual, self).__init__('visual')
class Collision(_GObject):
    def __init__(self):
        super(Collision, self).__init__('collision')
        
## Xml-based Robot object (e.g. sensor, link)
class _RObject(_XObject):
    
    def __init__(self, tagName):
        super(_RObject, self).__init__(tagName)
        self.name = self.parent = self.rpy = self.inertial = self.visual = self.collision = None
        self.aprops.append('name')
        self.vprops.extend(['parent', 'rpy', 'inertial', 'visual', 'collision'])
        self.textprops.append('parent')
        self.tripleprops.extend('rpy')
        
    def propStr(self):
        return super(_RObject, self).propStr() + ' name: %s parent: %s rpy: %s inertial: %s visual: %s collision: %s'%(self.name, self.parent, ' '.join(self.rpy), self.inertial, self.visual, self.collision)

    def validate(self):
        super(_RObject, self).validate()
        self.visual.validate()
        self.inertial.validate()
        self.collision.validate()                

    def loadTag(self, tag, name):
        if name == 'inertial':
            self.inertial = Inertial().loadDom(tag)
        elif name == 'visual':
            self.visual = Visual().loadDom(tag)
        elif name == 'collision':
            self.collision = Collision().loadDom(tag)
        else:
            super(_RObject, self).loadTag(tag, name)
    
class Sensor(_RObject):
    def __init__(self):
        super(Sensor, self).__init__('sensor')
        self.type        = None
        self.xyz         = None
        self.calibration = None
        self.vprops.extend(['xyz', 'calibration']) #validation
        self.aprops.append('type')
        self.textprops.append('calibration')
        self.tripleprops.append('xyz')
        
    def propStr(self):
        return super(Sensor, self).propStr() + ' type: %s xyz: %s calibration: %s'%(self.type, ' '.join(self.xyz), self.calibration)
    
class Link(_RObject):
    def __init__(self):
        super(Link, self).__init__('link')
        self.joint = None
        self.vprops.append('joint') #validation

    def propStr(self):
        return super(Link, self).propStr() + ' joint: %s'%(self.joint)

    def loadTag(self, tag, name):
        if name == 'joint':
            self.joint = Joint().loadDom(tag)
        else:
            super(Link, self).loadTag(tag, name)
        
class RobotDesc(_XObject):
    def __init__(self):
        super(RobotDesc, self).__init__('robot')
        self.sensors = []
        self.links   = []
    def validate(self):
        super(RobotDesc, self).validate()
        for s in self.sensors + self.links:
            s.validate()

    def loadTag(self, tag, name):
        if name == 'sensor':
            self.sensors.append(Sensor().loadDom(tag))
        elif name == 'link':
            self.links.append(Link().loadDom(tag))
        else:
            super(RobotDesc, self).loadTag(tag, name)

    def propStr(self):
        return super(RobotDesc, self).propStr() + " sensors: [%s] links: [%s]"%('\n'.join([str(s) for s in self.sensors]), '\n'.join([str(s) for s in self.links]))

def load(filename):
    dom = parse(filename)
    desc = RobotDesc().loadDom(dom.getElementsByTagName('robot')[0])
    desc.validate()
    return desc

def main():
    if len(sys.argv) != 2:
        print "Usage: %s FILENAME"%(sys.argv[0])
        sys.exit(1)
    try:
        robot = load(sys.argv[1])
        print str(robot)
    except XmlParseException, e:
        print >> sys.stderr, e

if __name__ == "__main__":
    main()
