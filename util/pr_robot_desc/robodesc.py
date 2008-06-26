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
        
    def propStr(self):
        return ''
    def __str__(self):
        return "[%s%s]"%(self.tagName, self.propStr())
    
    def loadTag(self, tag, name):
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
    def _validate(self):
        for p in self.vprops:
            val = getattr(self, p) 
            if val is None:
                raise XmlParseException("<%s> tag is missing a <%s> spec"%(self.tagName, p))
            if isinstance(val, _XObject):
                val._validate()

class Joint(_XObject):
    def __init__(self):
        super(Joint, self).__init__('joint')
        self.name        = None
        self.type        = None
        self.anchor      = None
        # optional props
        self.limit       = None
        self.axis        = None
        self.calibration = None
        
        self.vprops.append('anchor') #validation
        self.aprops.extend(['name', 'type'])

    def propStr(self):
        return super(Joint, self).propStr() + ' name: %s type: %s anchor: %s'%(self.name, self.type, self.anchor)
    
    def loadTag(self, tag, name):
        if name == 'anchor':
            self.anchor = getText(tag)
        elif name == 'axis':
            self.axis = getTriple(tag)
        elif name == 'limit':
            self.limit = getDouble(tag)
        elif name == 'calibration':
            self.calibration = getDouble(tag)
        else:
            super(Joint, self).loadTag(tag, name)
        
class Geometry(_XObject):
    #TODO: spec is unfinished here
    def __init__(self):
        super(Geometry, self).__init__('geometry')
        self.name    = None
        self.type    = None        
        self.aprops.extend(['name', 'type'])

    def propStr(self):
        return super(Geometry, self).propStr() + ' name: %s type: %s'%(self.name, self.type)

    def loadTag(self, tag, name):
        if name == 'mass':
            self.mass = getText(tag)
        elif name == 'com':
            self.com = getTriple(tag)
        elif name == 'inertia':
            val = getText(tag).split(' ')
            if len(val) != 6:
                raise XmlParseException("Invalid <%s> text: %s"%(name, getText(tag)))
            self.inertia = val
        else:
            pass #currently ignore size, etc...
            #super(Geometry, self).loadTag(tag, name)
    
class Inertial(_XObject):
    def __init__(self):
        super(Inertial, self).__init__('inertial')
        self.mass    = None
        self.com     = None
        self.inertia = None
        self.vprops.extend(['mass', 'com', 'inertia'])

    def propStr(self):
        return  super(Inertial, self).propStr() + ' mass: %s com: %s inertia: %s'%(self.mass, ' '.join(self.com), ' '.join(self.inertia))

    def loadTag(self, tag, name):
        if name == 'mass':
            self.mass = getText(tag)
        elif name == 'com':
            self.com = getTriple(tag)
        elif name == 'inertia':
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
        self.xyz        = None
        self.rpy        = None
        self.material   = None
        self.geometries = []
        self.vprops.extend(['xyz', 'rpy', 'material']) #validation props
        
    def propStr(self):
        georepr = '\n'.join([str(g) for g in self.geometries])
        return  super(_GObject, self).propStr() + ' xyz: %s rpy: %s material: %s geometries: %s'%(' '.join(self.xyz), ' '.join(self.rpy), self.material, georepr)

    def addGeometry(self, g):
        self.geometries.append(g)

    def _validate(self):
        super(_GObject, self)._validate()
        for g in self.geometries:
            g._validate()

    def loadTag(self, tag, name):
        if name == 'material':
            self.material = getText(tag)
        elif name == 'rpy':
            self.rpy = getTriple(tag)
        elif name == 'xyz':
            self.xyz = getTriple(tag)
        elif name == 'geometry':
            self.addGeometry(Geometry().loadDom(tag))
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
        self.name      = None
        self.parent    = None
        self.rpy       = None
        self.inertial  = None
        self.visual    = None
        self.collision = None
        self.aprops.append('name')
        self.vprops.extend(['parent', 'rpy', 'inertial', 'visual', 'collision'])

    def propStr(self):
        return super(_RObject, self).propStr() + ' name: %s parent: %s rpy: %s inertial: %s visual: %s collision: %s'%(self.name, self.parent, ' '.join(self.rpy), self.inertial, self.visual, self.collision)

    def _validate(self):
        super(_RObject, self)._validate()
        self.visual._validate()
        self.inertial._validate()
        self.collision._validate()                

    def loadTag(self, tag, name):
        if name == 'parent':
            self.parent = getText(tag)
        elif name == 'inertial':
            self.inertial = Inertial().loadDom(tag)
        elif name == 'visual':
            self.visual = Visual().loadDom(tag)
        elif name == 'collision':
            self.collision = Collision().loadDom(tag)
        elif name == 'rpy':
            self.rpy = getTriple(tag)
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

    def propStr(self):
        return super(Sensor, self).propStr() + ' type: %s xyz: %s calibration: %s'%(self.type, ' '.join(self.xyz), self.calibration)
    
    def loadTag(self, tag, name):
        if name == 'calibration':
            self.calibration = getText(tag)
        elif name == 'xyz':
            self.xyz = getTriple(tag)
        else:
            super(Sensor, self).loadTag(tag, name)

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
        
class RobotDesc(object):
    def __init__(self):
        self.sensors = {}
        self.links   = {}
    def validate(self):
        for s in self.sensors.itervalues():
            s._validate()
        for s in self.links.itervalues():
            s._validate()

    def addSensor(self, s):
        self.sensors[s.name] = s
    def addLink(self, s):
        self.links[s.name] = s
    def __str__(self):
        for s in self.sensors:
            print str(s), repr(s)
        return "[ROBOT sensors: [%s] links: [%s] ]"%('\n'.join([str(s) for s in self.sensors.itervalues()]), '\n'.join([str(s) for s in self.links.itervalues()]))

def load(filename):
    desc = RobotDesc()
    dom = parse(filename)
    root    = dom.getElementsByTagName('robot')[0]
    links   = root.getElementsByTagName('link')
    for tag in [t for t in links if t.nodeType == Node.ELEMENT_NODE]:
        desc.addLink(Link().loadDom(tag))
    sensors = root.getElementsByTagName('sensor')
    for tag in [t for t in sensors if t.nodeType == Node.ELEMENT_NODE]:
        desc.addSensor(Sensor().loadDom(tag))
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
