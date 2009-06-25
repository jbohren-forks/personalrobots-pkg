#!/usr/bin/env python

# Copyright (C) 2004, 2005: Bruce Merry, bmerry@cs.uct.ac.za
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.


# 20th Oct 2008, 0.93 - Updated by Campbell Barton AKA ideasman42, use Mesh rather then NMesh, dont import normals, vcolors work again.
# Updated by Campbell Barton AKA Ideasman42, 10% faster code.

# Portions of this code are taken from mod_meshtools.py in Blender
# 2.32.

try:
    import re, struct
except:
    struct= None

class element_spec(object):
    __slots__ = 'name', 'count', 'properties'
    def __init__(self, name, count):
        self.name = name
        self.count = count
        self.properties = []

    def load(self, format, stream):
        if format == 'ascii':
            stream = re.split('\s+', stream.readline())
        return map(lambda x: x.load(format, stream), self.properties)

    def index(self, name):
        for i, p in enumerate(self.properties):
            if p.name == name: return i
        return -1

class property_spec(object):
    __slots__ = 'name', 'list_type', 'numeric_type'
    def __init__(self, name, list_type, numeric_type):
        self.name = name
        self.list_type = list_type
        self.numeric_type = numeric_type

    def read_format(self, format, count, num_type, stream):
        if format == 'ascii':
            if (num_type == 's'):
                ans = []
                for i in xrange(count):
                    s = stream[i]
                    if len(s) < 2 or s[0] != '"' or s[-1] != '"':
                        print 'Invalid string', s
                        print 'Note: ply_import.py does not handle whitespace in strings'
                        return None
                    ans.append(s[1:-1])
                stream[:count] = []
                return ans
            if (num_type == 'f' or num_type == 'd'):
                mapper = float
            else:
                mapper = int
            ans = map(lambda x: mapper(x), stream[:count])
            stream[:count] = []
            return ans
        else:
            if (num_type == 's'):
                ans = []
                for i in xrange(count):
                    fmt = format + 'i'
                    data = stream.read(struct.calcsize(fmt))
                    length = struct.unpack(fmt, data)[0]
                    fmt = '%s%is' % (format, length)
                    data = stream.read(struct.calcsize(fmt))
                    s = struct.unpack(fmt, data)[0]
                    ans.append(s[:-1]) # strip the NULL
                return ans
            else:
                fmt = '%s%i%s' % (format, count, num_type)
                data = stream.read(struct.calcsize(fmt));
                return struct.unpack(fmt, data)

    def load(self, format, stream):
        if (self.list_type != None):
            count = int(self.read_format(format, 1, self.list_type, stream)[0])
            return self.read_format(format, count, self.numeric_type, stream)
        else:
            return self.read_format(format, 1, self.numeric_type, stream)[0]

class object_spec(object):
    __slots__ = 'specs'
    'A list of element_specs'
    def __init__(self):
        self.specs = []
    
    def load(self, format, stream):
        return dict([(i.name,[i.load(format, stream) for j in xrange(i.count) ]) for i in self.specs])
        
        '''
        # Longhand for above LC
        answer = {}
        for i in self.specs:
            answer[i.name] = []
            for j in xrange(i.count):
                answer[i.name].append(i.load(format, stream))
        return answer
            '''
        

def read(filename):
    format = ''
    version = '1.0'
    format_specs = {'binary_little_endian': '<',
            'binary_big_endian': '>',
            'ascii': 'ascii'}
    type_specs = {'char': 'b',
              'uchar': 'B',
              'int8': 'b',
              'uint8': 'B',
              'int16': 'h',
              'uint16': 'H',
              'ushort': 'H',
              'int': 'i',
              'int32': 'i',
              'uint': 'I',
              'uint32': 'I',
              'float': 'f',
              'float32': 'f',
              'float64': 'd',
              'double': 'd',
              'string': 's'}
    obj_spec = object_spec()

    try:
        file = open(filename, 'rU') # Only for parsing the header, not binary data
        signature = file.readline()
        
        if not signature.startswith('ply'):
            print 'Signature line was invalid'
            return None
        
        while 1:
            tokens = re.split(r'[ \n]+', file.readline())
            
            if (len(tokens) == 0):
                continue
            if (tokens[0] == 'end_header'):
                break
            elif (tokens[0] == 'comment' or tokens[0] == 'obj_info'):
                continue
            elif (tokens[0] == 'format'):
                if (len(tokens) < 3):
                    print 'Invalid format line'
                    return None
                if (tokens[1] not in format_specs): # .keys()): # keys is implicit
                    print 'Unknown format', tokens[1]
                    return None
                if (tokens[2] != version):
                    print 'Unknown version', tokens[2]
                    return None
                format = tokens[1]
            elif (tokens[0] == 'element'):
                if (len(tokens) < 3):
                    print 'Invalid element line'
                    return None
                obj_spec.specs.append(element_spec(tokens[1], int(tokens[2])))
            elif (tokens[0] == 'property'):
                if (not len(obj_spec.specs)):
                    print 'Property without element'
                    return None
                if (tokens[1] == 'list'):
                    obj_spec.specs[-1].properties.append(property_spec(tokens[4], type_specs[tokens[2]], type_specs[tokens[3]]))
                else:
                    obj_spec.specs[-1].properties.append(property_spec(tokens[2], None, type_specs[tokens[1]]))
        
        if format != 'ascii':
            file.close() # was ascii, now binary
            file = open(filename, 'rb')
            
            # skip the header...
            while not file.readline().startswith('end_header'):
                pass
        
        obj = obj_spec.load(format_specs[format], file)
        
    except IOError, (errno, strerror):
        try:    file.close()
        except:    pass
        
        return None
    try:    file.close()
    except:    pass
    
    return (obj_spec, obj);
    


def main():
    import sys
    obj_spec, obj = read(sys.argv[1])

if __name__=='__main__':
    main()

