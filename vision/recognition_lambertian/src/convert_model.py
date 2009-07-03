#!/usr/bin/env python


import numpy
import roslib; roslib.load_manifest('recognition_lambertian')
import rospy
import os
import ply_import
from scipy.io.numpyio import fwrite, fread
import sys

from robot_msgs.msg import *

#import psyco
#psyco.full()


def listDirectory(directory, fileExtList):                                        
    "get list of file info objects for files of particular extensions" 
    fileList = [os.path.normcase(f)
                for f in os.listdir(directory)]
    fileList = [os.path.join(directory, f)
               for f in fileList
                if os.path.splitext(f)[1] in fileExtList]
    return fileList


def frange(from_, to, step):
    while from_ <= to:
        yield from_
        from_ += step


class VoxelGrid:
    
    def __init__(self, point_cloud, x_res, y_res, z_res):
        self.x_res = x_res
        self.y_res = y_res
        self.z_res = z_res

        self.template = point_cloud
        self.x_min, self.y_min, self.z_min = numpy.min(point_cloud,0)
        self.x_max, self.y_max, self.z_max = numpy.max(point_cloud,0)        
        self.x_d= (self.x_max-self.x_min)/(self.x_res-1)
        self.y_d= (self.y_max-self.y_min)/(self.y_res-1)
        self.z_d= (self.z_max-self.z_min)/(self.z_res-1)
                
        self.point_cloud = point_cloud - (self.x_min, self.y_min, self.z_min)
        self.point_cloud /= (self.x_d,self.y_d,self.z_d)
        self.point_cloud = numpy.int32(numpy.floor(self.point_cloud))
        
        self.truncate = 20
        print "Computing distance transform"
        self.create_voxel_grid()
        print "Done computing DT"
#        print self.grid[:,:,-1]
        
    def in_bounds(self, p):        
        return p[0]>=0 and p[0]<self.x_res and p[1]>=0 and p[1]<self.y_res and p[2]>=0 and p[2]<self.z_res
        
    def create_voxel_grid(self):
        
        self.grid = numpy.empty((self.x_res,self.y_res,self.z_res), dtype=numpy.float32)
        self.grid[:] = -1
        q = []
        for p in self.point_cloud:
            if self.grid[p[0],p[1],p[2]] != 0:
                q.append(p)        
            self.grid[p[0],p[1],p[2]] = 0

        dir = numpy.array([ [0,0,1], [0,0,-1], [0,1,0], [0,-1,0], [1,0,0], [-1,0,0] ])
        
        while q!=[]:
            p = q.pop(0)
            v = self.grid[p[0],p[1],p[2]]
            for ofs in dir:
                n = p+ofs
                if self.in_bounds(n):
                    vn = self.grid[n[0],n[1],n[2]]
                    if vn<0:
                        self.grid[n[0],n[1],n[2]] = min(v+1, self.truncate)
                        q.append(n)
            
    def match_score(self, template, location):
        
        # move template to new location
        tmpl = template - location
                
        #transform template into grid coordinates
        tmpl -= (self.x_min,self.y_min,self.z_min)
        tmpl /= (self.x_d,self.y_d,self.z_d)
        coords = numpy.int32(tmpl)
 
        score = 0.0
        for p in coords:
            if self.in_bounds(p):
                score += self.grid[p[0],p[1],p[2]]
            else:
                score += self.truncate
        score /= len(coords)
        
        return score
    
    def save(self, filename):
        print "Saving %s"%filename
        f = open(filename,"wb")
        print >>f, "%d %d %d"%(self.x_res, self.y_res, self.z_res)
        print >>f, "%g %g %g"%(self.x_min, self.y_min, self.z_min)
        print >>f, "%g %g %g"%(self.x_max, self.y_max, self.z_max)
        fwrite(f, self.grid.size, self.grid)
        f.close()

        
        


def convert(ply_filename, filename):
    print "Converting %s to %s"%(ply_filename, filename)
    obj_spec,obj = ply_import.read(ply_filename)
    model_vertex = numpy.array(obj['vertex']) / 1000
    grid = VoxelGrid(model_vertex,50,50,50)
    grid.save(filename)
    
        
def usage():
    print "%s: model.ply new_model"%sys.argv[0]
    sys.exit(1)
        
if __name__ == '__main__':
    if len(sys.argv) != 3:
        usage()
    convert(sys.argv[1],sys.argv[2])
