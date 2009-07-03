#!/usr/bin/env python


import numpy
import roslib; roslib.load_manifest('recognition_lambertian')
import rospy
import os
import ply_import

from robot_msgs.msg import *
from recognition_lambertian.srv import *
from visualization_msgs.msg import Marker

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
            
    def show_grid(self, publisher):
        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "table_frame"        
        marker.ns = "voxel_grid"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.x_d
        marker.scale.y = self.y_d
        marker.scale.z = self.z_d
        marker.color.a = 0.7
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        
        marker2 = Marker()
        marker2.header.stamp = rospy.get_rostime()
        marker2.header.frame_id = "table_frame"        
        marker2.ns = "voxel_grid"
        marker2.id = 2
        marker2.type = Marker.POINTS
        marker2.action = Marker.ADD
        marker2.pose.position.x = 0
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.scale.x = self.x_d/3
        marker2.scale.y = self.y_d/3
        marker2.scale.z = self.z_d/3
        marker2.color.a = 0.1
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        for i in xrange(self.x_res):
            for j in xrange(self.y_res):
                for k in xrange(self.z_res):
                    if self.grid[i,j,k]==0:
                        
                        marker.points.append(Point(i*self.x_d+self.x_min,j*self.y_d+self.y_min,k*self.z_d+self.z_min))
                    else:
                        marker2.points.append(Point(i*self.x_d+self.x_min,j*self.y_d+self.y_min,k*self.z_d+self.z_min))
        publisher.publish(marker)
        publisher.publish(marker2)
        
        

class ModelFitNode():
    
    colors = [
              [0,0,1],
              [0,1,0],
              [1,0,0],
              [1,1,0],
              [0,1,1],
              [1,0,1]
              ]
    
    
    def __init__(self):
        rospy.init_node('model_fit')
        service = rospy.Service("recognition_lambertian/model_fit", ModelFit, self.model_fit)
        self.maker_pub = rospy.Publisher("visualization_marker",Marker)
        print "Initialized"
        self.cnt = 0
        self.models_dir = "/u/mariusm/public/Ikea_3D_/"  # TODO: make this a parameter
        self.models = {}




    def load_models(self):
        self.model_names = []
        for f in listDirectory(self.models_dir,'.ply'):
            print "Loading: ",f
            name = os.path.splitext(os.path.split(f)[1])[0]
            obj_spec,obj = ply_import.read(f)
            model_vertex = numpy.array(obj['vertex']) / 1000
            self.models[name] = VoxelGrid(model_vertex,50,50,50)
            self.model_names.append(name)
        print 'Done loading'

    def show_model(self, model, location, model_id=-1):
        
        if model_id==-1:
            model_id = self.cnt
        
        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "table_frame"
        
        marker.ns = "model_fit"
        marker.id = model_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.position.x = location[0]
        marker.pose.position.y = location[1]
        marker.pose.position.z = location[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 0.5
        marker.color.r = self.colors[model_id%len(self.colors)][0]
        marker.color.g = self.colors[model_id%len(self.colors)][1]
        marker.color.b = self.colors[model_id%len(self.colors)][2]
        for p in model:
            marker.points.append(Point(p[0],p[1],p[2]))
        self.maker_pub.publish(marker)
        
        self.cnt += 1

            
    def show_point_cloud(self, cloud):
        
        marker = Marker()
        marker.header.stamp = cloud.header.stamp
        marker.header.frame_id = cloud.header.frame_id
        
        marker.ns = "model_fit"
        marker.id = self.cnt
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 0.5
        marker.color.r = self.colors[self.cnt%len(self.colors)][0]
        marker.color.g = self.colors[self.cnt%len(self.colors)][1]
        marker.color.b = self.colors[self.cnt%len(self.colors)][2]
        for p in cloud.pts:
            marker.points.append(Point(p.x,p.y,p.z))
        self.maker_pub.publish(marker)
        
        self.cnt += 1

        
    def model_fit(self,req):
        print "Service called"
        
        point_cloud = numpy.empty((len(req.cloud.pts),3), dtype=numpy.float32)
        i = 0
        for p in req.cloud.pts:
            point_cloud[i] = (p.x,p.y,p.z)
            i+=1 

        mean = numpy.mean(point_cloud, axis=0)
        mean[2] = 0
                
#        print "Number of points", point_cloud.shape[0]
#        self.show_point_cloud(req.cloud)

        #grid = VoxelGrid(point_cloud,50,50,50)
        # get "center" of point cloud projected onto the plane
        #grid.show_grid(self.maker_pub)
        
        
        best_score = 1e10
        best_position = mean
        best_template = None
        for name in self.model_names:
            grid = self.models[name]        

            for xd in frange(-0.02, 0.02, 0.01): 
                for yd in frange(-0.02, 0.02, 0.01):
                    position = mean + (xd,yd,0)
                    print "Computing score"
                    score = grid.match_score(point_cloud, position )
                    print score  
                    if score<best_score:
                        best_score = score
                        best_position = position
                        best_template = name
        
        print "Best score", best_score
        self.show_model(self.models[best_template].template, best_position)
        
        return ModelFitResponse()

        
if __name__ == '__main__':
    node = ModelFitNode()
    node.load_models()
    rospy.spin()
    
