#!/usr/bin/env python3

#import yaml
import os, sys
import numpy as np
import trimesh
import matplotlib.pyplot as plt
from mesh_to_sdf import mesh_to_voxels
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time
import csv

class Map():

    def __init__(self, name, world_file, voxel_size,min_clearence_distance):
        
        self.world_file = world_file
        self.name = name
        self.voxel_size = voxel_size
        self.min_clearence_distance = min_clearence_distance
        
        print('initializing map',name,'using world', self.world_file)
        
        print("loading mesh from",self.world_file,"...")
        
        self.scene = trimesh.load(self.world_file, force='scene',split_object=True)
        print("mesh loaded type",type(self.scene))
        """
        self.mesh = trimesh.load(self.world_file, force='mesh')
        print("mesh loaded type",type(self.mesh))
        """
        
        self.centroid = self.scene.centroid
        self.extents = self.scene.extents
        print("mesh centroid",self.centroid)
        print("mesh extents",self.extents)

        mesh = trimesh.util.concatenate(
                tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                    for g in self.scene.geometry.values()))
        """
        self.centroid = self.mesh.bounding_box.centroid
        self.extents = self.mesh.bounding_box.extents        
        """
        self.max_extent = np.max(self.extents)
        self.voxel_resolution = int(np.ceil(self.max_extent / self.voxel_size))
        
        print("using voxel_size",self.voxel_size)
        # load numpy voxel array if it was saved to file
        
        self.voxels = None
        loaded = self.load()
        if not loaded:
            print("converting mesh to voxels...")
            #sign_method='depth' from the "laser" scans of the objects 
            start = time.time()
            print("start time",start)
            self.voxels = mesh_to_voxels(mesh, self.voxel_resolution,surface_point_method = 'sample',sign_method='normal')
            end = time.time()
            print("voxels created")
            print("end time",end)
            print("tim it took",end - start)
            self.voxels /= 2.0 / self.max_extent
            self.save()

          
        print("len(voxels)", len(self.voxels))
        print("len(voxels[0])", len(self.voxels[0]))
        print("self.extents", self.extents)
        print("self.centroid", self.centroid)
        min_ext = self.centroid - self.extents / 2.0
        max_ext = self.centroid + self.extents / 2.0
        print("max_ext", max_ext)
        print("min_ext", min_ext)
        # self.voxel_resolution = 50
        print("self.voxel_resolution", self.voxel_resolution)
        min_ext = self.centroid - np.ones(self.centroid.shape) * np.max(self.extents) / 2.0
        max_ext = self.centroid + np.ones(self.centroid.shape) * np.max(self.extents) / 2.0
        
        xgrid = np.linspace(min_ext[0], max_ext[0], self.voxel_resolution)
        ygrid = np.linspace(min_ext[1], max_ext[1], self.voxel_resolution)
        zgrid = np.linspace(min_ext[2], max_ext[2], self.voxel_resolution)
         
        
        print("voxels_array.shape ", self.voxels.shape)
        
        
    def load(self):
        numpy_voxel_file = self.world_file + ".npy"
        self.voxels = None
        if os.path.isfile(numpy_voxel_file):
            print("loading distance voxels from the file", numpy_voxel_file)
            with open(numpy_voxel_file, 'rb') as f:
                loaded_voxels = np.load(f)
                loaded_centroid = np.load(f)
                loaded_extents = np.load(f)
                loaded_voxel_resolution = np.load(f)
                # test if loaded are same
                print("loaded_extents",loaded_extents)
                print("loaded_voxel_resolution",loaded_voxel_resolution)
                print("loaded_centroid",loaded_centroid)
                if (loaded_centroid == self.centroid).all() and (loaded_extents == self.extents).all() and \
                    loaded_voxel_resolution == self.voxel_resolution:
                    self.voxels = loaded_voxels
                    print("voxels loaded from file")
                    return True
                else:
                    print("can not use loaded voxels")
                    return False
                
    def save(self):
        numpy_voxel_file = self.world_file + ".npy"        
        with open(numpy_voxel_file, 'wb') as f:
            np.save(f, self.voxels)
            np.save(f, self.centroid)
            np.save(f, self.extents)
            np.save(f, self.voxel_resolution)
            print("saved voxel file to",numpy_voxel_file)
            return True
    
    def get_voxel_index(self,pos,centroid,extents,resolution,round_indexes = True):
        pos_in_box = pos - centroid
        pos_in_box *= 2 / np.max(extents) #now it is -1 to 1
        pos_in_box += np.array([1.0,1.0,1.0]) #now it is 0 to 2
        pos_in_box *= resolution/2.0 #now 0 to resolution
        
        i = pos_in_box[0]
        j = pos_in_box[1]
        k = pos_in_box[2]
        if round_indexes:
            i = round(pos_in_box[0])
            j = round(pos_in_box[1])
            k = round(pos_in_box[2])
        
        if i < 0 or i >= resolution:
            raise IndexError("i index out of range %d"%(i))
        if j < 0 or j >= resolution:
            raise IndexError("j index out of range %d"%(j))
        if k < 0 or k >= resolution:
            raise IndexError("k index out of range %d"%(k))
        return np.array([i , j , k])
         
    def get_real_pos_from_index(self, pos_ijk):
        dp = self.max_extent / float(self.voxel_resolution - 1)
        min_ext = self.centroid - np.ones(self.centroid.shape) * self.max_extent / 2.0
        pos = min_ext + pos_ijk * dp
        return pos

    def max_distance(self):
        return self.max_extent
    
    def min_distance(self):
        return self.min_clearence_distance
    

def load_csv(file):
    samples = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            col = []
            for c in row:
                #print(c)
                col.append(float(c))
            samples.append(col)
    return samples

if __name__ == "__main__":
    if len(sys.argv)>1:
        print("converting file ",sys.argv[1],"to voxels")
        voxel_size = 0.05
        min_clearence_distance = None
        map = Map("map", sys.argv[1], voxel_size,min_clearence_distance)
        
        if len(sys.argv)>2:
            if sys.argv[2] == '3':
                fig = plt.figure(figsize=(10, 4))
                ax = fig.add_subplot(111, projection='3d')
                x = []
                y = []
                z = []
                last_done = 0
                precision = 1
                for i in range(0,map.voxel_resolution,precision):
                    done = 100 * i / float(map.voxel_resolution)
                    if done > last_done + 1.0:
                        print("done:", done, "% with collision points",len(x))
                        last_done = done
                    for j in range(0,map.voxel_resolution,precision):
                        for k in range(0,map.voxel_resolution,precision):
                            if map.voxels[i, j, k]<0.15:
                                pos_x_y_z = map.get_real_pos_from_index(np.array([i, j, k]))
                                if pos_x_y_z[2]>0.2 and pos_x_y_z[2] <5.0:
                                    x.append(pos_x_y_z[0])
                                    y.append(pos_x_y_z[1])
                                    z.append(pos_x_y_z[2])
                print("collision points obtained")
                ax.scatter(x, y,z )

                ax.set_xlabel('X axis')
                ax.set_ylabel('Y axis')
                ax.set_zlabel('Z axis')

                plt.show()
            else:
                pos = [0,0,1.0]
                index = map.get_voxel_index(pos,map.centroid,map.extents,map.voxel_resolution)
                print("index",index)
                print("map.voxel_resolution",map.voxel_resolution)
                z_index = index[2]
                heatmap = np.zeros((map.voxel_resolution,map.voxel_resolution))
                for i in range(0,map.voxel_resolution):
                    for j in range(0,map.voxel_resolution):
                        heatmap[i,j] = map.voxels[i,j,int(z_index)]
                        # if(heatmap[i,j]>0.15):
                        #     heatmap[i,j] = 0.15
                        if(heatmap[i,j]<-0.0):
                            heatmap[i,j] = -0.0

                fig1 = plt.figure(1,figsize=(10, 4))
                sdfplot1=plt.imshow(heatmap)
                fig1.colorbar(sdfplot1, ax=plt.gca())
                plt.show()
