import  numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting

class gridMap:
    def __init__(self,width,resolution=0.1):
        self.width = width
        self.resolution = resolution
        self.mapLength = int(width/resolution)
        self.map = np.zeros((self.mapLength, self.mapLength, self.mapLength))

    def add_obstacle(self,obs_aabb):
        #prevent from aabb format error
        preprocess_aabb(obs_aabb)

        obs_min = np.asarray(obs_aabb[0])  + (self.width/2)
        obs_max = np.asarray(obs_aabb[1]) + (self.width/2)
        obsIndex_min = (obs_min / self.resolution) 
        obsIndex_max = (obs_max/ self.resolution) 
        obsIndex_max =  np.around(obsIndex_max, 0)
        for i in range(obsIndex_max.shape[0]):
            if(obsIndex_max[i]>self.mapLength):
                obsIndex_max[i] = self.mapLength
        obsIndex_min =  np.around(obsIndex_min, 0)
        obsIndex_max =  np.around(obsIndex_max, 0)

        for x in range(int(obsIndex_min[0]),int(obsIndex_max[0])):
            for y in range (int(obsIndex_min[1]),int(obsIndex_max[1])):
                for z in range(int(obsIndex_min[2]),int(obsIndex_max[2])):
                    self.map[x,y,z] = 1

    def get_map(self):
        return  self.map

def visulaize_map(grid_map):
    #convert grid map
    voxels = np.array(grid_map,dtype=bool)
    colors = np.empty(voxels.shape, dtype=object)
    colors[voxels] = 'green'
    # and plot everything
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xticks(range(0,voxels.shape[0],2))
    ax.set_yticks(range(0,voxels.shape[1],2))
    ax.set_zticks(range(0,voxels.shape[2],2))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.voxels(voxels, facecolors=colors, edgecolor='k')

    plt.show()

def preprocess_aabb(aabb):
    aabb_min = list(aabb[0])
    aabb_max = list(aabb[1])
    for i in range(len(aabb_min)):
        if(aabb_min[i]>aabb_max[i]):
            temp = aabb[0][i]
            aabb[0][i] = aabb[1][i]
            aabb[1][i] =temp
    return

def test_dynamicEnv(obs_aabb):
    width = 2.0
    resolution = 0.1
    # obs_aabb = [[[0.7,0.3,-0.2],[1.0,0.4,0.4]],[[0.7,-0.2,-0.2],[1.0,-0.1,0.4]],[[0.7,-0.2,0.4],[1.0,0.4,0.5]],
    # [[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]] #2.6,-1.3,1.0 is the center point of the frame
    map = gridMap(width=width,resolution=resolution)
    for aabb in obs_aabb:
        map.add_obstacle(aabb)
    visulaize_map(map.get_map())
    return   

if __name__ == '__main__':
    width = 2.0
    resolution = 0.1
    obs_aabb = [[[0.7,0.3,-0.2],[1.0,0.4,0.4]],[[0.7,-0.2,-0.2],[1.0,-0.1,0.4]],[[0.7,-0.2,0.4],[1.0,0.4,0.5]],
    [[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]] #2.6,-1.3,1.0 is the center point of the frame
    obs_aabb = [[[0.6, -0.4, -0.2], [1.0, 0.6, -0.3]], [[0.6, 0.5, -1.0], [0.9, 0.6, 1.0]], [[0.6, -0.6, -1.0], [0.9, -0.5, 1.0]], [[0.6, -0.6, 0.9], [0.9, 0.6, 1.0]], [[0.6, -0.6, -1.0], [0.9, 0.6, -0.9]], [[0.6, -0.6, 0.4], [0.9, 0.6, 0.5]], [[0.6, -0.6, -0.4], [0.9, 0.6, -0.3]], [[0.6, 0.1, 0.5], [0.9, 0.2, 1.0]], [[0.6, 0.1, -0.4], [0.9, 0.2, 0.5]], [[0.6, 0.1, -1.0], [0.9, 0.2, -0.3]]]
    obs_aabb = [[[0.6, 0.5, -1.0], [0.9, 0.6, 1.0]], [[0.6, -0.6, -1.0], [0.9, -0.5, 1.0]], [[0.6, -0.6, 0.9], [0.9, 0.6, 1.0]], [[0.6, -0.6, -1.0], [0.9, 0.6, -0.9]], [[0.6, -0.6, 0.3], [0.9, 0.6, 0.4]], [[0.6, -0.6, -0.5], [0.9, 0.6, -0.4]], [[0.6, -0.2, 0.4], [0.9, -0.1, 1.0]], [[0.6, -0.2, -0.5], [0.9, -0.1, 0.4]], [[0.6, 0.2, -1.0], [0.9, 0.3, -0.4]]]
    map = gridMap(width=width,resolution=resolution)
    for aabb in obs_aabb:
        map.add_obstacle(aabb)
    visulaize_map(map.get_map())