from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from openrave_utils.funcs import *
import os
from openrave_utils.or_planner import ORPLANNER
import time
# from mpi4py import MPI
import numpy as np
import copy
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
from openrave_utils.shelf_obb import *
from openrave_utils.dynamic_env import *
from openrave_utils.result_notebook import resultNotebook,save_result

try:
    from multiprocessing import cpu_count
except:
    def cpu_count(): return 1


class Tower():
    def __init__(self,grid_index,position,height):
        #the x,y index on checkerboard
        self.grid_index = grid_index
        #position = [x,y,z]
        self.position = position
        #number of boxes constructing a tower
        self.height = height

class TowerEnv():
    def __init__(self,robot,env):
        self.env = env
        self.robot = robot
        #box tower element is class tower with different height 
        self.towers = []
        #random boxes on table class tower with height one
        self.random_boxes = []
        self.boxes_total_num = 5
        self.tower_boxes_num = 3
        self.random_boxes_num = self.boxes_total_num - self.tower_boxes_num

    def create_scenario_v1(self):
        """
            Return:[box1,box2,...],box=[pos + size]
        """
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        self.tower_boxes_num = np.random.choice(4,1)[0]
        self.random_boxes_num = self.boxes_total_num - self.tower_boxes_num        
        # x_limits = [3.3,3.6]
        # y_limits = [-1.9,-0.7]
        # x_pos = np.random.uniform(x_limits[0],x_limits[1])
        # y_pos = np.random.uniform(y_limits[0],y_limits[1])

        x_limits = [0.4,0.8]
        y_limits = [-0.6,0.6] 

        x_index_array = np.random.choice(5,1+self.random_boxes_num)
        y_index_array = np.random.choice(12,1+ self.random_boxes_num)

        x_pos = 0.5 + x_index_array[0] * 0.1 + 0.05
        y_pos = y_index_array[0] * 0.1 + 0.05 

        self.tower_pos_list = []
        self.tower_size_list = []
        side = 0.095
        box_size = [side,side,side]
        # box_pos  = [3.5,-1.3,0.78]
        init_box_pos = [x_pos,y_pos,0.05]
        # self.tower_pos_list.append(box_pos)
        # self.tower_size_list.append(box_size)        
        for i in range(self.tower_boxes_num):        
            box_pos = copy.deepcopy(init_box_pos)
            box_pos[2] += 0.1 * i
            self.tower_pos_list.append(box_pos)
            new_box_size = copy.deepcopy(box_size)
            self.tower_size_list.append(new_box_size)  


        #create boxes on the table
        # boxes_num = 2
        self.random_boxes_pos_list = []
        self.random_boxes_size_list = []

        # x_index_array = np.random.choice(5,self.random_boxes_num)          
        # y_index_array = np.random.choice(12,self.random_boxes_num)
        for i in range(self.random_boxes_num):
            # x_pos = np.random.uniform(x_limits[0],x_limits[1])
            # y_pos = np.random.uniform(y_limits[0],y_limits[1])
            x_pos = 0.5 + x_index_array[i+1] * 0.1 + 0.05
            y_pos = -0.6 + y_index_array[i+1] * 0.1 + 0.05            
            random_box_pos = [x_pos,y_pos,0.05] 
            self.random_boxes_pos_list.append(random_box_pos)
            new_box_size = copy.deepcopy(box_size)
            self.random_boxes_size_list.append(new_box_size)

        self.box_positions = []
        self.box_positions.extend(self.tower_pos_list)
        self.box_positions.extend(self.random_boxes_pos_list)

        self.box_shapes = []
        self.box_shapes.extend(self.tower_size_list)
        self.box_shapes.extend(self.random_boxes_size_list)

        return list(self.box_positions), list(self.box_shapes)                       



    def generate_start_config(self,robot_pos=None):
        #the first random box after tower boxes
        start_box_index = self.tower_boxes_num
        start_pos = self.box_positions[start_box_index]
        #move to 0.05m above box
        # start_pos[2] += 0.05
        arm_start_pos = [start_pos[0]+robot_pos[0],start_pos[1]+robot_pos[1],start_pos[2]+robot_pos[2]]
        start_config = self.ik_goal(arm_start_pos)
        return start_box_index, start_config

    def generate_goal_config(self,robot_pos=None):
        #the top box of the tower
        goal_box_index = self.tower_boxes_num - 1
        #move to 0.15m above tower
        goal_pos = self.box_positions[goal_box_index]
        goal_pos[2] += 0.1
        arm_goal_pos = [goal_pos[0]+robot_pos[0],goal_pos[1]+robot_pos[1],goal_pos[2]+robot_pos[2]]
        goal_config = self.ik_start(arm_goal_pos)
        return goal_config

    def generate_start_config_checkerboard(self,robot_pos=None):
        #self.random_boxes
        start_pos = copy.deepcopy(self.random_boxes[0].position)
        #move to 0.05m above box
        # start_pos[2] += 0.05
        arm_start_pos = [start_pos[0]+robot_pos[0],start_pos[1]+robot_pos[1],start_pos[2]+robot_pos[2]]
        start_config = self.ik_goal(arm_start_pos)

        start_box_index = self.first_random_box_index
        return start_box_index, start_config

    def generate_goal_config_checkerboard(self,robot_pos=None):
        #self.towers
        first_tower = self.towers[0]
        goal_pos = copy.deepcopy(first_tower.position)
        goal_pos[2] += 0.1 * (first_tower.height - 1)

        #move to 0.15m above tower
        goal_pos[2] += 0.1
        arm_goal_pos = [goal_pos[0]+robot_pos[0],goal_pos[1]+robot_pos[1],goal_pos[2]+robot_pos[2]]
        goal_config = self.ik_goal(arm_goal_pos)
        return goal_config


    def ik_start(self,pos):
        goal = eye(4,dtype=float)
        goal[0,3] = pos[0]
        goal[1,3] = pos[1]
        goal[2,3] = pos[2]

        Tz = matrixFromAxisAngle([-np.pi/2,0,0])
        goal_transform = np.dot(goal,Tz)
        # goal[0,0] = 0
        # goal[0,1] = -1
        # goal[1,0] = -1
        # goal[1,1] = 0
        # goal[2,2] = -1
        solution = self.robot.GetActiveManipulator().FindIKSolution(goal_transform,True)

        return solution

    def ik_goal(self,pos):
        goal = eye(4,dtype=float)
        goal[0,3] = pos[0]
        goal[1,3] = pos[1]
        goal[2,3] = pos[2]

        # Tz = matrixFromAxisAngle([-np.pi/2,0,0])
        # goal_transform = np.dot(goal,Tz)
        goal[0,0] = 0
        goal[0,1] = -1
        goal[1,0] = -1
        goal[1,1] = 0
        goal[2,2] = -1

        solution = self.robot.GetActiveManipulator().FindIKSolution(goal,True)
        return solution


    def show_transparent_traj(self):
        traj = [[-2.17436, -1.8944, 1.55, 1.65546, -1.24979, 1.52425, -2.4643], [-1.91637, -1.82585, 1.74978, 1.70763, -0.97835, 1.27638, -2.43277], [-1.65838, -1.75729, 1.94956, 1.7598, -0.70692, 1.02851, -2.40124], [-1.40039, -1.68874, 2.14934, 1.81197, -0.43549, 0.78065, -2.36971], [-1.1424, -1.62018, 2.34913, 1.86414, -0.16406, 0.53278, -2.33817], [-0.88441, -1.55163, 2.54891, 1.9163, 0.10737, 0.28492, -2.30664], [-0.64342, -1.48759, 2.73553, 1.96504, 0.36093, 0.05338, -2.27719], [-0.79923, -1.3095, 2.55398, 1.59689, 0.21966, 0.03288, -2.20552], [-0.95504, -1.1314, 2.37243, 1.22875, 0.07839, 0.01239, -2.13385], [-1.11085, -0.95331, 2.19088, 0.8606, -0.06289, -0.0081, -2.06218], [-1.26666, -0.77522, 2.00933, 0.49245, -0.20416, -0.0286, -1.99051], [-1.42248, -0.59713, 1.82778, 0.12431, -0.34543, -0.04909, -1.91884], [-1.57829, -0.41904, 1.64623, -0.24384, -0.4867, -0.06958, -1.84717], [-1.7341, -0.24095, 1.46468, -0.61198, -0.62797, -0.09008, -1.7755], [-1.79865, -0.16717, 1.38946, -0.7645, -0.6865, -0.09857, -1.7458], [-1.83186, 0.01445, 1.46536, -0.68314, -1.05517, -0.26922, -1.54962], [-1.86507, 0.19607, 1.54126, -0.60177, -1.42384, -0.43988, -1.35344], [-1.89827, 0.37768, 1.61715, -0.5204, -1.79252, -0.61054, -1.15725], [-1.93148, 0.5593, 1.69305, -0.43904, -2.16119, -0.7812, -0.96107], [-1.96469, 0.74092, 1.76895, -0.35767, -2.52986, -0.95186, -0.76489], [-1.9979, 0.92254, 1.84484, -0.27631, -2.89853, -1.12251, -0.5687], [-2.01255, 1.00264, 1.87832, -0.24042, -3.06114, -1.19778, -0.48218], [-1.82243, 0.9592, 1.82253, 0.0011, -2.67387, -1.21546, -0.46667], [-1.63231, 0.91577, 1.76675, 0.24262, -2.28661, -1.23315, -0.45115], [-1.44219, 0.87233, 1.71097, 0.48414, -1.89935, -1.25083, -0.43564], [-1.25208, 0.8289, 1.65518, 0.72566, -1.51209, -1.26851, -0.42013], [-1.06196, 0.78546, 1.5994, 0.96718, -1.12483, -1.2862, -0.40462]]

        transparency = 1.0

        waypoints_num = len(traj)
        # print('waypoints_num',waypoints_num)
        delta_transparency = 1.0/waypoints_num
        interpolate_step = 5
        # print('delta_transparency',delta_transparency)
        for i in range(waypoints_num):
            waypoint = np.array(traj[i])

            newrobot = RaveCreateRobot(env,robot.GetXMLId())
            newrobot.Clone(robot,0)
            for link in newrobot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
            env.Add(newrobot,True)
            newrobot.SetTransform(robot.GetTransform())
            newrobot.SetDOFValues(waypoint,manipulator.GetArmIndices())

            # transparency -=delta_transparency
            transparency -= delta_transparency * interpolate_step
            i+=interpolate_step

    def create_barrier(self):
        """
        Return: 4 box barriers pos and size
        """
        barriers_pos_list = []
        barriers_size_list = []

        x_limits = [0.4,0.8]
        y_limits = [-0.6,0.6]

        box1_pos = [x_limits[0]-0.05,0+0.1,0.15]
        box1_size = [0.1,0.8,0.3]
        box2_pos = [x_limits[0]+0.2,y_limits[0]+0.4-0.05,0.15]
        box2_size = [0.4,0.1,0.3]
        box3_pos = [x_limits[0]+0.5-0.05,0+0.1,0.15]
        box3_size = [0.1,0.8,0.3] 
        box4_pos = [x_limits[0]+0.2,y_limits[0]+0.8+0.05+0.2,0.15]
        box4_size = [0.4,0.1,0.3]

        barriers_pos_list.append(box1_pos)
        barriers_pos_list.append(box2_pos)
        barriers_pos_list.append(box3_pos)
        barriers_pos_list.append(box4_pos)

        barriers_size_list.append(box1_size)
        barriers_size_list.append(box2_size)
        barriers_size_list.append(box3_size)
        barriers_size_list.append(box4_size)

        return barriers_pos_list,barriers_size_list

    def create_scenario(self):
        """
            Return:[box1,box2,...],box=[pos + size]
        """
        self.center_board_z = 0.25
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        side = 0.095
        box_size = [side,side,side]                

        x_limits = [0.4,0.8]
        y_limits = [-0.6,0.6]

        self.tower_pos_list = []
        self.tower_size_list = []


        #4*4 space remained for start random box
        tower_box_index_array = np.random.choice(12,1,replace=False)   
        x_pos_index = int(tower_box_index_array[0]/3)
        y_pos_index = tower_box_index_array[0] % 3
        x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
        y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05
        init_box_pos = [x_pos,y_pos,0.05]
       
        for i in range(self.tower_boxes_num):        
            box_pos = copy.deepcopy(init_box_pos)
            box_pos[2] += 0.1 * i
            self.tower_pos_list.append(box_pos)
            new_box_size = copy.deepcopy(box_size)
            self.tower_size_list.append(new_box_size)  


        #create boxes on the table
        # boxes_num = 2
        self.random_boxes_pos_list = []
        self.random_boxes_size_list = []

        #4*4 space remained for start random box
        random_box_x_limits = [-0.5,0.5]
        random_box_y_limits = [0.6,0.9]        
        random_box_index_array = np.random.choice(30,self.random_boxes_num,replace=False)

        for i in range(self.random_boxes_num):
            x_pos_index = int(random_box_index_array[i]/3)
            y_pos_index = random_box_index_array[i] % 3
            x_pos = random_box_x_limits[0] + x_pos_index * 0.1 + 0.05
            y_pos = random_box_y_limits[0] + y_pos_index * 0.1 + 0.05            
            random_box_pos = [x_pos,y_pos,self.center_board_z+0.1] 
            self.random_boxes_pos_list.append(random_box_pos)
            new_box_size = copy.deepcopy(box_size)
            self.random_boxes_size_list.append(new_box_size)

        self.box_positions = []
        self.box_positions.extend(self.tower_pos_list)
        self.box_positions.extend(self.random_boxes_pos_list)

        self.box_shapes = []
        self.box_shapes.extend(self.tower_size_list)
        self.box_shapes.extend(self.random_boxes_size_list)

        return self.box_positions, self.box_shapes

    def create_scenario_tower_on_shelf(self):
        """
            Return:[box1,box2,...],box=[pos + size]
        """
        self.center_board_z = 0.25
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        side = 0.095
        box_size = [side,side,side]   
        self.tower_boxes_num = np.random.choice(4,1)[0] + 1
        # self.tower_boxes_num = 4
        self.random_boxes_num = self.boxes_total_num - self.tower_boxes_num        

        x_limits = [-0.1,0.1]
        y_limits = [0.6,0.9] 

        self.tower_pos_list = []
        self.tower_size_list = []

        x_grid_num = int((x_limits[1]-x_limits[0])/0.1)  
        y_grid_num = int((y_limits[1]-y_limits[0])/0.1)
        choice_num = x_grid_num * y_grid_num
        tower_box_index_array = np.random.choice(choice_num,1,replace=False)

        x_pos_index = int(tower_box_index_array[0]/y_grid_num)
        y_pos_index = tower_box_index_array[0] % y_grid_num
        x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
        y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05
        init_box_pos = [x_pos,y_pos,self.center_board_z+0.1]
       
        for i in range(self.tower_boxes_num):        
            box_pos = copy.deepcopy(init_box_pos)
            box_pos[2] += 0.1 * i
            self.tower_pos_list.append(box_pos)
            new_box_size = copy.deepcopy(box_size)
            self.tower_size_list.append(new_box_size)  


        #create boxes on the table
        # boxes_num = 2
        self.random_boxes_pos_list = []
        self.random_boxes_size_list = []

        #4*4 space remained for start random box

        #on table
        random_box_x_limits = [0.4,0.8]
        random_box_y_limits = [-0.5,0.5]

        x_grid_num = int((x_limits[1]-x_limits[0])/0.1)  
        y_grid_num = int((y_limits[1]-y_limits[0])/0.1)
        choice_num = x_grid_num * y_grid_num

        random_box_index_array = np.random.choice(choice_num,self.random_boxes_num,replace=False)

        for i in range(self.random_boxes_num):
            x_pos_index = int(random_box_index_array[i]/y_grid_num)
            y_pos_index = random_box_index_array[i] % y_grid_num
            x_pos = random_box_x_limits[0] + x_pos_index * 0.1 + 0.05
            y_pos = random_box_y_limits[0] + y_pos_index * 0.1 + 0.05            
            random_box_pos = [x_pos,y_pos,0.05] 
            self.random_boxes_pos_list.append(random_box_pos)
            new_box_size = copy.deepcopy(box_size)
            self.random_boxes_size_list.append(new_box_size)

        self.box_positions = []
        self.box_positions.extend(self.tower_pos_list)
        self.box_positions.extend(self.random_boxes_pos_list)

        self.box_shapes = []
        self.box_shapes.extend(self.tower_size_list)
        self.box_shapes.extend(self.random_boxes_size_list)

        return self.box_positions, self.box_shapes

    def create_scenario_checkerboard(self):
        """
            Return:[box1,box2,...],box=[pos + size]
        """
        self.center_board_z = 0.25
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        side = 0.095
        box_size = [side,side,side]   
        # self.tower_boxes_num = np.random.choice(4,1)[0] + 1
        self.tower_boxes_num = 10
        # self.tower_boxes_num = 4
       
        self.towers = []
        self.random_boxes = []


        #high towers near the robot
        self.towers_near = []
        x_limits = [0.40,0.50]
        y_limits = [-0.20,0.0] 

        x_grid_num = int(round((x_limits[1]-x_limits[0])/0.1))
        # print('(x_limits[1]-x_limits[0])',(x_limits[1]-x_limits[0])) 
        y_grid_num = int((y_limits[1]-y_limits[0])/0.1)
        choice_num = x_grid_num * y_grid_num
        # print('x_grid_num',x_grid_num)
        # print('y_grid_num',y_grid_num)
        # print('choice_num',choice_num)
        # print('int(self.tower_boxes_num/2)',int(self.tower_boxes_num/2))
        tower_box_index_array = np.random.choice(choice_num,2,replace=False)

        x_pos_index = int(tower_box_index_array[0]/y_grid_num)
        y_pos_index = tower_box_index_array[0] % y_grid_num
        x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
        y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05
        init_box_pos = [x_pos,y_pos,0.05]
       
        for i in range(2): 
            x_pos_index = int(tower_box_index_array[i]/y_grid_num)
            y_pos_index = tower_box_index_array[i] % y_grid_num
            x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
            y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05

            grid_index = [x_pos_index,y_pos_index]
            tower_pos = [x_pos,y_pos,0.05] 
            height = np.random.choice(4,1,replace=False) + 4
            tower = Tower(grid_index,tower_pos,height)
            self.towers_near.append(tower)

        #high towers on the right side of the robot
        self.towers_right = []
        x_limits = [0.50,0.80]
        y_limits = [-0.20,-0.10] 

        x_grid_num = int(round((x_limits[1]-x_limits[0])/0.1))
        # print('(x_limits[1]-x_limits[0])',(x_limits[1]-x_limits[0])) 
        y_grid_num = int((y_limits[1]-y_limits[0])/0.1)
        choice_num = x_grid_num * y_grid_num
        # print('x_grid_num',x_grid_num)
        # print('y_grid_num',y_grid_num)
        # print('choice_num',choice_num)
        # print('int(self.tower_boxes_num/2)',int(self.tower_boxes_num/2))
        tower_box_index_array = np.random.choice(choice_num,3,replace=False)

        x_pos_index = int(tower_box_index_array[0]/y_grid_num)
        y_pos_index = tower_box_index_array[0] % y_grid_num
        x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
        y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05
        init_box_pos = [x_pos,y_pos,0.05]
       
        for i in range(3): 
            x_pos_index = int(tower_box_index_array[i]/y_grid_num)
            y_pos_index = tower_box_index_array[i] % y_grid_num
            x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
            y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05

            grid_index = [x_pos_index,y_pos_index]
            tower_pos = [x_pos,y_pos,0.05] 
            height = np.random.choice(4,1,replace=False) + 4
            tower = Tower(grid_index,tower_pos,height)
            self.towers_right.append(tower)



        #lower towers far from the robot
        self.towers_far = []
        x_limits = [0.6,0.8]
        y_limits = [-0.2,0.2] 

        x_grid_num = int(round((x_limits[1]-x_limits[0])/0.1))  
        y_grid_num = int((y_limits[1]-y_limits[0])/0.1)
        choice_num = x_grid_num * y_grid_num
        tower_box_index_array = np.random.choice(choice_num,int(self.tower_boxes_num/2),replace=False)

        x_pos_index = int(tower_box_index_array[0]/y_grid_num)
        y_pos_index = tower_box_index_array[0] % y_grid_num
        x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
        y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05
        init_box_pos = [x_pos,y_pos,0.05]
       
        for i in range(self.tower_boxes_num/2): 
            x_pos_index = int(tower_box_index_array[i]/y_grid_num)
            y_pos_index = tower_box_index_array[i] % y_grid_num
            x_pos = x_limits[0] + x_pos_index * 0.1 + 0.05
            y_pos = y_limits[0] + y_pos_index * 0.1 + 0.05

            grid_index = [x_pos_index,y_pos_index]
            tower_pos = [x_pos,y_pos,0.05] 
            height = np.random.choice(2,1,replace=False) + 1
            tower = Tower(grid_index,tower_pos,height)
            self.towers_far.append(tower)

        #merge self.towers_far self.towers_near self.towers_right
        self.towers_far.extend(self.towers_near)
        self.towers_far.extend(self.towers_right)
        self.towers = copy.deepcopy(self.towers_far)



        #create boxes on the table
        # boxes_num = 2
        self.random_boxes_pos_list = []
        self.random_boxes_size_list = []

        #4*4 space remained for start random box

        #on table

        self.random_boxes_num = np.random.choice(4,1)[0] + 1      
        random_box_x_limits = [0.4,0.8]
        random_box_y_limits = [-0.6,-0.2]

        x_grid_num = int((x_limits[1]-x_limits[0])/0.1)  
        y_grid_num = int((y_limits[1]-y_limits[0])/0.1)
        choice_num = x_grid_num * y_grid_num

        random_box_index_array = np.random.choice(choice_num,self.random_boxes_num,replace=False)

        for i in range(self.random_boxes_num):
            x_pos_index = int(random_box_index_array[i]/y_grid_num)
            y_pos_index = random_box_index_array[i] % y_grid_num
            x_pos = random_box_x_limits[0] + x_pos_index * 0.1 + 0.05
            y_pos = random_box_y_limits[0] + y_pos_index * 0.1 + 0.05

            grid_index = [x_pos_index,y_pos_index]
            random_box_pos = [x_pos,y_pos,0.05]
            height = 1
            random_box = Tower(grid_index,random_box_pos,height)
            self.random_boxes.append(random_box)

        #got self.towers and self.random_boxes including all the towers, generate box list
        self.box_positions = []
        self.box_shapes = []


        for box in self.random_boxes:
            box_pos = box.position
            pos = copy.deepcopy(box_pos)
            new_box_size = copy.deepcopy(box_size)
            self.box_positions.append(pos)
            self.box_shapes.append(new_box_size)

        self.first_random_box_index = 0#len(self.box_positions)
        
        for tower in self.towers:
            base_pos = tower.position
            for i in range(tower.height):
                pos = copy.deepcopy(base_pos)
                new_box_size = copy.deepcopy(box_size)
                pos[2] += i * 0.1
                self.box_positions.append(pos)
                self.box_shapes.append(new_box_size)


            
        # self.box_positions.extend(self.tower_pos_list)
        # self.box_positions.extend(self.random_boxes_pos_list)


        # self.box_shapes.extend(self.tower_size_list)
        # self.box_shapes.extend(self.random_boxes_size_list)

        return self.box_positions, self.box_shapes



    def create_shelf(self,robot_pos=None):
        shapes = []
        positions = []
        horizonal_shape = [0.6,0.6,0.1]
        vertical_shape = [0.1,0.6,2.0]
        h_pos1 = [0.00,0.7,self.center_board_z]
        h_pos2 = [0.00,0.7,0.95]
        h_pos3 = [0.00,0.7,-0.75]
        v_pos1 = [-0.35,0.7,0.0]
        v_pos2 = [0.35,0.7,0.0]
        shapes = [horizonal_shape,horizonal_shape,horizonal_shape,vertical_shape,vertical_shape]
        positions = [h_pos1,h_pos2,h_pos3,v_pos1,v_pos2]
        # shapes.append([0.8,0.1,2.0])
        # positions.append([0.7,0.2,0.0])
        return positions,shapes

    def create_board(self,robot_pos=None):
        shapes = []
        positions = []
        vertical_shape = [0.6,0.1,0.9]
        v_pos1 = [0.7,-0.75,0.45]
        v_pos2 = [0.7,0.45,0.45]
        horizonal_shape = [0.6,1.2,0.1]
        h_pos1 = [0.7,-0.2,0.85]
        shapes = [vertical_shape,vertical_shape,horizonal_shape]
        positions = [v_pos1,v_pos2,h_pos1]
        # shapes.append([0.8,0.1,2.0])
        # positions.append([0.7,0.2,0.0])
        return positions,shapes

    def navigation(self,pos):
        #pos = [x,y,direction]
        self.basemanip = interfaces.BaseManipulation(self.robot)
        # find the boundaries of the environment
        with self.env:
            envmin = []
            envmax = []
            for b in self.env.GetBodies():
                ab = b.ComputeAABB()
                envmin.append(ab.pos()-ab.extents())
                envmax.append(ab.pos()+ab.extents())
            abrobot = self.robot.ComputeAABB()
            envmin = numpy.min(array(envmin),0)+abrobot.extents()
            envmax = numpy.max(array(envmax),0)-abrobot.extents()
        bounds = array(((envmin[0],envmin[1],-pi),(envmax[0],envmax[1],pi)))
        with self.env:
            self.robot.SetAffineTranslationLimits(envmin,envmax)
            self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
            self.robot.SetAffineRotationAxisMaxVels(ones(4))
            self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
            # pick a random position
            # with self.robot:
            #     while True:
            #         # goal = bounds[0,:]+np.random.rand(3)*(bounds[1,:]-bounds[0,:])
            #         goal = np.array([2.6,-1.3,3.14])
            #         self.robot.SetActiveDOFValues(goal)
            #         if not self.env.CheckCollision(self.robot):
                        # break

        # goal = np.array([2.3,-1.2,np.pi*0.5])  
        goal = pos                          
        print 'planning to: ',goal  
        self.basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1)
        # if self.basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1) is None:
        #     print 'retrying...'
        #     continue
        print 'waiting for controller'
        self.robot.WaitForController(0) 
        print('the goal is',goal)
        # raw_input('another goal?') 

def run():
    env = Environment()
    env.SetViewer('qtcoin')
    # robot_pos = [2.6, -1.3, 1.0]
    robot_pos = [0,0,0]
    env.Load(os.getcwd()+'/worlds/exp5.env.xml')

    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)
    tower_scenario = TowerEnv(robot)


    scenario_pos_list, scenario_size_list = tower_scenario.create_scenario()
    # print('scenario_pos_list',scenario_pos_list)
    boxes_body_list,_ = create_boxes(env,scenario_pos_list,scenario_size_list,robot_pos)
    volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
    for body_id,body in enumerate(boxes_body_list):
        for ig,g in enumerate(body.GetLinks()[0].GetGeometries()):
            g.SetDiffuseColor(volumecolors[body_id])


    raw_input()


    trials_num = 10
    for i in range(trials_num):
        scenario_pos_list, scenario_size_list = tower_scenario.create_scenario()
        _ = Recreate_boxes(env,scenario_pos_list,scenario_size_list,boxes_body_list,robot_pos)
        
        for body_id,body in enumerate(boxes_body_list):
            for ig,g in enumerate(body.GetLinks()[0].GetGeometries()):
                g.SetDiffuseColor(volumecolors[body_id])


        start_config = tower_scenario.generate_start_config()
        goal_config = tower_scenario.generate_goal_config()
        if(start_config is None):
            print('no start config')
            continue
        if(goal_config is None):
            print('no goal config')
            continue


        print('Find both start and goal config')
        execute_activeDOFValues(solution=start_config,robot=robot,env=env)


        raw_input()




        time_limit = 20.0
        motion_range = 20.0
        sample_per_batch = 100.0
        orplanner = ORPLANNER(robot=robot,env=env,planner_name = "OMPL_BITstar",
            time_limit = time_limit, motion_range = motion_range,samples_per_batch=sample_per_batch)
        #generate trajectory
        orplanner.set_goal(goal_config)
        # start_time = time.time()
        traj,interpolated_traj = orplanner.plan_traj()
        # end_time = time.time()
        if(traj is None):
            print("no trajectory solution")
            continue

        raw_input()


        taskmanip = interfaces.TaskManipulation(robot)
        taskmanip.CloseFingers()
        robot.WaitForController(0)        
        robot.Grab(boxes_body_list[3])

        robot.GetController().SetPath(traj)
        robot.WaitForController(0)
        time.sleep(0.5)






    while(True):
        pass




if __name__ == "__main__":
    run()