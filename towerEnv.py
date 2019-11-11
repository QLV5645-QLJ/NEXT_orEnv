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


class Tower_Env():
    def __init__(self,robot):
        self.robot = robot
        #3 box tower
        self.tower = []
        #random boxes on table
        self.random_boxes = []
        self.boxes_total_num = 5
        self.tower_boxes_num = 3
        self.random_boxes_num = self.boxes_total_num - self.tower_boxes_num

    def create_scenario(self):
        """
            Return:[box1,box2,...],box=[pos + size]
        """
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
                
        # x_limits = [3.3,3.6]
        # y_limits = [-1.9,-0.7]
        # x_pos = np.random.uniform(x_limits[0],x_limits[1])
        # y_pos = np.random.uniform(y_limits[0],y_limits[1])

        x_limits = [0.5,1.0]
        y_limits = [-0.6,0.6]        
        x_pos = 0.5 + np.random.choice(5,1)[0] * 0.1 + 0.05
        y_pos = -0.6 + np.random.choice(12,1)[0] * 0.1 + 0.05 

        self.tower_pos_list = []
        self.tower_size_list = []
        side = 0.095
        box_size = [side,side,side]
        # box_pos  = [3.5,-1.3,0.78]
        init_box_pos = [x_pos,y_pos,0.78]
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

        x_index_array = np.random.choice(5,self.random_boxes_num)          
        y_index_array = np.random.choice(12,self.random_boxes_num)
        for i in range(self.random_boxes_num):
            # x_pos = np.random.uniform(x_limits[0],x_limits[1])
            # y_pos = np.random.uniform(y_limits[0],y_limits[1])
            x_pos = 0.5 + x_index_array[i] * 0.1 + 0.05
            y_pos = -0.6 + y_index_array[i] * 0.1 + 0.05            
            random_box_pos = [x_pos,y_pos,0.78] 
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



    def generate_start_config(self):
        #the first random box after tower boxes
        start_box_index = self.tower_boxes_num
        start_pos = self.box_positions[start_box_index]
        #move to 0.05m above box
        start_pos[2] += 0.05
        start_config = self.ik(start_pos)
        return start_config

    def generate_goal_config(self):
        #the top box of the tower
        goal_box_index = self.tower_boxes_num - 1
        #move to 0.15m above tower
        goal_pos = self.box_positions[goal_box_index]
        goal_pos[2] += 0.15
        goal_config = self.ik(goal_pos)
        return goal_config

    def ik(self,pos):
        goal = eye(4,dtype=float)
        goal[0,3] = pos[0]
        goal[1,3] = pos[1]
        goal[2,3] = pos[2]
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

def run():
    env = Environment()
    env.SetViewer('qtcoin')
    # robot_pos = [2.6, -1.3, 1.0]
    robot_pos = [0,0,0]
    env.Load(os.getcwd()+'/worlds/exp5.env.xml')

    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)
    tower_scenario = Tower_Env(robot)


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