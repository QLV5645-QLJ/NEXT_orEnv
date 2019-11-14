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
from mpi4py import MPI
import numpy as np

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
from openrave_utils.shelf_obb import *
from openrave_utils.dynamic_env import *
from openrave_utils.result_notebook import resultNotebook,save_result
from openrave_utils.tower_env import TowerEnv
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--time_limit', type=float, default=5)
args = parser.parse_args()


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def run():
    env = Environment()
    # env.SetViewer('qtcoin')
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    mpi_size = comm.Get_size()

    print("rank:",rank,"size: ",mpi_size)

    "Main example code."
    robot_pos = [2.6, -1.3, 0.8]#the origin of the world but not actual robot pos
    env.Load(os.getcwd()+'/worlds/exp5.env.xml')

    #get robot info
    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)
    taskmanip = interfaces.TaskManipulation(robot)

    #load scene
    tower_env = TowerEnv(robot=robot,env=env)
    box_positions,box_shapes = tower_env.create_scenario_checkerboard()
    # board_positions,board_shapes = tower_env.create_shelf()
    positions,shapes = box_positions,box_shapes
    body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,robot_pos=robot_pos)
    time.sleep(0.1)

    #get task from dataset
    init_states = None
    goal_states = None
    obs_list,init_array,goal_array = read_task_withObs("dataset/tasks_continuous_stacking.txt")
    init_states = np.copy(init_array)
    goal_states = np.copy(goal_array)

    #use palner to plan path
    time_limit = args.time_limit#5.0
    motion_range = 20.0
    sample_per_batch = 100.0
    planner_name = "OMPL_BITstar"
    orplanner = ORPLANNER(robot=robot,env=env,planner_name = planner_name,
        time_limit = time_limit, motion_range = motion_range,samples_per_batch=sample_per_batch)
    goal_num = 400
    success_num = 0
    inside = True
    delta_time = 0.0

    # raw_input("...")
    res_notebook = resultNotebook()
    for i in range(goal_num):
        print("time_limit: %f motion_range:%f sample_per_batch:%f "%(time_limit,motion_range,sample_per_batch))
        print("path num:",i)
        print("sucess num:",success_num)
        print("total plan time:",delta_time)

        state_id = i #+ rank*goal_num
        #load scene
        obs_aabbs = list(obs_list[state_id])
        shapes,positions = transform_obb(obs_aabbs[0:-1])
        for body in body_list:
            env.Remove(body)
        body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,
            robot_pos=robot_pos,draw=True,no_color=3,smaller_box=True)
        time.sleep(0.1)

        #move to outside and then generate goal
        start_config = None
        start_config = np.copy(init_states[state_id,:])
        execute_activeDOFValues(solution=start_config,robot=robot,env=env)      
        time.sleep(0.1)
        if(env.CheckCollision(robot)):
            print("!!!start colliion!!!")
        goal_config = None
        goal_config = np.copy(goal_states[state_id,:])
        if(goal_config is None):
            print("No goal IK solution")
            continue

        #generate trajectory
        orplanner.set_goal(goal_config)
        start_time = time.time()
        traj,interpolated_traj = orplanner.plan_traj()
        end_time = time.time()
        if(traj is None):
            res_notebook.append(success=False,init_state=start_config,goal_state=goal_config)
            print("no trajectory solution")
            continue
        else:
            print("saving data")
            res_notebook.append(success=True,init_state=start_config,goal_state=goal_config,
                path_array=np.array(interpolated_traj))

        #grab object
        # taskmanip.CloseFingers()
        # time.sleep(2)
        # robot.Grab(body_list[0])
        # Execute the trajectory.
        robot.GetController().SetPath(traj)
        robot.WaitForController(0)
        time.sleep(0.5)
        #release grab
        # taskmanip.ReleaseFingers(target=body_list[0])
        # time.sleep(1)

        success_num+=1
        delta_time += (end_time-start_time)
    save_result(res_notebook,"result_data_stacking_continuous/%s_shelf_%drank_%ds.pkl"%(planner_name,rank,int(time_limit)))



if __name__ == "__main__":
    run()