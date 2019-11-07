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

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def run():
    env = Environment()
    # env.SetViewer('qtcoin')
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    print("rank:",rank)

    "Main example code."
    # load a scene from ProjectRoom environment XML file
    shelf_obb = ShelfObb()
    frame_shapes,frame_positions =shelf_obb.generate_frames()
    board_shapes,board_positions = shelf_obb.generate_borad_obb()
    shapes = frame_shapes + board_shapes
    positions = frame_positions + board_positions

    # archs_shapes,archs_positions,archs_start,archs_goal = generator.generate()
    robot_pos = [2.6, -1.3, 1.0]
    env.Load(os.getcwd()+'/worlds/exp4.env.xml')
    body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,
        robot_pos=robot_pos)

    #get robot info
    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)

    #get task from dataset
    init_states = None
    goal_states = None
    obs_list,init_array,goal_array = read_task_withObs("dataset/tasks_shelf.txt")
    init_states = np.copy(init_array)
    goal_states = np.copy(goal_array)

    #use palner to plan path
    time_limit = 50.0
    motion_range = 20.0
    sample_per_batch = 100.0
    planner_name = "OMPL_RRTstar"
    orplanner = ORPLANNER(robot=robot,env=env,planner_name = planner_name,
        time_limit = time_limit, motion_range = motion_range,samples_per_batch=sample_per_batch)
    goal_num = 60
    success_num = 0
    inside = True
    delta_time = 0.0
    for i in range(goal_num):
        print("time_limit: %f motion_range:%f sample_per_batch:%f "%(time_limit,motion_range,sample_per_batch))
        print("path num:",i)
        print("sucess num:",success_num)
        print("total plan time:",delta_time)

        state_id = i + rank*goal_num
        #load scene
        obs_aabbs = list(obs_list[state_id])
        shapes,positions = transform_obb(obs_aabbs)
        aabb_list = Recreate_boxes(env=env,pos_list=positions,size_list=shapes,
            body_list=body_list,robot_pos=robot_pos)

        #move to outside and then generate goal
        start_config = None
        start_config = np.copy(init_states[state_id,:])
        execute_activeDOFValues(solution=start_config,robot=robot,env=env)      
        time.sleep(0.1)
        goal_config = None
        goal_config = np.copy(goal_states[state_id,:])
        if(goal_config is None):
            print("No goal IK solution")
            continue

        # print("rank id",rank)
        # print("start config",start_config)
        # print("goal config",goal_config)

        #generate trajectory
        orplanner.set_goal(goal_config)
        start_time = time.time()
        traj,interpolated_traj = orplanner.plan_traj()
        end_time = time.time()
        if(traj is None):
            print("no trajectory solution")
            continue

        #record trjectory if available
        # record_trajectory_withobs(start=start_config,end=goal_config,
            # traj=interpolated_traj,fileId=rank)

        # Execute the trajectory.
        robot.GetController().SetPath(traj)
        robot.WaitForController(0)
        time.sleep(0.1)

        success_num+=1
        delta_time += (end_time-start_time)
    result_string = "path num: %.2f, sucess num: %.2f total success time: %.4f"%(goal_num,success_num,delta_time)
    with open('result_data/result_%s_shelf.txt'%planner_name,'a+') as f:
        f.write(result_string+"\n")
    f.close()



if __name__ == "__main__":
    run()