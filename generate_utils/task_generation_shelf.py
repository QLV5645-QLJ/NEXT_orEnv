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
from openrave_utils.dynamic_env import *
from openrave_utils.generate_relative_shape_pos import *
from openrave_utils.grid_map import *
from openrave_utils.shelf_obb import *


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def run():    
    "Main example code."

    # load a scene from ProjectRoom environment XML file
    env = Environment()
    # env.SetViewer('qtcoin')
    shelf_obb = ShelfObb()
    frame_shapes,frame_positions =shelf_obb.generate_frames()
    board_shapes,board_positions = shelf_obb.generate_borad_obb()
    shapes = frame_shapes + board_shapes
    positions = frame_positions + board_positions

    # create environment
    robot_pos = [2.6, -1.3, 1.0]
    env.Load(os.getcwd()+'/worlds/exp4.env.xml')
    body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,robot_pos=robot_pos)
    time.sleep(1)

    #get robot info
    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)

    #find available solution and execute solution 3.5,-1.2,1.0
    start_config,start_pos = generate_goalIk_shelf(robot=robot,center_pos=list(robot_pos))
    if(start_config is None):
        print("initialize robot arm eith failure")
        exit()
    execute_activeDOFValues(solution=start_config,robot=robot,env=env)
    time.sleep(0.1)
    

    #use palner to plan path
    goal_num = 10000
    success_num = 0
    inside = True
    delta_time = 0.0
    for i in range(goal_num):
        if(success_num>=3000):
            break
        #genrate scene
        frame_shapes,frame_positions =shelf_obb.generate_frames()
        board_shapes,board_positions = shelf_obb.generate_borad_obb()
        shapes = frame_shapes + board_shapes
        positions = frame_positions + board_positions
        aabb_list = Recreate_boxes(env=env,pos_list=positions,size_list=shapes,body_list=body_list,
            robot_pos=robot_pos)
        time.sleep(0.1)

        goal_config = None
        start_config = None
        start_config,start_pos = generate_goalIk_shelf(robot=robot,center_pos=list(robot_pos))
        execute_activeDOFValues(solution=start_config,robot=robot,env=env)      
        goal_config,end_pos = generate_goalIk_shelf(robot=robot,center_pos=list(robot_pos),
            pos_contraint=True,contrained_pos=list(start_pos))
        if(goal_config is None):
            print("No goal IK solution")
            continue
        execute_activeDOFValues(solution=goal_config,robot=robot,env=env)      
        time.sleep(0.1)

        write_data_withObs(init_state=start_config,goal_state=goal_config,aabb_list=list(aabb_list))
        success_num +=1
        print success_num
        # record_trajectory_randomObs(start=start_config,end=goal_config,
            # traj=interpolated_traj,fileId=rank,aabb_list = (aabb_list).tolist())
        # Execute the trajectory.

def run1():
    print"test"
if __name__ == "__main__":
    run()