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
from openrave_utils.dynamic_env import *
from openrave_utils.generate_relative_shape_pos import *
from openrave_utils.grid_map import *
import numpy as np

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def run():    
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    print("rank:",rank)

    "Main example code."
    # load a scene from ProjectRoom environment XML file
    env = Environment()
    # env.SetViewer('qtcoin')
    generator = Shape_pos_generator()
    archs_shapes,archs_positions,archs_start,archs_goal = generator.generate()
    robot_pos = [2.6, -1.3, 1.0]
    env_id = rank#rank%215

    env.Load(os.getcwd()+'/worlds/exp3.env.xml')
    # create_exp2(env)
    body_list,aabb_list = create_boxes(env=env,pos_list=archs_positions[env_id],
        size_list=archs_shapes[env_id],robot_pos=robot_pos)
    # test_dynamicEnv(aabb_list)

    #get robot info
    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)

    #find available solution and execute solution 3.5,-1.2,1.0
    outside_pos_center = transform_pos(original_position=robot_pos,relative_position=archs_start[env_id])
    inside_pos_center = transform_pos(original_position=robot_pos,relative_position=archs_goal[env_id])
    start_config,start_pos = generate_goalIk_exp1(right=True,robot=robot,pos=outside_pos_center)
    # start_config = generate_goalIk(right=True,robot=robot)
    if(start_config is None):
        print("initialize robot arm eith failure")
        exit()
    execute_activeDOFValues(solution=start_config,robot=robot,env=env)
    time.sleep(0.1)
    

    #use palner to plan path
    goal_num = 50000
    success_num = 0
    inside = True
    delta_time = 0.0
    for i in range(goal_num):
        #generate scene
        print success_num
        if(success_num>=3000):
            break
        env_id = np.random.randint(216)
        archs_shapes,archs_positions,archs_start,archs_goal = generator.generate()
        box_aabb_list = Recreate_boxes(env=env,pos_list=archs_positions[env_id],size_list=archs_shapes[env_id],
            body_list=body_list,robot_pos=robot_pos)
        table_aabb = [[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]
        # aabb_list = (box_aabb_list).tolist().append(table_aabb)
        # aabb_list = np.array(aabb_list)
        aabb_list = box_aabb_list.tolist()
        aabb_list.append(table_aabb)
        outside_pos_center = transform_pos(original_position=robot_pos,relative_position=archs_start[env_id])
        inside_pos_center = transform_pos(original_position=robot_pos,relative_position=archs_goal[env_id])

        #move to outside and then generate goal
        goal_config = None
        start_config = None
        start_config,start_pos = generate_goalIk_exp1(right=True,robot=robot,pos=outside_pos_center)
        execute_activeDOFValues(solution=start_config,robot=robot,env=env)      
        print "moving to outside"
        time.sleep(0.1)
        goal_config,end_pos = generate_goalIk_exp1(right=True,robot=robot,pos=inside_pos_center)
        execute_activeDOFValues(solution=goal_config,robot=robot,env=env)      
        if(goal_config is None):
            print("No goal IK solution")
            continue

        write_data_withObs(init_state=start_config,goal_state=goal_config,aabb_list=list(aabb_list))
        # record_trajectory_randomObs(start=start_config,end=goal_config,
            # traj=interpolated_traj,fileId=rank,aabb_list = (aabb_list).tolist())
        
        # Execute the trajectory.
        success_num+=1
        time.sleep(0.1)


if __name__ == "__main__":
    run()