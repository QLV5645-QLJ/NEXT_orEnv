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
from openrave_utils.shelf_obb import *


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
	shelf_obb = ShelfObb()
	frame_shapes,frame_positions =shelf_obb.generate_frames()
	board_shapes,board_positions = shelf_obb.generate_borad_obb()
	shapes = frame_shapes + board_shapes
	positions = frame_positions + board_positions

    # archs_shapes,archs_positions,archs_start,archs_goal = generator.generate()
	robot_pos = [2.6, -1.3, 1.0]
	env.Load(os.getcwd()+'/worlds/exp4.env.xml')
	body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,robot_pos=robot_pos)
	# print(str(aabb_list.tolist()))
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
	time_limit = 50.0
	motion_range = 20.0
	sample_per_batch = 100.0
	orplanner = ORPLANNER(robot=robot,env=env,planner_name = "OMPL_BITstar",
		time_limit = time_limit, motion_range = motion_range,samples_per_batch=sample_per_batch)
	goal_num = 50000
	success_num = 0
	inside = True
	delta_time = 0.0
	for i in range(goal_num):
		if(success_num>20):
		    break

		print("time_limit: %f motion_range:%f sample_per_batch:%f "%(time_limit,motion_range,sample_per_batch))
		print("path num:",i)
		print("sucess num:",success_num)
		print("total plan time:",delta_time)

		#move to outside and then generate goal
		goal_config = None
		start_config = None
		start_config,start_pos = generate_goalIk_shelf(robot=robot,center_pos=list(robot_pos))
		execute_activeDOFValues(solution=start_config,robot=robot,env=env)      
		print "moving to outside"
		goal_config,end_pos = generate_goalIk_shelf(robot=robot,center_pos=list(robot_pos),
			pos_contraint=True,contrained_pos=list(start_pos))
		# execute_activeDOFValues(solution=goal_config,robot=robot,env=env)      
		if(goal_config is None):
			print("No goal IK solution")
			continue

		raw_input("...")
		continue

		#generate trajectory
		orplanner.set_goal(goal_config)
		start_time = time.time()
		traj,interpolated_traj = orplanner.plan_traj()
		end_time = time.time()
		if(traj is None):
			print("no trajectory solution")
			continue

		# record_trajectory_randomObs(start=start_config,end=goal_config,
		# 	traj=interpolated_traj,fileId=rank,aabb_list = (aabb_list).tolist())
		# Execute the trajectory.
		robot.GetController().SetPath(traj)
		robot.WaitForController(0)
		time.sleep(0.5)

		success_num+=1
		delta_time += (end_time-start_time)


if __name__ == "__main__":
    run()
