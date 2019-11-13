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
# from openrave_utils.shelf_obb import *
from openrave_utils.tower_env import TowerEnv

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
	env.SetViewer('qtcoin')
	robot_pos = [2.6, -1.3, 0.8]#the origin of the world but not actual robot pos
	env.Load(os.getcwd()+'/worlds/exp5.env.xml')
	# body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,robot_pos=robot_pos)
	# print(str(aabb_list.tolist()))
	time.sleep(1)

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
    

    #use palner to plan path
	time_limit = 50.0
	motion_range = 20.0
	sample_per_batch = 100.0
	orplanner = ORPLANNER(robot=robot,env=env,planner_name = "OMPL_BITstar",
		time_limit = time_limit, motion_range = motion_range,samples_per_batch=sample_per_batch)
	goal_num = 5000000
	success_num = 0
	inside = True
	delta_time = 0.0
	path_num = 0
	while(True):
		if(success_num>300):
		    break
		box_positions,box_shapes = tower_env.create_scenario_checkerboard()
		board_positions,board_shapes = tower_env.create_board()
		positions,shapes = box_positions+board_positions,box_shapes+board_shapes
		for body in body_list:
			env.Remove(body)
		body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,robot_pos=robot_pos,draw=True,no_color=3)
		aabb_list = add_table_AABB(aabb_list)
		time.sleep(0.1)
		#add color

		# raw_input("...")
		#move to outside and then generate goal
		goal_config = None
		start_config = None
		# start_config,start_pos = generate_goalIk_shelf(robot=robot,center_pos=list(robot_pos))
		# print "getting start config"
		start_box_index,start_config = tower_env.generate_start_config_checkerboard(robot_pos=robot_pos)
		if(start_config is None):
			# print("No start IK solution")
			continue
		# print "getting goal config"
		goal_config = tower_env.generate_goal_config_checkerboard(robot_pos=robot_pos)     
		if(goal_config is None):
			# print("No goal IK solution")
			continue
		print("time_limit: %f motion_range:%f sample_per_batch:%f "%(time_limit,motion_range,sample_per_batch))
		print("path num:",path_num)
		print("sucess num:",success_num)
		print("total plan time:",delta_time)
		path_num+=1
		execute_activeDOFValues(solution=start_config,robot=robot,env=env)      

		#generate trajectory
		orplanner.set_goal(goal_config)
		start_time = time.time()
		traj,interpolated_traj = orplanner.plan_traj()
		end_time = time.time()
		if(traj is None):
			print("no trajectory solution")
			continue

		# raw_input("...")

		# record_trajectory_randomObs(start=start_config,end=goal_config,
			# traj=interpolated_traj,fileId=rank,aabb_list = (aabb_list).tolist())
		
		#grab object
		taskmanip.CloseFingers()
		time.sleep(2)
		robot.Grab(body_list[start_box_index])
		# Execute the trajectory.
		robot.GetController().SetPath(traj)
		robot.WaitForController(0)
		time.sleep(0.5)
		#release grab
		taskmanip.ReleaseFingers(target=body_list[start_box_index])
		time.sleep(1)
		success_num+=1
		delta_time += (end_time-start_time)


if __name__ == "__main__":
    run()
