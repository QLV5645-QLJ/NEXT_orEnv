from __future__ import with_statement # for python 2.5

import time 
import numpy as np
import os
from openrave_utils.or_planner import ORPLANNER
# from mpi4py import MPI
from openrave_utils.funcs import *

import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments


class ORController:
	"""
	interface for openrave environment
	"""
	def __init__(self):
		"""
		initialize the openrave,load xml file to the openrave environment 
		"""
		self.env = Environment()
		self.env.SetViewer('qtcoin')
		self.env.Load('worlds/exp2.env.xml')
		print "initialize the world"

		#get robot info
		self.robot = self.env.GetRobots()[0]
		self.manipulator = self.robot.GetManipulator('arm')
		self.robot.SetActiveManipulator(self.manipulator)
		self.lower,self.upper = self.robot.GetDOFLimits(self.manipulator.GetArmIndices())
		self.lower,self.upper = np.array(self.lower),np.array(self.upper)

		#get collision checker info
		print("*****current collision checker:",self.env.GetCollisionChecker(),"**********")

		self.outside_pos_center = [3.5,-1.7,1.0]
		self.inside_pos_center = [3.5,-1.2,1.0]

		self.current_initState = None
		self.current_goalState = None

		self.handles = []

	def set_initState(self,preprocess_state=False):
		"""
		get and set initial collision-free state for the planning task
		return: 7-element list[.....]
		"""
		start_config,start_pos = generate_goalIk_exp1(right=True,robot=self.robot,pos=self.outside_pos_center)
		execute_activeDOFValues(solution=start_config,robot=self.robot,env=self.env)
		self.current_initState = np.array(start_config)
		time.sleep(0.1)

		if(preprocess_state):
			self.process_state(start_config)
		return start_config

	def get_goalState(self,preprocess_state=False):
		"""
		get a collision-free goal state for the planning task
		return: 7-element list[.....]
		"""
		goal_config,goal_pos = generate_goalIk_exp1(right=False,robot=self.robot,pos=self.inside_pos_center)
		execute_activeDOFValues(solution=goal_config,robot=self.robot,env=self.env)
		self.current_goalState = np.array(goal_config)
		time.sleep(0.1)

		if(preprocess_state):
			self.process_state(goal_config)
		return goal_config

	def step_path(self,start_state,goal_state,preprocess_state=False):
		"""
		TODO: return True if collision happens
		interpolate the action, step the interpolated action in the environment,
		return: True if no collision happens, False if collision happens
		"""
		# test_config = (self.current_goalState + self.current_initState)/2
		# execute_activeDOFValues(solution=test_config,robot=self.robot,env=self.env)
		if(preprocess_state):
			self.recover_state(start_state)
			self.recover_state(goal_state)
		
		points = []

		# interpolate the action and check collision
		collision_flag = False
		action_norm = np.sqrt(np.sum((start_state-goal_state)**2))
		num_action = int ( action_norm / 0.1 )
		for i in range(num_action):
			step_percent = float(i)/float(num_action)
			step_state = interpolate(from_state=start_state,to_state = goal_state,ratio=step_percent)
			collision_flag = self.check_collision(step_state)
			if(collision_flag):
				print("collision happen!!!!")
				# return True
			points.append(self.current_endPose())
		collision_flag = self.check_collision(goal_state)

		points.append(self.current_endPose())
		self.draw_line(points)

		return collision_flag
	
	def check_collision(self,state=None):
		"""
		check collision given specific configuration
		"""
		# print("collision check state:",state)
		execute_activeDOFValues(solution=state,robot=self.robot,env=self.env)
		time.sleep(0.2)
		check = self.env.CheckCollision(self.robot)
		return check
	
	def recover_state(self,state):
		"""
		This is for process the state by mistake
		"""
		state[1] = state[1] - 0.768
	
	def process_state(self,state):
		"""
		This is for process the state by mistake while generating training data
		"""
		state[1] = state[1] + 0.768
	
	def retime_path(self,start_state,goal_state):	
		"""
		TODO: add collision listener here, add more trajectory here
		"""
		traj = RaveCreateTrajectory(self.env,'')
		traj.Init(self.robot.GetActiveConfigurationSpecification())
		traj.Insert(0,start_state)
		traj.Insert(1,goal_state)

		# interpolate the action and check collision
		state_list = []
		action_norm = np.sqrt(np.sum((start_state-goal_state)**2))
		num_action = int ( action_norm / 0.5)
		for i in range(num_action):
			step_percent = float(i)/float(num_action)
			step_state = interpolate(from_state=start_state,to_state = goal_state,ratio=step_percent)
			state_list.append(step_state)
		state_list.append(goal_state)

		for i in range(num_action+1):
			traj.Insert(i,state_list[i])
		status = planningutils.RetimeActiveDOFTrajectory(traj,self.robot,hastimestamps=False,
			maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')

		num = traj.GetNumWaypoints()
		for i in range(num):
			traj_data = traj.GetWaypoint(i)
			print np.round(np.array(traj_data[0:6]),4) 
		getData_trajectory(traj)

		if(status == PlannerStatus.HasSolution):
			self.robot.GetController().SetPath(traj)
			self.robot.WaitForController(0)
		else:
			print"no retime solution"
		
		time.sleep(5)
	
	def smooth_path(self,start_state,goal_state):			
		traj = RaveCreateTrajectory(self.env,'')
		traj.Init(self.robot.GetActiveConfigurationSpecification())
		traj.Insert(0,start_state)
		traj.Insert(1,goal_state)
		status = planningutils.SmoothActiveDOFTrajectory(traj,self.robot)

		num = traj.GetNumWaypoints()
		for i in range(num):
			traj_data = traj.GetWaypoint(i)
			print np.round(np.array(traj_data[0:6]),4) 

		if(status == PlannerStatus.HasSolution):
			self.robot.GetController().SetPath(traj)
			self.robot.WaitForController(0)
		else:
			print "no smooth solution"
		time.sleep(5)

	def draw_line(self,points):
		# self.handles.append(self.env.drawlinestrip(points=array(((3.5,-0.5,1.0),(3.5,1.5,1.0))),
			# linewidth=3.0,colors=array(((1,0,0),(1,0,0)))))
		color_array = []
		for i in range(len(points)):
			color_array.append([1,0,0])
		self.handles.append(self.env.drawlinestrip(points=array(points),
			linewidth=3.0,colors=array(color_array)))

	def current_endPose(self):
		pose = []
		t = self.robot.GetActiveManipulator().GetTransform()
		pose.append(t[0][3])
		pose.append(t[1][3])
		pose.append(t[2][3])
		return pose

def test_collisionChecker():
	controller = ORController()
	controller.set_initState()
	controller.get_goalState()
	#test collision checker
	collision_state = [ -8.107663e-02,   1.21121e+00,  -3.9115e-17,  -4.06948e-01,  -3.5568e-15,  -8.04912e-01,   8.10787e-02]
	free_state = [ 0.71972398, -0.01671372, -0.71666667,  0.27556126,  0.66127495, -0.08373806,  0.80789855]
	collision_flag = controller.check_collision(state = collision_state)
	print("collision state:",collision_flag)
	collision_flag = controller.check_collision(state = free_state)
	print("collision state:",collision_flag)

def test_stepPath():
	controller = ORController()
	from_state = controller.set_initState()
	to_state = controller.get_goalState()
	print np.round(np.array(from_state),4) 
	print np.round(np.array(to_state),4) 
	controller.step_path(start_state=from_state,goal_state=to_state)
	time.sleep(5)

if __name__ == "__main__":
	"""
	note:
	[-2.61799388, -1.97222205, -2.74016693, -0.87266463, -4.79965544, 
	-1.57079633, -3.00196631,  0.        ,  0.        ,  0.        ,
       -0.01745329]
	[ 2.61799388,  1.97222205,  2.74016693,  3.14159265,  1.30899694,
    1.57079633,  3.00196631,  2.44346095,  2.44346095,  2.44346095,
	3.15904595]
	"""
	controller = ORController()
	generate_tasks(controller,3000)
	read_tasks('dataset/tasks.txt')
