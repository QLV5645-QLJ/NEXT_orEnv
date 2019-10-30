from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from openrave_utils.funcs import *
import os

class ORPLANNER():
	def __init__(self,env,robot,planner_name,time_limit=10.0,motion_range=0.2,samples_per_batch = 10.0):
		self.env = env
		self.robot = robot
		self.planner_name = planner_name#"OMPL_RRTstar"
		self.planner = RaveCreatePlanner(env, planner_name)
		self.simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')
		self.time_limit = time_limit
		self.motion_range = motion_range
		self.samples_per_batch = samples_per_batch

		self.params = Planner.PlannerParameters()

	
	def set_goal(self,goal_config):
		self.params.SetRobotActiveJoints(self.robot)
		if(self.planner_name == "OMPL_RRTstar"):
			self.params.SetExtraParameters("""<time_limit>%s</time_limit><range>%s</range>"""
				%(str(self.time_limit),str(self.motion_range)))
		else:
			# self.params.samples_per_batch = self.samples_per_batch
			self.params.SetExtraParameters("<time_limit>%s</time_limit><samples_per_batch>%s</samples_per_batch>"
				%(str(self.time_limit),str(self.samples_per_batch)))
		self.params.SetGoalConfig(goal_config)
		# self.params.SetGoalConfig(goal_config)
		# self.params.SetExtraParameters('<time_limit>100</time_limit>')#50,0.02
        # param_string = """<time_limit>100</time_limit>
		#  <_postprocessing planner="parabolicsmoother">
		#  <_nmaxiterations>100</_nmaxiterations></_postprocessing>"""

	def plan_traj(self):
	    with self.env:
	        with self.robot:
				# Invoke the planner.
				print 'Calling the %s planner.'%self.planner_name
				traj = RaveCreateTrajectory(self.env, '')
				self.planner.InitPlan(self.robot, self.params)
				result = self.planner.PlanPath(traj)
				# print self.planner.SendCommand('GetParameters')
				if result != PlannerStatus.HasSolution:
					return None,None
				traj_array = getData_trajectory(traj)

				print "calling the iterpolator"
				final_waypoints = self.interpolate_traj(traj_array,traj)
				# Shortcut the path.
				# print 'Calling the OMPL_Simplifier for shortcutting.'
				# self.simplifier.InitPlan(self.robot, Planner.PlannerParameters())
				# result = self.simplifier.PlanPath(traj)
				# # assert result == PlannerStatus.HasSolution
				# if result != PlannerStatus.HasSolution:
				# 	return None
				# getData_trajectory(traj)

				# Time the trajectory.
				print 'Timing trajectory'
				result = planningutils.SmoothActiveDOFTrajectory(traj,self.robot)
				# result = planningutils.RetimeTrajectory(traj)
				# assert result == PlannerStatus.HasSolution
				if result != PlannerStatus.HasSolution:
					return None,None
				getData_trajectory(traj)
				return traj,final_waypoints

	def interpolate_traj(self,raw_traj_array,interpolated_traj):
		"""Return list of arrays

			Args:
				raw_traj_array: list of arrays
			Return:
				interpolated_traj: rave traj instance
		"""		
		# raw_traj_array = 
		final_waypoints = []
		interpolate_step = 0.5
		#interpolate raw trajectory
		for i in range(len(raw_traj_array)-1):
		    # print('waypoint%d'%i)
		    from_state = asarray(raw_traj_array[i])
		    to_state = asarray(raw_traj_array[i+1])
		    interpolated_waypoints = self.interpolate(from_state, to_state, interpolate_step)
		    final_waypoints.extend(interpolated_waypoints)

		#create traj instance for execution
		traj = RaveCreateTrajectory(self.env, '')
		traj.Init(self.robot.GetActiveConfigurationSpecification())
		traj.Insert(0,self.robot.GetActiveDOFValues())
		print('len(final_waypoints)',len(final_waypoints))
		for i in range(len(final_waypoints)):
		    traj.Insert(i+1,final_waypoints[i])

		interpolated_traj = traj
		return final_waypoints



	def interpolate(self, from_state, to_state, interpolate_step):
	    """Return list of arrays

	    	Args:
	    		from_state: DoF dimension array
	    		to_state: DoF dimension array
	    		interpolate_step: step_length between two interpolate points
	    	Return:
	    		interpolated_waypoints: list of arrays including from_state
	    """
	    interpolated_waypoints = [from_state]
	    stepLength = sqrt(sum((to_state-from_state)**2))
	    print('stepLength',stepLength)
	    interpolated_points_num = int(stepLength / interpolate_step) 
	    disp = to_state - from_state
	    for i in range(interpolated_points_num):
	        ratio = (i+1)*interpolate_step / stepLength
	        new_state = from_state + disp * ratio
	        interpolated_waypoints.append(new_state)

	    return interpolated_waypoints
