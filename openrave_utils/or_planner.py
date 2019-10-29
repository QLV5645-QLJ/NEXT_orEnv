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
				print self.planner.SendCommand('GetParameters')
				if result != PlannerStatus.HasSolution:
					return None
				# getData_trajectory(traj)


				# Shortcut the path.
				print 'Calling the OMPL_Simplifier for shortcutting.'
				self.simplifier.InitPlan(self.robot, Planner.PlannerParameters())
				result = self.simplifier.PlanPath(traj)
				# assert result == PlannerStatus.HasSolution
				if result != PlannerStatus.HasSolution:
					return None
				getData_trajectory(traj)

				# Time the trajectory.
				print 'Timing trajectory'
				result = planningutils.SmoothActiveDOFTrajectory(traj,self.robot)
				# result = planningutils.RetimeTrajectory(traj)
				# assert result == PlannerStatus.HasSolution
				if result != PlannerStatus.HasSolution:
					return None
				getData_trajectory(traj)
				return traj

