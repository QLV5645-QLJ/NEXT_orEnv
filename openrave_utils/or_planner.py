from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from openrave_utils.funcs import *
import os

class ORPLANNER():
	def __init__(self,env,robot,planner_name):
		self.env = env
		self.robot = robot
		self.planner_name = planner_name#"OMPL_RRTstar"
		self.planner = RaveCreatePlanner(env, planner_name)
		self.simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')

		self.params = Planner.PlannerParameters()

	
	def set_goal(self,goal_config):
		self.params.SetRobotActiveJoints(self.robot)
		# self.params.SetGoalConfig(goal_config)
		self.params.SetExtraParameters('<time_limit>100</time_limit><range>0.02</range>')#50,0.02
        # param_string = """<_postprocessing planner="parabolicsmoother">
        # <_nmaxiterations>100</_nmaxiterations></_postprocessing>"""
		# self.params.SetExtraParameters(param_string)
		# self.params.SetExtraParameters('<range>0.02</range>')
		self.params.SetGoalConfig(goal_config)


	def plan_traj(self):
	    with self.env:
	        with self.robot:
				# Invoke the planner.
				print 'Calling the %s planner.'%self.planner_name
				traj = RaveCreateTrajectory(self.env, '')
				self.planner.InitPlan(self.robot, self.params)
				result = self.planner.PlanPath(traj)
				if result != PlannerStatus.HasSolution:
					return None
				getData_trajectory(traj)

				# Shortcut the path.
				print 'Calling the OMPL_Simplifier for shortcutting.'
				self.simplifier.InitPlan(self.robot, Planner.PlannerParameters())
				result = self.simplifier.PlanPath(traj)
				# assert result == PlannerStatus.HasSolution
				if result != PlannerStatus.HasSolution:
					return None
				# getData_trajectory(traj)

				# Time the trajectory.
				print 'Timing trajectory'
				result = planningutils.RetimeTrajectory(traj)
				# assert result == PlannerStatus.HasSolution
				if result != PlannerStatus.HasSolution:
					return None
				getData_trajectory(traj)
				return traj

