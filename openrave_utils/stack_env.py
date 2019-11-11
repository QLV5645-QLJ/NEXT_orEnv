import numpy as np
class StackEnv:
	def __init__(self):
		self.box_positions = []
		self.box_shapes = []
		self.goal_pos = []
		return

	def generate_boxes(self):
		"""
		return: positions [[pos_x,pos_y,pos_z],...]
				shapes [[shape_x,shape_y,shape_z],...]
		"""
		positions = []
		shapes = []

		self.box_positions = list(positions)
		self.box_shapes = list(shapes)
		return positions,shapes

	def generate_startConfig(self):
		"""
		robot need to start over one box. We need start config of robot arm and box id. 
		return: box_id 
				start_config:
		"""
		box_id = 0
		start_config = np.zeros(shape=[1,7])

		return box_id,start_config

	def generate_goalConfig(self):
		"""
		move box to the goal postion in order to do stacking
		return: goal config 
		"""
		goal_config = np.zeros(shape=[1,7])

		return goal_config