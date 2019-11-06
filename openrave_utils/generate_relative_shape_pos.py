import copy
import numpy as np
class Shape_pos_generator:

	def shape_to_pos(self,box_shapes):
		"""
		Args:
			box_shapes: list of box's size

		Return:
			box_positions: list of box's position
		"""
		position_0 = [x/2 for x in box_shapes[0]]

		#the second box's y poistion is related to the length of third box
		position_1 = [x/2 for x in box_shapes[1]]
		position_1[1] += (box_shapes[2][1] - box_shapes[0][1])

		#the third box's z position is related to the height of first box
		position_2 = [x/2 for x in box_shapes[2]]
		position_2[2] += box_shapes[0][2]

		box_positions = copy.deepcopy([position_0,position_1,position_2])
		return box_positions

	def change_depth_X(self,box_shapes, variation):
		"""
		Change the depth of arch, extend all three boxes' x.
		Args:
			box_shapes: list of box's size
			variation: change of the arch size		
		Return:
			box_shapes: list of box's size	
		"""
		for box_shape in box_shapes:
			box_shape[0] += variation
		# return box_shapes


	def change_width_Y(self,box_shapes, variation):
		"""
		Change the width of arch which means extend the top box.
		Args:
			box_shapes: list of box's size
			variation: change of the arch size

		Return:
			box_shapes: list of box's size	
		"""
		box_shapes[2][1] += variation
		# return box_shapes

	def change_height_Z(self,box_shapes, variation):
		"""
		Change the heights of first and second box which are pillars.
		Args:
			box_shapes: list of box's size
			variation: change of the arch size

		Return:
			box_shapes: list of box's size	
		"""
		box_shapes[0][2] += variation
		box_shapes[1][2] += variation
		# return box_shapes

	def change_to_point_one(self,box_shapes):
		new_box_shapes = []
		for box in box_shapes:
			box = [x/10.0 for x in box]
			new_box_shapes.append(box)
		return new_box_shapes

	def change_to_real_world_pos(self,box_positions):
		new_box_positions = []
		for box in box_positions:			
			box[0] += 0.7
			box[1] += -0.2
			box[2] += -0.2
			new_box_positions.append(box)
		return new_box_positions

	def position_variations(self,box_positions,x_pos_variation,y_pos_variation):
		#three boxes [x,y,z]
		box_positions_array = np.asarray(box_positions)
		box_positions_array[:,0] += x_pos_variation
		box_positions_array[:,1] += y_pos_variation
		new_box_positions = box_positions_array.tolist()
		return new_box_positions

	def get_start_goal(self,box_positions):
		centric_start = []
		centric_goal = []
		#the first box's x and z, y - 0.15
		centric_start = [box_positions[0][0],box_positions[0][1]-0.15,box_positions[0][2]]
		#the first box's x and z, the third box's y
		centric_goal = [box_positions[0][0],box_positions[2][1],0.0]
		return centric_start,centric_goal

	def generate(self):
		basic_box_shapes = [[3.0,1.0,6.0],[3.0,1.0,6.0],[3.0,6.0,1.0]]
		basic_box_positions = self.shape_to_pos(basic_box_shapes)

		#the element of archs_shapes is basic_box_shapes like
		archs_shapes = []
		archs_positions = []
		archs_start = []
		archs_goal = []

		variations = [-1.0,0.0,1.0]
		x_pos_variations = [0.0,0.1]
		y_pos_variations = [-0.1,0.0,0.1,0.2]

		for x_variation in variations:
			for y_variation in variations:
				for z_variation in variations:
					for x_pos_variation in x_pos_variations:
						for y_pos_variation in y_pos_variations:

							new_box_shapes = copy.deepcopy(basic_box_shapes)
							self.change_depth_X(new_box_shapes, x_variation)
							self.change_width_Y(new_box_shapes, y_variation)
							self.change_height_Z(new_box_shapes, z_variation)
							# /10
							new_box_shapes = self.change_to_point_one(new_box_shapes)
							archs_shapes.append(new_box_shapes)
							#change from using left-down as origin to robot_base as origin
							new_box_positions = self.shape_to_pos(new_box_shapes)
							new_box_positions = self.change_to_real_world_pos(new_box_positions)
							#add the position variance
							new_box_positions = self.position_variations(new_box_positions,x_pos_variation,y_pos_variation)

							#using real new_box_positions to get the start and goal
							centric_start_pos, centric_goal_pos = self.get_start_goal(new_box_positions)
							archs_start.append(centric_start_pos)
							archs_goal.append(centric_goal_pos)
							archs_positions.append(new_box_positions)

		return archs_shapes,archs_positions,archs_start,archs_goal

if __name__ == "__main__":
	generator = Shape_pos_generator()
	archs_shapes,archs_positions = generator.generate()
	print('len',len(archs_shapes))
	# print('archs_shapes',archs_shapes)
	# print('archs_positions',archs_positions)