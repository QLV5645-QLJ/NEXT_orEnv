import numpy as np
import random
class ShelfObb():
	def __init__(self):
		self.heighth = 2.0
		self.length = 1.2
		#[0.3,1.2,0.1],[0.3,1.2,0.1],[0.3,0.1,2.0],[0.3,0.1,2.0]
		self.vertical_board_shape = [0.3,0.1,self.heighth]
		self.horizonal_board_shape = [0.3,self.length,0.1]
		self.vertical_board_position = [[0.75,self.length/2.0-0.05,0.00],[0.75,-self.length/2.0+0.05,0.00 ]]
		self.horizonal_board_position = [[0.75,0.00,self.heighth/2.0-0.05 ],[0.75,0.00,-self.heighth/2.0+0.05 ]]

	def generate_frames(self):
		frame_shapes = [list(self.vertical_board_shape),list(self.vertical_board_shape),
			list(self.horizonal_board_shape),list(self.horizonal_board_shape)]
		frame_positions = list(self.vertical_board_position) + list(self.horizonal_board_position)
		# frame_positions.append(list(self.vertical_board_position))
		# frame_positions.append(list(self.horizonal_board_position))
		# print frame_shapes
		# print frame_positions
		return frame_shapes,frame_positions

	def generate_potential_pos(self):
		potential_z_pos1 = [0.35,0.45,0.55]#[0.25,0.45,0.65]#[0.35,0.45,0.55]
		potential_z_pos2 = [-0.35,-0.45,-0.55]#[-0.25,-0.45,-0.65]#[-0.35,-0.45,-0.55]
		potential_y_pos1 = [-0.25,-0.15,-0.05,0.05,0.15,0.25]#[-0.35,-0.15,-0.05,0.05,0.15,0.35]#[-0.25,-0.15,-0.05,0.05,0.15,0.25]
		potential_y_pos2 = [-0.25,-0.15,-0.05,0.05,0.15,0.25]#[-0.35,-0.15,-0.05,0.05,0.15,0.35]#[-0.25,-0.15,-0.05,0.05,0.15,0.25]
		potential_y_pos3 = [-0.25,-0.15,-0.05,0.05,0.15,0.25]#[-0.35,-0.15,-0.05,0.05,0.15,0.35]#[-0.25,-0.15,-0.05,0.05,0.15,0.25]

		z1 = random.choice(potential_z_pos1)
		z2 = random.choice(potential_z_pos2)
		y1 = random.choice(potential_y_pos1)
		y2 = random.choice(potential_y_pos2)
		y3 = random.choice(potential_y_pos3)

		return [z1,z2,y1,y2,y3]

	def generate_borad_obb(self):
		[z1,z2,y1,y2,y3]=self.generate_potential_pos()
		return self.generate_obb(z1,z2,y1,y2,y3)

	def generate_obb(self,z1,z2,y1,y2,y3):
		horizonal_board_pos1 = [0.75,0.00,z1]
		horizonal_board_pos2 = [0.75,0.00,z2]
		vertical_board_pos1 = [0.75,y1,(z1+self.heighth/2.0+0.05)/2.0]
		vertical_board_pos2 = [0.75,y2,(z1+z2)/2.0]
		vertical_board_pos3 = [0.75,y3,(z2-self.heighth/2.0+0.05)/2.0]
		horizonal_board_shape = list(self.horizonal_board_shape)
		vertical_board_shape1 = [0.3,0.1,((self.heighth/2.0)-z1-0.05)]
		vertical_board_shape2 = [0.3,0.1,((z1-z2)+0.1)]
		vertical_board_shape3 = [0.3,0.1,((z2+(self.heighth/2.0)+0.05))]
		shapes = [list(horizonal_board_shape),list(horizonal_board_shape),list(vertical_board_shape1),
				list(vertical_board_shape2),list(vertical_board_shape3) ]
		positions = [list(horizonal_board_pos1),list(horizonal_board_pos2),list(vertical_board_pos1),
				list(vertical_board_pos2),list(vertical_board_pos3)]
		# print shapes
		# print positions
		return shapes,positions

# if __name__ == "__main__":
# 	test = ShelfObb()
# 	frame_shapes,frame_positions =test.generate_frames()
# 	board_shapes.board_positions = test.generate_borad_obb()
# 	shapes = frame_shapes + board_shapes
# 	positions = frame_positions + board_positions
	
# 	env = Environment()
# 	env.SetViewer('qtcoin')
#     env.Load(os.getcwd()+'/../worlds/exp4.env.xml')	
#     body_list,aabb_list = create_boxes(env=env,pos_list=poses,size_list=shapes,robot_pos=robot_pos)