from openravepy import *
import numpy, time
import numpy as np
import copy 
import math
def create_box(env,OBB,box_id):
	import numpy as np
	"""
	env: openrave env
	AABB: aabb format pf the box,[[x_min,y_min,z_min,x_max,y_max,z_max]]
	box_id: for creating box name
	return: body instance
	"""
	box_obb = np.array(OBB)
	body = RaveCreateKinBody(env,'')
	body.SetName('nextbox_%d'%(box_id))
	body.InitFromBoxes(box_obb,True) # set geometry as one box of extents 0.1, 0.2, 0.3
	env.AddKinBody(body)

	return body

def reCreate_box(env,body,obb):
	box_obb = np.array(obb)
	with env:
		env.Remove(body)
		body.InitFromBoxes(box_obb,True) # set geometry as two boxes
		env.AddKinBody(body)

def transform_relativeAABB(relative_position,size):
	"""
	transform the box size in position to aabb format
	position: [box_x,box_y,box_z] the box position related to robot
	size: [x_range, y_range, z_range]
	return: [[x_min,y_min,z_min],[x_max,y_max,z_max]]
	"""
	world_box_x = relative_position[0]#-original_position[0]
	world_box_y = relative_position[1]#-original_position[1]
	world_box_z = relative_position[2]#-original_position[2]
	x_min,x_max = world_box_x - (size[0]/2.0), world_box_x + (size[0]/2.0)
	y_min,y_max = world_box_y - (size[1]/2.0), world_box_y + (size[1]/2.0)
	z_min,z_max = world_box_z - (size[2]/2.0), world_box_z + (size[2]/2.0)
	return [[x_min,y_min,z_min],[x_max,y_max,z_max]]

def transform_pos(original_position,relative_position):
	world_box_x = relative_position[0]+original_position[0]
	world_box_y = relative_position[1]+original_position[1]
	world_box_z = relative_position[2]+original_position[2]
	return [world_box_x,world_box_y,world_box_z]

def create_exp2(env):
	robot_pos = [2.6, -1.3, 1.0]#2.6,-1.3,1.0
	box_size = [[0.3,0.1,0.6],[0.3,0.1,0.6],[0.3,0.6,0.1]]
	box_relative_pos = [[0.85,0.35,0.1],[0.85,-0.15,0.1],[0.85,0.1,0.45]]
	# box_size = [[0.3,0.1,0.6],[0.3,0.1,0.6],[0.3,0.6,0.1]]
	# box_relative_pos = [[0.85,0.35,0.1],[0.85,-0.15,0.1],[0.85,0.1,0.45]]
	for i in range(3):
		# trandormed_aabb = transform_aabb(original_position=robot_pos,position=box_relative_pos[i],
			# size=box_size[i])
		[world_box_x,world_box_y,world_box_z] = transform_pos(original_position=robot_pos,
			relative_position = box_relative_pos[i])
		[extend_x,extend_y,extend_z] = [box_size[i][0]/2.0,box_size[i][1]/2.0,box_size[i][2]/2.0]
		temp_obb= np.array([[world_box_x,world_box_y,world_box_z,
			extend_x,extend_y,extend_z]])
		create_box(env,temp_obb,i)
		print("creating box",temp_obb)
		# time.sleep(5)

def create_boxes(env,pos_list,size_list,robot_pos):
	"""
	create multiple box on env
	pos_list: [[x,y,z]],relative position to robot position
	size_list: [[rang_x,range_y,range_z]], box size
	robot_pos: original position
	return: body_list:[body1,...] box instance, 
			aabb_list:[[[x_min,y_min,z_max],[x_max,y_max,z_max]],...] 
					  obstacle aabb format
	"""
	body_list = []
	aabb_list = []#[[[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]]#table aabb
	for i in range(len(pos_list)):
		transformed_aabb = transform_relativeAABB(relative_position=pos_list[i],size=size_list[i])
		[world_box_x,world_box_y,world_box_z] = transform_pos(original_position=robot_pos,
			relative_position = pos_list[i])
		[extend_x,extend_y,extend_z] = [size_list[i][0]/2.0,size_list[i][1]/2.0,size_list[i][2]/2.0]
		temp_obb= np.array([[world_box_x,world_box_y,world_box_z,
			extend_x,extend_y,extend_z]])
		box_body=create_box(env,temp_obb,i)
		body_list.append(box_body)
		aabb_list.append(copy.copy(transformed_aabb))
		# print("creating box",temp_obb)
	aabb_array = np.array(aabb_list)
	aabb_array = np.around(aabb_array,2)
	return body_list,aabb_array

def Recreate_boxes(env,pos_list,size_list,body_list,robot_pos):
	"""
	create multiple box on env
	pos_list: [[x,y,z]],relative position to robot position
	size_list: [[rang_x,range_y,range_z]], box size
	robot_pos: original position
	return: body_list:[body1,...] box instance, 
			aabb_list:[[[x_min,y_min,z_max],[x_max,y_max,z_max]],...] 
					  obstacle aabb format
	"""
	aabb_list = []#[[[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]]#table aabb
	for i in range(len(pos_list)):
		transformed_aabb = transform_relativeAABB(relative_position=pos_list[i],size=size_list[i])
		[world_box_x,world_box_y,world_box_z] = transform_pos(original_position=robot_pos,
			relative_position = pos_list[i])
		[extend_x,extend_y,extend_z] = [size_list[i][0]/2.0,size_list[i][1]/2.0,size_list[i][2]/2.0]
		temp_obb= np.array([[world_box_x,world_box_y,world_box_z,
			extend_x,extend_y,extend_z]])
		reCreate_box(env,body_list[i],temp_obb)
		aabb_list.append(copy.copy(transformed_aabb))
		# print("creating box",temp_obb)
	aabb_array = np.array(aabb_list)
	aabb_array = np.around(aabb_array,2)
	return aabb_array

if __name__ == "__main__":
	env = Environment() # create openrave environment
	env.SetViewer('qtcoin') # attach viewer (optional)

	robot_pos = [2.6, -1.3, 0.8]
	box_size = [[0.3,0.1,0.6],[0.3,0.1,0.6],[0.3,0.6,0.1]]
	box_relative_pos = [[0.85,0.35,0.3],[0.85,-0.15,0.3],[0.85,0.1,0.6]]
	for i in range(3):
		# trandormed_aabb = transform_aabb(original_position=robot_pos,position=box_relative_pos[i],
			# size=box_size[i])
		[world_box_x,world_box_y,world_box_z] = transform_pos(original_position=robot_pos,
			relative_position = box_relative_pos[i])
		[extend_x,extend_y,extend_z] = [box_size[i][0]/2.0,box_size[i][1]/2.0,box_size[i][2]/2.0]
		temp_aabb= np.array([[world_box_x,world_box_y,world_box_z,
			extend_x,extend_y,extend_z]])
		create_box(env,temp_aabb,i)
		time.sleep(5)
	time.sleep(5)