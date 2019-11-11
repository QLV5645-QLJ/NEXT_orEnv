import numpy as np
import pickle

def save_result(notebook, filename):
	f = open(filename, 'wb')
	pickle.dump(notebook, f)
	f.close()

class notebook:
	def __init__(self,success,init_state,goal_state,path_array=None):
		self.success = success
		if(success):
			self.path = np.array(path_array)
		self.init_state = np.array(init_state)
		self.goal_state = np.array(goal_state)


class resultNotebook:
	def __init__(self):
		self.task = 0
		self.success_count = 0
		self.notebook = []
		return

	def append(self,success,init_state,goal_state,path_array=None):
		res = notebook(success=success,init_state=init_state,goal_state=goal_state,
			path_array=path_array)
		self.notebook.append(res)
		if(success):
			self.success_count += 1
		self.task +=1
		return

def get_length(path_array):
    path_length=0.0
    num = path_array.shape[0]
    for i in range(num-1):
        motion_length = np.sqrt(np.sum((path_array[i+1] - path_array[i])**2))
        path_length += motion_length
    return path_length

def load_test(planner_name,dirname="result_data_shelf/"):
	import pickle
	import glob
	# planner_name = "OMPL_RRTstar"
	time_limits = [5,10,15,20,25,30,35,40,45,50]
	for time_limit in time_limits:
		task_num = 0
		success_num = 0
		total_path_length = 0.0
		# rank_ids = range(time_limit/5)
		files = glob.glob(dirname+"%s_shelf_*_%ds.pkl"%(planner_name,time_limit))
		for filename in files:
			# filename = "result_data/%s_shelf_%drank_%ds.pkl"%(planner_name,rank,time_limit)
			# print filename
			result = pickle.load(open(filename,"rb"))
			for (nb_id,nb) in enumerate(result.notebook):
				if(nb.success):
					task_id = task_num + nb_id
					path_length = get_length(nb.path)
					total_path_length += path_length
				else:
					penalty = 50.0
					total_path_length += penalty
			task_num += result.task
			success_num += result.success_count
		success_rate = success_num*1.0/task_num*1.0
		average_path_length= total_path_length/task_num*1.0
		print("planner: %s, time limit = %d, success rate = %f, average path cost = %f"%(
			planner_name,time_limit,success_rate,average_path_length))