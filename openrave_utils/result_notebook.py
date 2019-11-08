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

def load_test(planner_name):
	import pickle
	# planner_name = "OMPL_RRTstar"
	time_limits = [5,10,20,50]
	for time_limit in time_limits:
		task_num = 0
		success_num = 0
		total_path_length = 0.0
		rank_ids = range(time_limit/5)
		for rank in rank_ids:
			filename = "result_data/%s_shelf_%drank_%ds.pkl"%(planner_name,rank,time_limit)
			result = pickle.load(open(filename,"rb"))
			task_num += result.task
			success_num += result.success_count
			for (nb_id,nb) in enumerate(result.notebook):
				if(nb.success):
					num_mpi = time_limit/5
					task_id = rank*(1000/num_mpi)+nb_id
					path_length = get_length(nb.path)
					# print("task id:",task_id,path_length)
					# if(task_id==370):
						# print((nb.path.tolist()))
					total_path_length += path_length
				else:
					penalty = 50.0
					total_path_length += penalty
		success_rate = success_num*1.0/task_num*1.0
		average_path_length= total_path_length/task_num*1.0
		print("planner: %s, time limit = %d, success rate = %f, average path cost = %f"%(
			planner_name,time_limit,success_rate,average_path_length))