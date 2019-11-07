from generate_utils.task_generation_shelf import *
import generate_utils.task_generation_dynamics as task_generation_dynamics
from openrave_utils.funcs import read_result
def print_result():
	planner_name = "BITstar"
	path_num,success_num,total_sucess_time = read_result("result_data/result_OMPL_%s_shelf.txt"%planner_name)
	print("%s: success_rate = %f , average_time = %f"%(planner_name,success_num/path_num,total_sucess_time/success_num))
	planner_name = "RRTstar"
	path_num,success_num,total_sucess_time = read_result("result_data/result_OMPL_%s_shelf.txt"%planner_name)
	print("%s: success_rate = %f , average_time = %f"%(planner_name, success_num/path_num,total_sucess_time/success_num))
	planner_name = "NEXT"
	path_num,success_num,total_sucess_time = read_NEXT_result("result_data/result_OMPL_%s_shelf.txt"%planner_name)
	print("%s: success_rate = %f , average_time = %f"%(planner_name, success_num/path_num,total_sucess_time/success_num))

print_result()