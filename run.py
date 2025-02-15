import generate_utils.task_generation_shelf as shelf_task
import generate_utils.task_generation_dynamics as task_generation_dynamics
from openrave_utils.funcs import read_result,collect_files,conver_traj_to_task
from openrave_utils.result_notebook import load_test
import openrave_utils.test_base as base_pr
import openrave_utils.test_grasp as grasp_pr
import generate_utils.task_generation_stacking as stacking_tasks

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


# load_test("OMPL_RRTstar",dirname="result_data_stacking/")
# load_test("OMPL_BITstar",dirname="result_data_stacking/")
# grasp_pr.run()
# base_pr.run()
# collect_files()
stacking_tasks.run()
# conver_traj_to_task()
# shelf_task.run()