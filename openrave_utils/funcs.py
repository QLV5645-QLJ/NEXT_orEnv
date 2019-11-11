from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import numpy as np
def current_dofValues(robot):
    manipulator = robot.GetManipulator('arm')   
    return robot.GetDOFValues(manipulator.GetArmIndices())   

def current_endPose(robot):
    return robot.GetActiveManipulator().GetTransform()

def generate_goalIk(right=True,robot=None):
    """
    generate ik solution of the right of the board
    """
    goal = eye(4,dtype=float)
    board_pos = [3.5,0.0,1.0]
    delta_y = 0
    solution = None
    if(right):
        board_pos[1] = -1.2
        delta_y = 1 
    else:
        board_pos[1] = -1.4
        delta_y = -1 

    for i in range(1000):
        #box position 3.5 -1.3 0.74
        goal[0,3] = board_pos[0] -  random.rand(1)/3#Thand[0,3] + random.rand(1)/10
        goal[1,3] = board_pos[1] + delta_y * random.rand(1)/5#Thand[1,3] + random.rand(1)/10
        goal[2,3] = board_pos[2] + random.rand(1)/3#Thand[2,3] + random.rand(1)/10
        solution = robot.GetActiveManipulator().FindIKSolution(goal,True)
        if(solution is not None):
            break
    # print("goal:",goal[0:3,3])
    return solution

def execute_dofValues(solution,robot,env):
    with env:
        robot.SetDOFValues(solution,robot.GetActiveManipulator().GetArmIndices())
        env.UpdatePublishedBodies()

def execute_activeDOFValues(solution,robot,env):
    manipulator = robot.GetManipulator('arm')   
    with env:
        robot.SetActiveDOFs(manipulator.GetArmIndices())
        robot.SetActiveDOFValues(solution)
        robot.SetActiveManipulator(manipulator)


def getData_trajectory(traj):
    duration = traj.GetDuration()
    num = traj.GetNumWaypoints()
    traj_array = []
    for i in range(num):
        traj_data = traj.GetWaypoint(i)
        # print("traj waypoint%d:"%i,traj_data)
        traj_array.append(list(traj_data))
    traj_array = asarray(traj_array)
    print(traj_array.shape) #6,15
    for i in range(num-1):
        # stepLength = sqrt(sum((traj_array[i+1,]-traj_array[i,])**2))
        stepLength = sum(abs(traj_array[i+1,]-traj_array[i,]))
        # print("step%d delta:"%i,(traj_array[i+1,]-traj_array[i,]))
        # print("step%d length: %f"%(i,stepLength))
    # print("trajectory num:",num)
    return traj_array
    # print("duration",duration)

def generate_goalIk_exp1(right=True,robot=None,pos=None):
    """
    generate ik solution of the right of the board
    """
    goal = eye(4,dtype=float)
    delta_y = 0
    solution = None
    if(right):
        delta_y = 1 
    else:
        delta_y = -1 

    for i in range(1000):
        #box position 3.5 -1.3 0.74
        goal[0,3] = pos[0] + random.uniform(-0.2,0.2)#(-0.3,0.3)#Thand[0,3] + random.rand(1)/10
        goal[1,3] = pos[1] + random.uniform(-0.2,0.2)#Thand[1,3] + random.rand(1)/10
        goal[2,3] = pos[2] + random.uniform(-0.3,0.3)#Thand[2,3] + random.rand(1)/10
        solution = robot.GetActiveManipulator().FindIKSolution(goal,True)
        if(solution is not None):
            # print("goal pose:",goal[0,3],goal[1,3],goal[2,3])
            break
    goal_pos =  [goal[0,3],goal[1,3],goal[2,3]]
    # print("goal:",goal[0:3,3])
    return solution,goal_pos

def generate_goalIk_exp2(right=True,robot=None,pos=None):
    """
    generate ik solution of the right of the board
    """
    goal = eye(4,dtype=float)
    delta_y = 0
    solution = None
    if(right):
        delta_y = 0.6
    else:
        delta_y = -0.6

    for i in range(10000):
        #box position 3.5 -1.3 0.74
        goal[0,3] = pos[0] + random.uniform(-0.1,0.1)#(-0.3,0.3)#Thand[0,3] + random.rand(1)/10
        goal[1,3] = pos[1] + random.uniform(0.0,delta_y)#Thand[1,3] + random.rand(1)/10
        goal[2,3] = pos[2] + random.uniform(-1.0,1.0)#Thand[2,3] + random.rand(1)/10
        solution = robot.GetActiveManipulator().FindIKSolution(goal,True)
        if(solution is not None):
            # print("goal pose:",goal[0,3],goal[1,3],goal[2,3])
            break
    goal_pos =  [goal[0,3],goal[1,3],goal[2,3]]
    # print("goal:",goal[0:3,3])
    return solution,goal_pos

def generate_goalIk_shelf(robot=None,center_pos=None,pos_contraint=False,contrained_pos=None):
    """
    generate ik solution of the right of the board
    """
    goal = eye(4,dtype=float)
    delta_y = 0.6
    delta_x = [0.6,0.9]
    delta_z = 1.0 
    solution = None
    start_pos_array = np.asarray(contrained_pos)

    for i in range(5000):
        #box position 3.5 -1.3 0.74
        goal[0,3] = center_pos[0] + random.uniform(delta_x[0],delta_x[1])#(-0.3,0.3)#Thand[0,3] + random.rand(1)/10
        goal[1,3] = center_pos[1] + random.uniform(-delta_y,delta_y)#Thand[1,3] + random.rand(1)/10
        goal[2,3] = center_pos[2] + random.uniform(-delta_z,delta_z)#Thand[2,3] + random.rand(1)/10
        solution = robot.GetActiveManipulator().FindIKSolution(goal,True)
        if(pos_contraint):
            goal_pos_array = np.array( [goal[0,3],goal[1,3],goal[2,3]])
            if(np.sqrt(np.sum((start_pos_array-goal_pos_array)**2))<0.5):
                continue
        if(solution is not None):
            break
    goal_pos =  [goal[0,3],goal[1,3],goal[2,3]]
    # print("goal:",goal[0:3,3])
    return solution,goal_pos

def record_trajectory_withobs(start,end,traj,fileId):
    end = list(numpy.around(numpy.array(end),5))
    start = list(numpy.around(numpy.array(start),5))
    traj =  (numpy.around(numpy.array(traj),5)).tolist()
    obs_aabb = [[[0.7,0.3,-0.2],[1.0,0.4,0.4]],[[0.7,-0.2,-0.2],[1.0,-0.1,0.4]],[[0.7,-0.2,0.4],[1.0,0.4,0.5]],[[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]] #2.6,-1.3,1.0 is the center point of the frame
    with open('/clever/dataset/roboArm_6/data_%d.txt'%fileId, 'a') as f:
        f.write("obtacles: "+str(obs_aabb)+"\n")
        f.write("start: "+str(start)+"\n")
        f.write("tajectories: "+str(traj)+"\n")
        f.write("end: "+str(end)+"\n")
    f.close()

def record_trajectory_endEffecor(start,end,traj,fileId):
    end = list(numpy.around(numpy.array(end),5))
    start = list(numpy.around(numpy.array(start),5))
    traj =  (numpy.around(numpy.array(traj),5)).tolist()
    obs_aabb = [[[0.7,0.3,-0.2],[1.0,0.4,0.4]],[[0.7,-0.2,-0.2],[1.0,-0.1,0.4]],[[0.7,-0.2,0.4],[1.0,0.4,0.5]],[[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]] #2.6,-1.3,1.0 is the center point of the frame
    with open('/clever/dataset/roboArm_4/data_%d_endeffecotr.txt'%fileId, 'a') as f:
        f.write("obtacles: "+str(obs_aabb)+"\n")
        f.write("start: "+str(start)+"\n")
        f.write("tajectories: "+str(traj)+"\n")
        f.write("end: "+str(end)+"\n")
    f.close()

def record_trajectory_randomObs(start,end,traj,aabb_list,fileId):
    end = list(numpy.around(numpy.array(end),5))
    start = list(numpy.around(numpy.array(start),5))
    traj =  (numpy.around(numpy.array(traj),5)).tolist()
    obs_aabb = list(aabb_list)
    with open('/root/catkin_ws/NEXT_ws/simulation/dataset/tower/data_%d.txt'%fileId, 'a') as f:
        f.write("obtacles: "+str(obs_aabb)+"\n")
        f.write("start: "+str(start)+"\n")
        f.write("tajectories: "+str(traj)+"\n")
        f.write("end: "+str(end)+"\n")
    f.close()


def collect_files():
    from os import path
    fileids = range(300)
    num_data = 0
    with open('/root/catkin_ws/NEXT_ws/simulation/dataset/data_shelf.txt', 'a+') as wf:
        for id in fileids:
            if(path.exists('/root/catkin_ws/NEXT_ws/simulation/dataset/shelf/data_%d.txt'%id)):
                with open('/root/catkin_ws/NEXT_ws/simulation/dataset/shelf/data_%d.txt'%id, 'r') as rf:
                    # trajs = rf.readlines()
                    trajs = rf.read()
                    # print(trajs)
                    wf.write(trajs)
                    # num_data += len(trajs)/4
                    # print(len(trajs)/4)
    # print("num data:",num_data)
    return 

def interpolate(from_state, to_state, ratio):
    """
    if from_state <= to_state: roate in nishizhen
    if from_state > to_state: rotate in shunshizhen
    """
    import numpy as np
    disp = None
    dim_state = from_state.shape[0]
    disp = to_state - from_state
    new_state = from_state + disp * ratio
    return new_state

def write_data(init_state,goal_state):
    import numpy as np
    init = list(np.array(init_state))
    goal = list(np.array(goal_state))
    with open('dataset/tasks.txt', 'a+') as f:
        f.write("start: "+str(init)+"\n")
        f.write("end: "+str(goal)+"\n")
    f.close()
    return

def read_tasks(filename):
    import re
    import numpy as np
    init_array = None
    goal_array = None
    init_list = []
    goal_list = []
    lines = None
    p = r"[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?"
    with open(filename, 'r') as f:
        lines = f.readlines()
    for i in range(len(lines)/2):
        init_nums = [float(s) for s in re.findall(p, lines[2*i])]
        goal_nums = [float(s) for s in re.findall(p, lines[2*i+1])]
        init_list.append(list(init_nums))
        goal_list.append(list(goal_nums))
    init_array = np.array(init_list)
    goal_array = np.array(goal_list)

    print init_array.shape
    print goal_array.shape
    # print init_array
    # print goal_array

    return init_array,goal_array

def generate_tasks(controller,num):
    for i in range(num):
        from_state = controller.set_initState()
        time.sleep(20)
        to_state = controller.get_goalState()
        write_data(from_state,to_state)
    return

def write_data_withObs(init_state,goal_state,aabb_list):
    import numpy as np
    init = (numpy.around(numpy.array(init_state),5)).tolist()
    goal = (numpy.around(numpy.array(goal_state),5)).tolist()
    obs_aabb = (numpy.around(numpy.array(aabb_list),5)).tolist()
    with open('dataset/tasks_test.txt', 'a+') as f:
        f.write("obtacles: "+str(obs_aabb)+"\n")
        f.write("start: "+str(init)+"\n")
        f.write("end: "+str(goal)+"\n")
    f.close()
    return

def read_task_withObs(filename):
    lines = None
    with open(filename, 'r') as f:
        lines = f.readlines()

    num_task = len(lines)/3
    obs_list = []
    init_array = np.zeros(shape=(num_task,7))
    goal_array = np.zeros(shape=(num_task,7))

    for i in range(num_task):
        obs_line = lines[i*3].split(' ')[1:]
        start_line = lines[3*i+1].split(' ')[1:]
        end_line = lines[3*i+2].split(' ')[1:]

        obs_mat = clean_obsLine(obs_line)
        start_mat = clean_line(start_line)
        end_mat = clean_line(end_line)
        obs_aabb = get_aabbs(obs_mat)

        init_array[i]=np.copy(start_mat)
        goal_array[i] = np.copy(end_mat)
        obs_list.append(list(obs_aabb))
    return obs_list,init_array,goal_array

def clean_entry(entry):
    entry = entry.strip()
    while entry[0] == '[':
        entry = entry[1:]
    while entry[-1] == ',':
        entry = entry[:-1]
    while entry[-1] == ']':
        entry = entry[:-1]
    entry = np.float(entry)
    return entry

def clean_obsLine(line):
    for i in range(len(line)):
        line[i] = clean_entry(line[i])
    line = np.array(line).reshape((-1, 3))
    line = np.around(line,2)
    return line

def clean_line(line):
    for i in range(len(line)):
        line[i] = clean_entry(line[i])
    line = np.array(line).reshape((-1, 7))
    return line

def get_aabbs(obs_mat):
    obs_mat = np.around(obs_mat,2)
    num = int(len(obs_mat)/2)
    aabbs = []
    for i in range(num):
        aabb_min = list(obs_mat[2*i])
        aabb_max = list(obs_mat[2*i+1])
        aabbs.append([aabb_min,aabb_max])
    # print("aabbs:",aabbs)
    return  aabbs

def transform_obb(aabb_list):
    shapes = []
    positions = []
    for aabb in aabb_list:
        aabb_min = aabb[0]
        aabb_max = aabb[1]
        (shape_x,shape_y,shape_z) = (aabb_max[0]-aabb_min[0],aabb_max[1]-aabb_min[1],aabb_max[2]-aabb_min[2])
        (pos_x,pos_y,pos_z) = ((aabb_max[0]+aabb_min[0])/2.0,(aabb_max[1]+aabb_min[1])/2.0,
            (aabb_max[2]+aabb_min[2])/2.0)
        shapes.append([shape_x,shape_y,shape_z])
        positions.append([pos_x,pos_y,pos_z])
    return shapes,positions

def read_result(filename):
    import re
    import numpy as np
    path_num = 0.0
    success_num = 0.0
    total_sucess_time = 0.0
    lines = None
    p = r"[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?"
    with open(filename, 'r') as f:
        lines = f.readlines()
    for i in range(len(lines)):
        nums = [float(s) for s in re.findall(p, lines[i])]
        # print nums
        path_num += nums[0]
        success_num +=nums[1]
        total_sucess_time += nums[2]
    return path_num,success_num,total_sucess_time

def read_NEXT_result(filename):
    import re
    import numpy as np
    path_num = 0.0
    success_num = 0.0
    total_sucess_time = 0.0
    success_iters = 0.0
    lines = None
    p = r"[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?"
    with open(filename, 'r') as f:
        lines = f.readlines()
    num_data = len(lines)/2
    for i in range(num_data):
        nums = [float(s) for s in re.findall(p, lines[2*i+1])]
        # print nums
        path_num += nums[0]
        success_num +=(nums[1]*nums[0])
        success_iters += nums[2]
        total_sucess_time += (nums[3]*nums[1]*nums[0])
    # print path_num,success_num,total_sucess_time
    return path_num,success_num,total_sucess_time

def add_table_AABB(aabb_list):
    aabb_list = aabb_list.tolist()#add table pos
    aabb_list.append([[0.5,-0.9,-0.1],[1.0,0.5,0.0]])
    aabb_list = np.array(aabb_list)
    return aabb_list

if __name__ == "__main__":
    obs_list,init_array,goal_array = read_task_withObs("../dataset/tasks_dynamics.txt")
    print obs_list[5]
    print init_array[5]
    print goal_array[5]