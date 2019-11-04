from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
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
        print("step%d length: %f"%(i,stepLength))
    print("trajectory num:",num)
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

def record_trajectory_withobs(start,end,traj,fileId):
    end = list(numpy.around(numpy.array(end),5))
    start = list(numpy.around(numpy.array(start),5))
    traj =  (numpy.around(numpy.array(traj),5)).tolist()
    obs_aabb = [[[0.7,0.3,-0.2],[1.0,0.4,0.4]],[[0.7,-0.2,-0.2],[1.0,-0.1,0.4]],[[0.7,-0.2,0.4],[1.0,0.4,0.5]],[[0.6,-0.4,-0.2],[1.0,0.6,-0.3]]] #2.6,-1.3,1.0 is the center point of the frame
    with open('/clever/dataset/roboArm_4/data_%d.txt'%fileId, 'a') as f:
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

def collect_files():
    fileids = [21,22]
    with open('/clever/dataset/roboArm_3/data_all.txt', 'a+') as wf:
        for id in fileids:
            with open('/clever/dataset/roboArm_3/data_%d.txt'%id, 'r') as rf:
                print(id)
                trajs = rf.read()
                wf.write(trajs)

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


if __name__ == "__main__":
    # record_trajectory_withobs(start=[1.0,1.0],end=[0.31,0.31],traj=[[1.0,1.0],[0.2,0.2]],fileId=3)
    from_state = random.randint(6, size=7)
    to_state = random.randint(6, size=7)
    print(from_state)
    print(to_state)
    print interpolate(from_state,to_state,0.1)