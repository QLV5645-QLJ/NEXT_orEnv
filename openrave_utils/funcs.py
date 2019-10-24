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
        stepLength = sqrt(sum((traj_array[i+1,]-traj_array[i,])**2))
        # print("step%d delta:"%i,(traj_array[i+1,]-traj_array[i,]))
        # print("step%d length: %f"%(i,stepLength))
    print("num:",num)
    # print("duration",duration)