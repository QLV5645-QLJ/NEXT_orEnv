from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from openrave_utils.funcs import *
import os
from openrave_utils.or_planner import ORPLANNER
import time
# from mpi4py import MPI
import numpy as np

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
from openrave_utils.shelf_obb import *
from openrave_utils.dynamic_env import *
from openrave_utils.result_notebook import resultNotebook,save_result

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def run():
    env = Environment()
    env.SetViewer('qtcoin')
    # comm = MPI.COMM_WORLD
    # rank = comm.Get_rank()
    # print("rank:",rank)

    "Main example code."
    # load a scene from ProjectRoom environment XML file
    shelf_obb = ShelfObb()
    frame_shapes,frame_positions =shelf_obb.generate_frames()
    board_shapes,board_positions = shelf_obb.generate_borad_obb()
    shapes = frame_shapes + board_shapes
    positions = frame_positions + board_positions

    # archs_shapes,archs_positions,archs_start,archs_goal = generator.generate()
    robot_pos = [2.6, -1.3, 1.0]
    env.Load(os.getcwd()+'/worlds/exp4.env.xml')
    body_list,aabb_list = create_boxes(env=env,pos_list=positions,size_list=shapes,
        robot_pos=robot_pos)

    #get robot info
    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)

    traj = []
    waypoint1 = [-2.17436, -1.8944, 1.55, 1.65546, -1.24979, 1.52425, -2.4643]
    waypoint2 = [-1.91637, -1.82585, 1.74978, 1.70763, -0.97835, 1.27638, -2.43277]
    traj.append(waypoint1)
    traj.append(waypoint2)

    transparency = 1.0

    for waypoint in traj:
        np.array(waypoint)

        newrobot = RaveCreateRobot(env,robot.GetXMLId())
        newrobot.Clone(robot,0)
        for link in newrobot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(transparency)



        env.Add(newrobot,True)
        newrobot.SetTransform(robot.GetTransform())
        newrobot.SetDOFValues(waypoint,manipulator.GetArmIndices())

        transparency -=0.3

    time.sleep(20)


if __name__ == "__main__":
    run()