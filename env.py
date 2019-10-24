from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from openrave_utils.funcs import *
import os
from openrave_utils.or_planner import ORPLANNER

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def main(env,options):
    "Main example code."
    # load a scene from ProjectRoom environment XML file
    env.Load(os.getcwd()+'/worlds/exp0.env.xml')
    print(env.GetCollisionChecker())
    time.sleep(1)

    #get robot info
    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)

    #find available solution and execute solution
    start_config = generate_goalIk(right=True,robot=robot)
    execute_activeDOFValues(solution=start_config,robot=robot,env=env)   
    time.sleep(0.1)
    
    #find available solution
    goal_config = generate_goalIk(right=False,robot=robot)

    #use palner to plan path
    orplanner = ORPLANNER(robot=robot,env=env,planner_name = "OMPL_RRTstar")

    goal_num = 100
    success_num = 0
    for i in range(goal_num):
        goal_config = generate_goalIk(right=bool(i%2),robot=robot)
        orplanner.set_goal(goal_config)
        traj = orplanner.plan_traj()

        if(traj is None):
            print("no solution")
            continue

        # Execute the trajectory.
        robot.GetController().SetPath(traj)
        robot.WaitForController(0)
        time.sleep(2)
        success_num+=1
    print("path num:",goal_num)
    print("sucess time:".success_num)
    print("success rate:",success_num/goal_num)




from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Explicitly specify goals to get a simple navigation and manipulation demo.', usage='openrave.py --example simplemanipulation [options]')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)


if __name__ == "__main__":
    run()