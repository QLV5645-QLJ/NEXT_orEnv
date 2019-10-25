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
def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def main(env,options):
    "Main example code."
    # load a scene from ProjectRoom environment XML file
    env.Load(os.getcwd()+'/worlds/exp1.env.xml')
    print(env.GetCollisionChecker())
    time.sleep(1)

    #get robot info
    robot = env.GetRobots()[0]
    manipulator = robot.GetManipulator('arm')
    robot.SetActiveManipulator(manipulator)

    #find available solution and execute solution 3.5,-1.2,1.0
    start_config = generate_goalIk_exp1(right=True,robot=robot,pos=[3.4,-1.2,1.0])
    # start_config = generate_goalIk(right=True,robot=robot)
    if(start_config is None):
        print("initialize robot arm eith failure")
        exit()
    execute_activeDOFValues(solution=start_config,robot=robot,env=env)   
    time.sleep(0.1)
    

    #use palner to plan path
    time_limit = 40.0
    motion_range = 20
    orplanner = ORPLANNER(robot=robot,env=env,planner_name = "OMPL_RRTstar",
        time_limit = time_limit, motion_range = motion_range)
    goal_num = 100
    success_num = 0
    inside = True
    delta_time = 0.0
    for i in range(goal_num):
        print("time_limit: %f motion_range: %f"%(time_limit,motion_range))
        print("path num:",i)
        print("sucess num:",success_num)
        print("total plan time:",delta_time)

        #generate goal
        goal_config = None
        if(inside):
            # goal_config = generate_goalIk(right=not inside,robot=robot)3.5,-1.7,1.0
            goal_config = generate_goalIk_exp1(right=True,robot=robot,pos=[3.4,-1.7,1.0])
        else:
            # goal_config = generate_goalIk(right=not inside,robot=robot)3.5,-1.2,1.0
            goal_config = generate_goalIk_exp1(right=True,robot=robot,pos=[3.4,-1.2,1.0])

        #generate trajectory
        if(goal_config is None):
            print("No goal IK solution")
            continue
        orplanner.set_goal(goal_config)
        start_time = time.time()
        traj = orplanner.plan_traj()
        end_time = time.time()
        if(traj is None):
            print("no trajectory solution")
            continue

        # Execute the trajectory.
        robot.GetController().SetPath(traj)
        robot.WaitForController(0)
        time.sleep(0.5)

        success_num+=1
        delta_time += (end_time-start_time)
        inside = not inside      
    # print("path num:",goal_num)
    # print("sucess time:",success_num)
    # print("total time:",delta_time)



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