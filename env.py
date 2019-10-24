from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from openrave_utils.funcs import *
import os

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

    #get current state
    # print robot.GetDOFValues(manipulator.GetArmIndices())   
    # print("before start:",env.CheckCollision(robot))

    #find available solution
    start_config = generate_goalIk(right=True,robot=robot)

    #execute solution
    # execute_dofValues(solution=start_config,robot=robot,env=env)
    execute_activeDOFValues(solution=start_config,robot=robot,env=env)   
    # print current_dofValues(robot)
    time.sleep(3)

    #check collision
    # print("after start:",env.CheckCollision(robot))    
    
    #find available solution
    goal_config = generate_goalIk(right=False,robot=robot)
    
    #execute solution
    # execute_dofValues(solution=goal_config,robot=robot,env=env)
    # execute_activeDOFValues(solution=goal_config,robot=robot,env=env)
    # print current_dofValues(robot)      
    # time.sleep(10)

    planner_name = "OMPL_RRTstar"
    planner = RaveCreatePlanner(env, planner_name)
    simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')

    params = Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(goal_config)

    param_string_old = """<_postprocessing planner="parabolicsmoother">
    <_nmaxiterations>100</_nmaxiterations>
    </_postprocessing>"""
    params.SetExtraParameters(param_string_old)

    with env:
        with robot:
            # Invoke the planner.
            print 'Calling the %s planner.'%planner_name
            traj = RaveCreateTrajectory(env, '')
            planner.InitPlan(robot, params)
            result = planner.PlanPath(traj)
            assert result == PlannerStatus.HasSolution
            getData_trajectory(traj)

            # Time the trajectory.
            print 'Timing trajectory'
            result = planningutils.RetimeTrajectory(traj)
            assert result == PlannerStatus.HasSolution
            getData_trajectory(traj)

    # Execute the trajectory.
    raw_input('Press <ENTER> to execute trajectory.')
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)
    time.sleep(2)





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