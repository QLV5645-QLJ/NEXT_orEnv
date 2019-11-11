from openravepy import *
import numpy, time

def run():
	env=Environment()
	env.Load('data/lab1.env.xml')
	env.SetViewer('qtcoin')
	robot = env.GetRobots()[0]
	target = env.GetKinBody('mug1')
	gmodel = databases.grasping.GraspingModel(robot,target)
	if not gmodel.load():
	    gmodel.autogenerate()

	validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
	basemanip = interfaces.BaseManipulation(robot)
	with robot:
	    grasp = validgrasps[0]
	    gmodel.setPreshape(grasp)
	    T = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
	    traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)


	raveLogInfo('traj has %d waypoints, last waypoint is: %s'%(traj.GetNumWaypoints(),repr(traj.GetWaypoint(-1))))
	robot.GetController().SetPath(traj)
	robot.WaitForController(0)
	check = env.CheckCollision(robot)
	print("robot collision before grab:",check)
	#grasp config
	taskmanip = interfaces.TaskManipulation(robot)
	taskmanip.CloseFingers()
	time.sleep(5)
	robot.Grab(target)

	check = env.CheckCollision(robot)
	print("robot collision afetr grab:",check)
	#get stuff
	T[2][3] = T[2][3] + 0.1
	traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)
	robot.GetController().SetPath(traj)
	robot.WaitForController(0)
	time.sleep(5)
	check = env.CheckCollision(robot)
	print("robot collision after move:",check)

	#release fingers
	taskmanip = interfaces.TaskManipulation(robot)
	taskmanip.ReleaseFingers()
	time.sleep(5)

	#move without object
	robot.ReleaseAllGrabbed()
	T[2][3] = T[2][3] + 0.1
	traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)
	robot.GetController().SetPath(traj)
	robot.WaitForController(0)
	time.sleep(5)


	time.sleep(20)

if __name__ == "__main__":
	run()