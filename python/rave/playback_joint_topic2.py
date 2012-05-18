import openravepy as rave
import numpy as np
from os.path import join
import time
from comm import comm

env = rave.Environment()
env.SetViewer('qtcoin')
env.Load('robots/pr2-beta-sim.robot.xml')
pr2 = env.GetRobots()[0]


comm.setDataRoot("/home/joschu/Data/comm3")
jointSub = comm.FileSubscriber("joints","txt",comm.ArrayMessage)


rosnames = np.load("/home/joschu/Data/knot_kinect2/joint_names.npy")
ros2rave = np.array([pr2.GetJointIndex(name) for name in rosnames])

while True:
    try:
        msg = jointSub.recv()        
    except StopIteration:
        break

    rosvalues = msg.data
    
    goodrosinds = np.flatnonzero(ros2rave != -1)
    
    raveinds = ros2rave[goodrosinds]
    ravevalues = rosvalues[goodrosinds]
    
    
    pr2.SetJointValues(ravevalues[:20],raveinds[:20])
    pr2.SetJointValues(ravevalues[20:],raveinds[20:])
    
    
    time.sleep(.01)
    
    
    
    
