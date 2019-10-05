import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
pupperId = p.loadMJCF("pupper_pybullet.xml")
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
