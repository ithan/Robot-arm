import pybullet
import time
import pybullet_data
import numpy

#pybullet configuration
physicsClient = pybullet.connect(pybullet.GUI)#or pybullet.DIRECT for non-graphical version
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
pybullet.setGravity(0,0,-10)
planeId = pybullet.loadURDF("plane.urdf")
frame = 0


# Arm
class RoboticArm:
	def __init__(self, name , pos, orinetation, maxForce):
		self.armpath = "C:\\Users\\ithan\\OneDrive\\Escritorio\\Robo hand\\HIAlternative\\ar2.urdf"
		self.cubeStartPos = pos
		self.cubeStartOrientation = orinetation
		self.name = name
		self.URDF = 0
		self.debugIds = []
		self.maxForce = maxForce

	def init(self):
		for joint in range(pybullet.getNumJoints(self.URDF)):
			name = self.name + " - " + str(joint)
			self.addDebugParams(name, -5, 5, 0)
		#collisiones correctas
		pybullet.setCollisionFilterGroupMask(self.URDF, -1, 0, 0)
		for e in list([3,4]):
			for i in range(6):
				pybullet.setCollisionFilterPair(self.URDF, self.URDF, e, i, 0)

	def loadURDF(self):
		tempflags = pybullet.URDF_USE_INERTIA_FROM_FILE |  pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL | pybullet.URDF_USE_SELF_COLLISION
		self.URDF = pybullet.loadURDF(self.armpath,self.cubeStartPos, self.cubeStartOrientation, useFixedBase= 1, flags = tempflags)

	def changeVelocity(self, jointIndex, velocity):
		pybullet.setJointMotorControl2(self.URDF, jointIndex ,controlMode=pybullet.VELOCITY_CONTROL, force=self.maxForce ,targetVelocity=velocity)

	def addDebugParams(self,name, min, max, start):
		tempId = pybullet.addUserDebugParameter(paramName = name ,  rangeMin = min,  rangeMax = max, startValue = start)
		self.debugIds.append( tempId )

	def updateFromParams(self):
		for joint in range(pybullet.getNumJoints(self.URDF)):
			params = pybullet.readUserDebugParameter(self.debugIds[joint])
			self.changeVelocity(joint, params)

class Sphere:
	def __init__(self,pos):
		self.path = "C:\\Users\\ithan\\OneDrive\\Escritorio\\Robo hand\\HIAlternative\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\sphere_small.urdf"
		self.cubeStartPos = pos
		self.URDF = 0
	def init(self):
		self.URDF = pybullet.loadURDF(self.path,self.cubeStartPos, useFixedBase= 1)


boxIds = []
boxIds.append( RoboticArm("base", [0,0,0], pybullet.getQuaternionFromEuler([0,0,0]), 500) )

#create the sphere
sphere = Sphere([.5,.5,1])
sphere.init()
textid = 0

for boxId in boxIds:
	boxId.loadURDF()
	boxId.init()

#create the robotic arms
def update():
	for boxId in boxIds:
		boxId.updateFromParams()

def distance_finder(vectorA,vectorB) :
    return (((vectorB[0]-vectorA[0])**2)+((vectorB[1]-vectorA[1])**2)+((vectorB[2]-vectorA[2])**2))**(1/2)

while pybullet.isConnected(physicsClient):

	frame = frame + 1
	update()
	roboPos = list(pybullet.getLinkState(boxIds[0].URDF, 5)[0])
	spherePos = list(pybullet.getBasePositionAndOrientation(sphere.URDF)[0])
	dist = distance_finder( roboPos , spherePos)
	if frame % 30 == 0:
		if textid != 0:
			pybullet.removeUserDebugItem(textid)
		textid = pybullet.addUserDebugText(str("%.2f" % dist), [.5,.5,1], textColorRGB= [1,0,0])
	pybullet.stepSimulation()
	time.sleep(1./240.)


pybullet.disconnect()
