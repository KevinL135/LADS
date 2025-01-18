import math
import numpy as np
from numpy import array, cross
from numpy.linalg import solve, norm

#All distances in centimeters, origin is center of rotation 
#Facing the direction of the pressure washer, +x is right, +y is up, +z is forward
#For spherical coordinates, (0,0) is forwards, a rotates ccw around the +y axis, and b rotates ccw around the +x axis
#Arrays are x, y, z

avar = 5


def testing():
  return avar

class vector:
  def __init__(self, x, y, z):
    self.arr = np.array([x, y, z])
  def setX(self, x):
    self.arr[0] = x
  def setY(self, y):
    self.arr[1] = y
  def setZ(self, z):
    self.arr[2] = z
  def getX(self):
    return self.arr[0]
  def getY(self):
    return self.arr[1]
  def getZ(self):
    return self.arr[2]
  def getArray(self):
    return self.arr
  def __str__(self):
    return f"({self.getX()}, {self.getY()}, {self.getZ()})"

class sphericalAngle: #angles in degrees
  def __init__(self, lr, ud):
    self.lr = lr
    self.ud = ud
  def lrRadians(self):
    return math.radians(self.lr)
  def udRadians(self):
    return math.radians(self.ud)

class frameAngle: #angles in degrees 
  def __init__(self, outer, inner):
    self.outerAngle = outer # 0 is forwards, positive is upwards
    self.innerAngle = inner # 0 is forwards, positive is left 
  def outerRadians(self):
    return math.radians(self.outerAngle)
  def innerRadians(self):
    return math.radians(self.innerAngle)


def sphericalToPos(angle: sphericalAngle): #return normalized positiion
  x = -math.cos(angle.udRadians())*math.sin(angle.lrRadians())
  y = math.sin(angle.udRadians())
  z = math.cos(angle.udRadians())*math.cos(angle.lrRadians())
  p = vector(x, y, z)
  return p

def posToSpherical(p: vector):
  lr = math.degrees(math.atan(-p.getX()/p.getZ()))
  xzhypothenuse = math.sqrt(p.getX()*p.getX()+p.getZ()*p.getZ())
  ud = math.degrees(math.atan(p.getY()/xzhypothenuse))
  sAngle = sphericalAngle(lr, ud)
  return sAngle

def sphericalToFrame(angle: sphericalAngle):
  p = sphericalToPos(angle)
  outerAngle = math.degrees(math.atan(p.getY()/p.getZ()))
  yzhypothenuse = math.sqrt(p.getY()*p.getY()+p.getZ()*p.getZ())
  innerAngle = math.degrees(math.atan(-p.getX()/yzhypothenuse))
  fAngle = frameAngle(outerAngle, innerAngle)
  return fAngle

def calculateTrajectory(angle, x): # angle in degrees, calculates y value of a point on trajectory given x value 
  angleRad = math.radians(angle)
  y = (math.tan(angleRad)*(x-pivotToTip*math.cos(angleRad)) -
      (g/2)*pow(((x-pivotToTip*math.cos(angleRad))/(waterSpeed*math.cos(angleRad))), 2) + 
      pivotToTip*math.sin(angleRad)) 
  return y

def estimateAngle(x, y): # returns estimated angle in degrees 
  a = -45
  y1 = calculateTrajectory(a, x)
  while (y-y1 > 0.01):
    a += (y-y1)*0.15
    y1 = calculateTrajectory(a, x)
  return a

def rotateToAngle(sAngle: sphericalAngle):
  fAngle = sphericalToFrame(sAngle);

  #Add code
  currentAngle = sAngle

def fire():
  #Add code
  print("Fired")

def rotateToFireAtPosition(pos: vector):
  lr = math.degrees(math.atan(-pos.getX()/pos.getZ()))
  groundDistance = math.sqrt(pos.getX()*pos.getX()+pos.getZ()*pos.getZ())
  ud = estimateAngle(groundDistance, pos.getY())
  sAngle = sphericalAngle(lr, ud)
  rotateToAngle(sAngle)

  input("Press the Enter key to fire") 
  fire()

def pointToPosition(pos: vector):
  sAngle = posToSpherical(pos)
  rotateToAngle(sAngle)

g = 981 # cm/s^2
pivotToTip = 50
waterSpeed = 500

camera1Pos = vector(7, -10, 1) # change later 
camera2Pos = vector(-7, -10, 1) # change later 

currentAngle = sphericalAngle(0, 0)

#Main firing sequence

#Detect position code here
target = vector(5, 7, 120)
rotateToFireAtPosition(target)


#Tests 

testA = sphericalAngle(40, 60)
print(testA.lr)
v = sphericalToPos(testA)
print(v)
sA = sphericalToFrame(testA)
print(sA.outerAngle)
print(sA.innerAngle)
print(calculateTrajectory(40, 120))
print(estimateAngle(120, 145))
