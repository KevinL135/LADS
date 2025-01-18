import math
import numpy as np
from numpy import array, cross
from numpy.linalg import solve, norm
from picamera2 import Picamera2
import cv2


#All distances in centimeters, origin is center of rotation 
#Facing the direction of the pressure washer, +x is right, +y is up, +z is forward
#For spherical coordinates, (0,0) is forwards, a rotates ccw around the +y axis, and b rotates ccw around the +x axis
#Arrays are x, y, z

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
  
def arrayToVector(array: np.array):
  return vector(array[0], array[1], array[2])

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

def frameToPos(angle: frameAngle):
  x = -math.sin(angle.innerRadians())
  y = math.cos(angle.innerRadians())*math.sin(angle.outerRadians())
  z = math.cos(angle.innerRadians())*math.cos(angle.outerRadians())
  p = vector(x, y, z)
  return p
  
def posToFrame(p: vector):
  outerAngle = math.degrees(math.atan(p.getY()/p.getZ()))
  yzhypothenuse = math.sqrt(p.getY()*p.getY()+p.getZ()*p.getZ())
  innerAngle = math.degrees(math.atan(-p.getX()/yzhypothenuse))
  fAngle = frameAngle(outerAngle, innerAngle)
  return fAngle

def sphericalToFrame(angle: sphericalAngle):
  p = sphericalToPos(angle)
  fAngle = posToFrame(p)
  return fAngle

def frameToSpherical(angle: frameAngle):
  p = frameToPos(angle)
  sAngle = posToSpherical(p)
  return sAngle


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

def findIntersection(A0: vector, A1: vector, B0: vector, B1: vector):

  # compute unit vectors of directions of lines A and B
  UA = (A1.getArray() - A0.getArray()) / norm(A1.getArray() - A0.getArray())
  UB = (B1.getArray() - B0.getArray()) / norm(B1.getArray() - B0.getArray())
  # find unit direction vector for line C, which is perpendicular to lines A and B
  UC = cross(UB, UA)
  UC /= norm(UC)

  # solve the system derived in user2255770's answer from StackExchange: https://math.stackexchange.com/q/1993990
  RHS = B0.getArray() - A0.getArray()
  LHS = array([UA, -UB, UC]).T
  tOutput = solve(LHS, RHS)
  p1 = A0.getArray() + tOutput[0]*UA
  p2 = B0.getArray() + tOutput[1]*UB
  intersection = (p1+p2)/2
  distanceVector = p1-p2
  distance = math.sqrt(distanceVector[0]*distanceVector[0] + distanceVector[1]*distanceVector[1] + distanceVector[2]*distanceVector[2])
  print("Dist: " + str(distance))
  return vector(intersection[0], intersection[1], intersection[2])

def rotateToAngle(fAngle: frameAngle):
  # if (fAngle.innerAngle > maxLeft or fAngle.innerAngle < maxRight):
  # raise Exception("Out of bounds")
  outerSteps = round(fAngle.outerAngle/stepAngle)
  outerStepAngle = outerSteps*stepAngle
  innerSteps = round(fAngle.innerAngle/stepAngle)
  innerStepAngle = innerSteps*stepAngle

  #Add code
  global currentAngle
  currentAngle = frameAngle(outerStepAngle, innerStepAngle) 
  print("Rotated to <" + str(currentAngle.outerAngle) + ", " + str(currentAngle.innerAngle) + ">")

def fire():
  #Add code
  print("Fired")

def rotateToFireAtPosition(pos: vector):
  lr = math.degrees(math.atan(-pos.getX()/pos.getZ()))
  groundDistance = math.sqrt(pos.getX()*pos.getX()+pos.getZ()*pos.getZ())
  ud = estimateAngle(groundDistance, pos.getY())
  sAngle = sphericalAngle(lr, ud)
  rotateToAngle(sphericalToFrame(sAngle))

  input("Press the Enter key to fire") 
  fire()

def pointToPosition(pos: vector):
  sAngle = posToSpherical(pos)
  rotateToAngle(sAngle)

g = 981 # cm/s^2
pivotToTip = 50
waterSpeed = 500
stepAngle = 0.225 # degrees 

rCamPos = vector(7, -10, 1) # change later 
lCamPos = vector(-7, -10, 1) # change later 
rCamTilt = 10.93 # degrees
lCamTilt = 10.16 # degrees

maxLeft = 40 # degrees
maxRight = -40 # degrees

currentAngle = frameAngle(0, 0)

#Main firing sequence

#Detect position code here
rCamreldir = vector(-5, 12, 20)
lCamreldir = vector(6, -2, 20)

rCamfa = posToFrame(rCamreldir)
rCamfa.outerAngle = rCamfa.outerAngle + rCamTilt
rCamDirection = frameToPos(rCamfa)

lCamfa = posToFrame(lCamreldir)
lCamfa.outerAngle = lCamfa.outerAngle + lCamTilt
lCamDirection = frameToPos(lCamfa)

target = findIntersection(rCamPos, arrayToVector(rCamPos.getArray()+rCamDirection.getArray()), lCamPos, arrayToVector(lCamPos.getArray()+lCamDirection.getArray()))
rotateToFireAtPosition(target)


#Tests 
print("Testing")
# print(mathtest.testing())
# testA = sphericalAngle(40, 60)
# print(testA.lr)
# v = sphericalToPos(testA)
# print(v)
# sA = sphericalToFrame(testA)
# print(sA.outerAngle)
# print(sA.innerAngle)
# print(calculateTrajectory(40, 120))
# print(estimateAngle(120, 145))

# print("Intersection test")
# print(findIntersection(vector(0, 0, 2), vector(3, 4, 0), vector(4, 0, 0), vector(0, 3, 2)))
