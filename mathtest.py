import math
import numpy as np
from numpy import array, cross
from numpy.linalg import solve, norm

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
  def __init__(self, a, b):
    self.a = a
    self.b = b
  def aRadians(self):
    return math.radians(self.a)
  def bRadians(self):
    return math.radians(self.b)

class frameAngle: #angles in degrees 
  def __init__(self, outer, inner):
    self.outerAngle = outer # 0 is forwards, positive is upwards
    self.innerAngle = inner # 0 is forwards, positive is left 
  def outerRadians(self):
    return math.radians(self.outerAngle)
  def innerRadians(self):
    return math.radians(self.innerAngle)

currentAngle = sphericalAngle(0, 0)

def sphericalToXYZ(angle: sphericalAngle): #return normalized positiion
  x = -math.cos(angle.bRadians())*math.sin(angle.aRadians())
  y = math.sin(angle.bRadians())
  z = math.cos(angle.bRadians())*math.cos(angle.aRadians())
  p = vector(x, y, z)
  return p

def sphericalToFrame(angle: sphericalAngle):
  p = sphericalToXYZ(angle)
  outerAngle = math.degrees(math.atan(p.getY()/p.getZ()))
  yzhypothenuse = math.sqrt(p.getY()*p.getY()+p.getZ()*p.getZ())
  innerAngle = math.degrees(math.atan(-p.getX()/yzhypothenuse))
  fAngle = frameAngle(outerAngle, innerAngle)
  return fAngle

g = 981 #cm/s^2
camera1Pos = vector(7, -10, 1) # change later 
camera2Pos = vector(-7, -10, 1) # change later 


testA = sphericalAngle(40, 60)
print(testA.a)
v = sphericalToXYZ(testA)
print(v)
sA = sphericalToFrame(testA)
print(sA.outerAngle)
print(sA.innerAngle)