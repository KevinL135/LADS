import math
import numpy as np
from numpy import array, cross
from numpy.linalg import solve, norm
from picamera2 import Picamera2
import cv2
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time

#import imagedetection

#motor setup
#define GPIO pins
direction= 8 # Direction (DIR) GPIO Pin
step = 11 # Step GPIO Pin
EN_pin = 7 # enable pin (LOW to enable)
 
dir2 = 27
step2 = 17
en2 = 22

# Declare a instance of class pass GPIO pins numbers and the motor type
outerMotor = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
GPIO.setup(EN_pin,GPIO.OUT) # set enable pin as output

innerMotor = RpiMotorLib.A4988Nema(dir2, step2, (21,21,21), "DRV8825")
GPIO.setup(en2,GPIO.OUT) # set enable pin as output

GPIO.output(EN_pin,GPIO.LOW) # pull enable to low to enable motor
GPIO.output(en2,GPIO.LOW) # pull enable to low to enable motor


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
  print("Gap: " + str(distance))
  return vector(intersection[0], intersection[1], intersection[2])

def parabolicAngle(pos:vector):
  lr = math.degrees(math.atan(-pos.getX()/pos.getZ()))
  groundDistance = math.sqrt(pos.getX()*pos.getX()+pos.getZ()*pos.getZ())
  ud = estimateAngle(groundDistance, pos.getY())
  sAngle = sphericalAngle(lr, ud)
  return sAngle

def directAngle(pos: vector):
  sAngle = posToSpherical(pos)
  return sAngle  

def rotateToAngle(fAngle: frameAngle):
  # if (fAngle.innerAngle > maxLeft or fAngle.innerAngle < maxRight):
  #   raise Exception("Out of bounds")
  global currentAngle
  targetOuterSteps = round(fAngle.outerAngle/stepAngle)
  outerStepAngle = targetOuterSteps*stepAngle
  targetInnerSteps = round(fAngle.innerAngle/stepAngle)
  innerStepAngle = targetInnerSteps*stepAngle

  currentOuterSteps = round(currentAngle.outerAngle/stepAngle)
  currentInnerSteps = round(currentAngle.innerAngle/stepAngle)

  deltaOuterSteps = targetOuterSteps - currentOuterSteps
  deltaInnerSteps = targetInnerSteps - currentInnerSteps

  outerDirection = False
  if (deltaOuterSteps < 0):
    deltaOuterSteps = -deltaOuterSteps
    outerDirection = not outerDirection 

  innerDirection = False
  if (deltaInnerSteps < 0):
    deltaInnerSteps = -deltaInnerSteps
    innerDirection = not innerDirection 

  print("Outer Steps: " + str(deltaOuterSteps))
  print("Inner Steps: " + str(deltaInnerSteps))

  outerMotor.motor_go(False, # False=Clockwise, True=Counterclockwise
                         "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                         deltaOuterSteps, # number of steps
                        .0005, # step delay [sec]
                         outerDirection, # True = print verbose output 
                         0) # initial delay [sec]

  innerMotor.motor_go(False, # False=Clockwise, True=Counterclockwise
                         "Full" , # Step type (Full,Half,1/4,1/8,1/16,1/32)
                         deltaInnerSteps, # number of steps
                        .0005, # step delay [sec]
                         innerDirection, # True = print verbose output 
                         0) # initial delay [sec]
  
  currentAngle = frameAngle(outerStepAngle, innerStepAngle) 
  print("Rotated to <" + str(currentAngle.outerAngle) + ", " + str(currentAngle.innerAngle) + ">")

def fire():
  #Add code
  print("Fired")

def rotateToFireAtPosition(pos: vector):
  aimAngle = directAngle(pos)
  rotateToAngle(sphericalToFrame(aimAngle))
  input("Press the Enter key to fire") 
  fire()


def obj_data(img):
     obj_pos = (0,0)
     hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
     mask=cv2.inRange(hsv,lower_range,upper_range)
     _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
     cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
     for c in cnts:
        x=400
        if cv2.contourArea(c)>x:
            x,y,w,h=cv2.boundingRect(c)
            centerx = (int) (x+w/2)
            centery = (int) (y+h/2)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.rectangle(img,(centerx,centery),(centerx,centery),(0,0,255),5)
            obj_pos = (x,y)

     return obj_pos

g = 981 # cm/s^2
pivotToTip = 50
waterSpeed = 500
stepAngle = 0.225 # degrees 

rCamPos = vector(6.9, -11.3, 1.7) # change later 
lCamPos = vector(-7.05, -11.3, 1.7) # change later 
rCamTilt = 10.93 # degrees
lCamTilt = 10.16 # degrees

rCamSlopePerPixel = 0.001500
lCamSlopePerPixel = 0.001500

maxLeft = 40 # degrees
maxRight = -40 # degrees

currentAngle = frameAngle(0, 0)

lower_range=np.array([51,38,216])#color detection ranges
upper_range=np.array([91,255,255])

#camera Setup
cam0 = Picamera2(0)
cam1 = Picamera2(1)

height = 600
width = 800
middle = (int(width / 2), int(height / 2))
cam0.configure(cam0.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
cam0.start()
cam1.configure(cam0.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
cam1.start()
    
while True:
    width = 800
    height= 600
    
    pointx = 400
    pointy = 100
    
    centerx = 400
    centery = 300
    frame0=cam0.capture_array()
    frame0=cv2.resize(frame0,(width,height))
    cv2.line(frame0, (pointx, pointy), (pointx, pointy), (0, 0, 255), 10) 
    cv2.line(frame0, (centerx,centery), (centerx,centery), (0, 255, 0), 10) 
    obj_width_in_frame0=obj_data(frame0)
    
    frame1=cam1.capture_array()
    frame1=cv2.resize(frame1,(width,height))
    cv2.line(frame1, (centerx,centery), (centerx,centery), (0, 255, 0), 10) 
    cv2.line(frame1, (pointx, pointy), (pointx, pointy), (0, 0, 255), 10) 
    obj_width_in_frame1=obj_data(frame1)
    if (obj_width_in_frame1[0] != 0 ) and (obj_width_in_frame0[0] != 0 ):
        x=obj_width_in_frame0[0]
        y=obj_width_in_frame0[1]
        cv2.putText(frame0, f"Position: {x}, {y}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)
        x=obj_width_in_frame1[0]
        y=obj_width_in_frame1[1]
        cv2.putText(frame1, f"Position: {x}, {y}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
        
            #Main firing sequence

            #Detect position code hererCamreldir = vector((obj_width_in_frame0[0]-centerx)*rCamSlopePerPixel, (obj_width_in_frame0[1]-centery)*rCamSlopePerPixel, 1)
            lCamreldir = vector((obj_width_in_frame1[0]-centerx)*lCamSlopePerPixel, (obj_width_in_frame1[1]-centery)*lCamSlopePerPixel, 1)
            rCamreldir = vector((obj_width_in_frame0[0]-centerx)*rCamSlopePerPixel, (obj_width_in_frame0[1]-centery)*rCamSlopePerPixel, 1)


            rCamfa = posToFrame(rCamreldir)
            rCamfa.outerAngle = rCamfa.outerAngle + rCamTilt
            rCamDirection = frameToPos(rCamfa)
            print("Right Cam Direction:" + str(rCamDirection))

            lCamfa = posToFrame(lCamreldir)
            lCamfa.outerAngle = lCamfa.outerAngle + lCamTilt
            lCamDirection = frameToPos(lCamfa)
            print("Right Cam Direction:" + str(lCamDirection))

            target = findIntersection(rCamPos, arrayToVector(rCamPos.getArray()+rCamDirection.getArray()), lCamPos, arrayToVector(lCamPos.getArray()+lCamDirection.getArray()))
            print(target)
            rotateToFireAtPosition(target)
    cv2.imshow("Right",frame0)
    cv2.imshow("Left",frame1)
    cv2.moveWindow("frame0",50,100)
    cv2.moveWindow("frame1",50,900)
    if cv2.waitKey(1)&0xFF==27:
        break

cv2.destroyAllWindows()


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
