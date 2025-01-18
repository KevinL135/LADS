import cv2
import numpy as np
from picamera2 import Picamera2

cam0 = Picamera2(0)
cam1 = Picamera2(1)

height = 600
width = 800
middle = (int(width / 2), int(height / 2))
cam0.configure(cam0.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
cam0.start()
cam1.configure(cam0.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
cam1.start()
lower_range=np.array([0,125,229])
upper_range=np.array([27,255,255])

Known_distance=1
Known_width=1

def Focal_Length_Finder(Known_distance, real_width, width_in_rf_image):

    focal_length = (width_in_rf_image * Known_distance) / real_width
    return focal_length

def obj_data(img):
     obj_pos = (0,0)
     hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
     mask=cv2.inRange(hsv,lower_range,upper_range)
     _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
     cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
     for c in cnts:
        x=600
        if cv2.contourArea(c)>x:
            x,y,w,h=cv2.boundingRect(c)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            obj_pos = (x,y)
     return obj_pos
    
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
    if obj_width_in_frame0[0] != 0:
        x=obj_width_in_frame0[0]
        y=obj_width_in_frame0[1]
        cv2.putText(frame0, f"Position: {x}, {y}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)

    frame1=cam1.capture_array()
    frame1=cv2.resize(frame1,(width,height))
    cv2.line(frame1, (centerx,centery), (centerx,centery), (0, 255, 0), 10) 
    cv2.line(frame1, (pointx, pointy), (pointx, pointy), (0, 0, 255), 10) 
    obj_width_in_frame1=obj_data(frame1)
    if obj_width_in_frame1[0] != 0:
        x=obj_width_in_frame1[0]
        y=obj_width_in_frame1[1]
        cv2.putText(frame1, f"Position: {x}, {y}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)


    cv2.imshow("Right",frame0)
    cv2.imshow("Left",frame1)
    if cv2.waitKey(1)&0xFF==27:
        break
cap.release()
cv2.destroyAllWindows()

def camerasGetTargetPixel():

    frame0=cam0.capture_array()
    frame0=cv2.resize(frame0,(width,height))

    obj_width_in_frame0=obj_data(frame0)
    if obj_width_in_frame0[0] != 0:
        x=obj_width_in_frame0[0]
        y=obj_width_in_frame0[1]
        cv2.putText(frame0, f"Position: {x}, {y}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)

    frame1=cam1.capture_array()
    frame1=cv2.resize(frame1,(width,height))
    #cv2.line(frame1, (centerx,centery), (centerx,centery), (0, 255, 0), 10) 
    #cv2.line(frame1, (pointx, pointy), (pointx, pointy), (0, 0, 255), 10) 
    obj_width_in_frame1=obj_data(frame1)
    if obj_width_in_frame1[0] != 0:
        x=obj_width_in_frame1[0]
        y=obj_width_in_frame1[1]
        cv2.putText(frame1, f"Position: {x}, {y}", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)


    # cv2.imshow("Right",frame0)
    # cv2.imshow("Left",frame1)
    # if cv2.waitKey(1)&0xFF==27:
    #     break
    # cap.release()
    # cv2.destroyAllWindows()
    return np.array([height, width])