import cv2
import numpy as np
import random
import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import Jetson.GPIO as GPIO
import smbus
from adafruit_servokit import ServoKit
import sys

myKit=ServoKit(channels=16)

# Define servo channels (adjust if needed)
SERVO_LREX = 0  # Light and Right X on the same
SERVO_LREY = 1
SERVO_LUPLID = 2
SERVO_RUPLID = 3
SERVO_LLOLID = 4
SERVO_RLOLID = 5
last_blink_time = 0  # Initialize last blink time
blink_interval = 5  # Minimum interval between blinks (in seconds)

def centerEyes():
    myKit.servo[SERVO_LREX].angle = 100
    myKit.servo[SERVO_LREY].angle = 80
    myKit.servo[SERVO_LLOLID].angle = 100
    myKit.servo[SERVO_RLOLID].angle = 90
    myKit.servo[SERVO_LUPLID].angle = 70
    myKit.servo[SERVO_RUPLID].angle = 120

def blinkEyes():
    global last_blink_time
    current_time = time.time()
    if (current_time - last_blink_time) > blink_interval and random.randint(1, 500) <= 5: 
        myKit.servo[SERVO_LLOLID].angle = 10
        myKit.servo[SERVO_RLOLID].angle = 180
        myKit.servo[SERVO_LUPLID].angle = 180
        myKit.servo[SERVO_RUPLID].angle = 10
        time.sleep(.2)
        myKit.servo[SERVO_LLOLID].angle = 100
        myKit.servo[SERVO_RLOLID].angle = 90
        myKit.servo[SERVO_LUPLID].angle = 70
        myKit.servo[SERVO_RUPLID].angle = 120
        last_blink_time = current_time  # Update last blink time

def map_value(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

def constrain(value, low, high):
    return max(min(value, high), low)

########################################
## Variables for Distance Reference
######################################## 
# distance from camera to object(face) measured
KNOWN_DISTANCE = 76.2  # centimeter
ref_image_face_width = 182 # Width of face with known distance
# width of face in the real world or Object Plane
KNOWN_WIDTH = 14.3  # centimeter

########################################
## Variables for video servo dimesions 
######################################## 
#videoX = 1280
videoX = 640
#videoY = 960
videoY = 480
servoX = 1300
servoY = 1400
servoXMin = 900
servoYMin = 800

#Angles for servos 
EyeYMax = 112
EyeYMin = 52
EyeXMax = 145
EyeXMin = 57

########################################
## Variables to start video capture
######################################## 

# Position eyes
centerEyes()
blinkEyes()

# Setup video capture stream with webcam
camera_id = "/dev/video0"
video_capture = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
video_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, videoX)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, videoY)
video_capture.set(cv2.CAP_PROP_FPS, 20)
face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

########################################
## Variables for FPS calc
######################################## 
# used to record the time when we processed last frame 
prev_frame_time = 0
  
# used to record the time at which we processed current frame 
new_frame_time = 0

# focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):
    """
    This Function Calculate the Focal Length(distance between lens to CMOS sensor), it is simple constant we can find by using
    MEASURED_DISTACE, REAL_WIDTH(Actual width of object) and WIDTH_OF_OBJECT_IN_IMAGE
    :param1 Measure_Distance(int): It is distance measured from object to the Camera while Capturing Reference image

    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 14.3 centimeters)
    :param3 Width_In_Image(int): It is object width in the frame /image in our case in the reference image(found by Face detector)
    :retrun focal_length(Float):"""
    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value


# distance estimation function
def distance_finder(focal_length, real_face_width, face_width_in_frame):
    """
    This Function simply Estimates the distance between object and camera using arguments(focal_length, Actual_object_width, Object_width_in_the_image)
    :param1 focal_length(float): return by the focal_length_Finder function

    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 5.7 Inches)
    :param3 object_Width_Frame(int): width of object in the image(frame in our case, using Video feed)
    :return Distance(float) : distance Estimated
    """
    distance = (real_face_width * focal_length) / face_width_in_frame
    return distance

# face detector function
def face_data(image):
    """
    This function Detect the face
    :param Takes image as argument.
    :returns face_width in the pixels
    """

    face_width = 0
    face_width = 0
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(gray_image, 1.3, 5)
    for (x, y, h, w) in faces:
        #cv2.rectangle(image, (x, y), (x + w, y + h), WHITE, 1)
        print('{} {} {} {}'.format(x,y,h,w))
        face_width = w

    return face_width



########################################
## Misc variables
######################################## 
focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_face_width)
#cv2.imshow("ref_image", ref_image)

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []


maxYfound=0
maxXfound=0
minYfound=2000
minXfound=2000
########################################
## Begin loop on processing frames
######################################## 
while True:
    # Grab a single frame of video
    ret, frame = video_capture.read()

    blinkEyes()

    # Only process every other frame of video to save time
    if frame is not None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        face_locations = face_detector.detectMultiScale(gray, 1.3, 5)

        # time when we finish processing for this frame 
        new_frame_time = time.time() 

        # fps will be number of frame processed in given time frame 
        # since their will be most of time error of 0.001 second 
        # we will be subtracting it to get more accurate result 
        fps = 1/(new_frame_time-prev_frame_time) 
        prev_frame_time = new_frame_time 
  
        # converting the fps into integer 
        fps = int(fps) 
        #print('FPS: {}'.format(fps))


    shortest_distance = None
    centerX = 0
    centerY = 0    
    for (x, y, h, w) in face_locations:
        #print('Face_Location_CV2: {} {} {} {}'.format(x,y,h,w))

        Distance = distance_finder(focal_length_found, KNOWN_WIDTH, w)
        if(shortest_distance is None or Distance < shortest_distance):
           shortest_distance = Distance
           top = y
           bottom = y+h
           right = x+w
           left = x
           centerX = int(((right-left)/2)+left)
           centerY = int(((top-bottom)/2)+bottom)

           # Draw a box around the face
           cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

    ## If shortest distances in faces found, move servo
    if(shortest_distance is not None):
       
        # Calculate servo positions based on face position and distance
        #print(f"servoX={servoX}, servoY={servoY}")
        #print(f"distance={shortest_distance}, top={top}, bottom={bottom}, right={right}, left={left}, centerX={centerX}, centerY={centerY}")
        #servoXMax_mod = servoX - int((20000 / shortest_distance))
        #servoYMax_mod = servoY - int((70000 / shortest_distance))
        #servoYMin_mod = servoYMin + int((70000 / shortest_distance))

        setServoX = centerX
        setServoY = centerY
        #setServoX = int((((videoX - right) * servoXMax_mod) / videoX) + servoXMin_mod)
        #setServoY = int((((videoY - top) * servoYMax_mod) / videoY) + servoYMin_mod)

        # Map calculated coordinates directly to servo angles
        servo_angle_eye_x = map_value(setServoX, 0, videoX, EyeXMax, EyeXMin)
        servo_angle_eye_y = map_value(setServoY, 0, videoY, EyeYMax, EyeYMin)
        
        #print(f"servo_angle_reye_x={servo_angle_reye_x}, servo_angle_reye_y={servo_angle_reye_y}, servo_angle_leye_x={servo_angle_leye_x}, servo_angle_leye_y={servo_angle_leye_y}")

        # Set servo angles using ServoKit
        myKit.servo[SERVO_LREX].angle = servo_angle_eye_x
        myKit.servo[SERVO_LREY].angle = servo_angle_eye_y

        time.sleep(0.005)  # 5ms delay

        shortest_distance = None

    # Display the resulting image
    try:
       cv2.imshow('Video', frame)
    except:
       pass

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()

