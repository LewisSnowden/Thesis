"""
Demonstration of the GazeTracking library.
Check the README.md for complete documentation.
"""

import cv2
from gaze_tracking import GazeTracking
import mediapipe as mp
import math
import time
import numpy as np
import random
import keyboard
import csv



def mouth_ratio(landmarks):
        """Calculates a ratio that can indicate whether an mouth is open or not.
        It's the division of the width of the eye, by its height.

        Arguments:
            landmarks (multi_face_landmarks): Facial landmarks for the face region
        Returns:
            The computed ratio
        """
        left = (landmarks.landmark[78].x, landmarks.landmark[78].y)
        right = (landmarks.landmark[308].x, landmarks.landmark[308].y)
        top = (landmarks.landmark[13].x, landmarks.landmark[13].y)
        bottom = (landmarks.landmark[14].x, landmarks.landmark[14].y)

        mouth_width = math.hypot((left[0] - right[0]), (left[1] - right[1]))
        mouth_height = math.hypot((top[0] - bottom[0]), (top[1] - bottom[1]))

        try:
            ratio = mouth_width / mouth_height
        except ZeroDivisionError:
            ratio = None

        return ratio
def mouth_open(landmarks):
    mouthVal = mouth_ratio(landmarks)
    if mouthVal is None: #checking if landmarks are of type none if so return -1 error code
        return -1
    elif mouthVal<=5:  #if the mouth ratio is less than 5 mouth is open return 1
        return 1
    else:
        return 0 #else return mouth open to be 0





mp_drawing = mp.solutions.drawing_utils
mp_face_mesh = mp.solutions.face_mesh

gaze = GazeTracking()
webcam = cv2.VideoCapture(0)
#webcam2 = cv2.VideoCapture(1,cv2.CAP_DSHOW)
#webcam2.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
#webcam2.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
#webcam2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
#webcam2.set(cv2.CAP_PROP_FPS, 30)
#webcam2.set(cv2.CAP_PROP_BUFFERSIZE,3)

last_action =""
last_action_sent=""
system_status = "Standby"
system_mode = "Forward"
exit = False
RIGHT_EYE_POINTS= [33, 246, 161, 160, 159, 158, 157, 173, 133, 155, 154, 153, 145, 144, 163, 7]
LEFT_EYE_POINTS= [362, 398, 384, 385, 386, 387, 388, 466, 263, 249, 390, 373, 374, 380, 381, 382]
f = open("TestResults.csv","w",encoding='UTF8',newline='')
writer=csv.writer(f)
total_tests=0
total_correct=0
writer.writerow(['Displayed Direction','X Value','Detected Direction'])




with mp_face_mesh.FaceMesh(min_detection_confidence=0.5,min_tracking_confidence=0.5) as face_mesh:
    while not exit:
        blackFrame = np.zeros((1080,1920,3),dtype="uint8")
        rand_x=random.randint(0,1920)
        rand_y=random.randint(360,720)
        cv2.circle(blackFrame,(rand_x,rand_y),9,(0,255,0),3)
        cv2.putText(blackFrame,str(total_tests),(100,900), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
        cv2.imshow("circle",blackFrame)
        if(rand_x>1280):
            location_of_circle ="Right"
        elif(rand_x<640):
            location_of_circle="Left"
        else:
            location_of_circle="Center"

        while True:
           
            # We get a new frame from the webcam
            sucess, frame = webcam.read()
            #sucess2,blackFrame=webcam2.read()
        
        
        
            if not sucess:
                print("ignore empty frame")
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = face_mesh.process(frame)
            if results.multi_face_landmarks:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                frame.flags.writeable = True
                ih,iw,ic=frame.shape
                

                if results.multi_face_landmarks: #if a face was detected
                    for id, lm in enumerate(results.multi_face_landmarks[0].landmark):  #un normalising the results convert to actual coordinates 
                        lm.x = int(lm.x*iw)
                        lm.y = int(lm.y*ih)
                    gaze.refresh(frame,results)
                    frame = gaze.annotated_frame()
                    text = ""
                else:
                    continue  #pulling out of loop if no face detected

                #We send this frame to GazeTracking to analyze it
            
                if gaze.is_blinking():
                    text = "Blinking"
                    if last_action =="":  #first time running through loop 
                        last_action = "Blinking"
                        last_blink_time = time.time()
                        
                    elif last_action == "Blinking":
                        cur_time = time.time()
                        if cur_time - last_blink_time >=2:  # send no control signal as forward/backwards wont change mode
                            if system_mode == "Forward":
                                system_mode ="Backward"
                            else:   
                                system_mode = "Forward"
                            last_blink_time = time.time()
                    else:   #previously was a different action
                        last_action = "Blinking"
                        last_blink_time = time.time()
                        
                    
                elif gaze.is_right():
                    text = "Right"
                    if last_action =="":  #first time running through loop 
                        last_action = "Looking right"
                        last_right_time = time.time()
                    elif last_action == "Looking right": #previous action is the same
                        cur_time = time.time()
                        if cur_time - last_right_time >=1:  # send control signal here
                            if system_status=="Active":
                                if system_mode == "Forward":
                                
                                    print("Moving: right")
                                else:
                                
                                    print("Moving: back right")
                            last_right_time = time.time()
                            
                    else:   #previously was a different action
                        last_action = "Looking right"
                        last_right_time = time.time()
                       
                    
                    
                elif gaze.is_left():
                    text = "Left"
                
                    if last_action =="":  #first time running through loop 
                        last_action = "Looking left"
                        last_left_time = time.time()
                       
                    elif last_action == "Looking left": #previous action is the same
                        cur_time = time.time()
                        if cur_time - last_left_time >=1:  # send control signal here
                            if system_status=="Active":
                                if system_mode == "Forward":
                                
                                    print("Moving: left")
                                else:
                                
                                    print("Moving: back left")
                            last_left_time = time.time()
                    else:   #previously was a different action
                        last_action = "Looking left"
                        last_left_time = time.time()
                        
                    

                elif gaze.is_center():
                    text = "Center"
                
                    if last_action =="":  #first time running through loop 
                        last_action = "Looking centre"
                        last_centre_time = time.time()
                       
                    elif last_action == "Looking centre": #previous action is the same
                        cur_time = time.time()
                        if cur_time - last_centre_time >=1:  # send signal here
                            if system_status=="Active":
                            
                                if system_mode == "Forward":
                                    print("Moving: Forward")
                                
                                else:
                                    print("Moving: Backward")
                                    
                                last_centre_time = time.time()
                    else:   #previously was a different action
                        last_action = "Looking centre"
                        last_centre_time = time.time()
                       
                    
                cv2.putText(frame, last_action, (90, 60), cv2.FONT_HERSHEY_DUPLEX, 1.6, (147, 58, 31), 2)
                cv2.putText(frame, system_status, (90, 205), cv2.FONT_HERSHEY_DUPLEX,  0.9, (147, 58, 31), 1)
                cv2.putText(frame, system_mode, (90, 240), cv2.FONT_HERSHEY_DUPLEX,  0.9, (147, 58, 31), 1)
                cv2.putText(frame, str(gaze.horizontal_ratio()), (90, 300), cv2.FONT_HERSHEY_DUPLEX, 1.6, (147, 58, 31), 2)
            
                
                


                left_pupil = gaze.pupil_left_coords()
                right_pupil = gaze.pupil_right_coords()
                cv2.putText(frame, "Left pupil:  " + str(left_pupil), (90, 130), cv2.FONT_HERSHEY_DUPLEX, 0.9, (147, 58, 31), 1)
                cv2.putText(frame, "Right pupil: " + str(right_pupil), (90, 165), cv2.FONT_HERSHEY_DUPLEX, 0.9, (147, 58, 31), 1)
            
                cv2.imshow("Demo", frame)
            
        

            if cv2.waitKey(1)& 0xFF==ord('q'):
                total_tests=total_tests+1
                if(text==location_of_circle):
                    total_correct=total_correct+1
                writer.writerow([location_of_circle,str(rand_x),text])
                break
            if keyboard.is_pressed('d'):
                exit=True
                break
webcam.release()
#webcam2.release()
cv2.destroyAllWindows()
writer.writerow(['Total number of tests',str(total_tests)])
writer.writerow(['Total number of correct detections',str(total_correct)])
f.close()
