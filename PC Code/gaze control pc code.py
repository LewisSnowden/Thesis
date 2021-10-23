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
import socket
from playsound import playsound
import socket,cv2, pickle,struct,imutils,threading
import ntplib
IP_ADDRESS = '10.1.1.34'

def camera_capture_thead_function():
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    global current_frame
    client_socket.connect((IP_ADDRESS,8001))
    payload_size = struct.calcsize("Q")
    data = b""
    while(not(exit)):   
        while len(data) < payload_size:
            packet = client_socket.recv(4*1024) # 4K
            if not packet: break
            data+=packet
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("Q",packed_msg_size)[0]
            
        while len(data) < msg_size:
            data += client_socket.recv(4*1024)

        frame_data = data[:msg_size]
        data  = data[msg_size:]
        frame = pickle.loads(frame_data)
        frame = imutils.resize(frame,width=1920)
        current_frame=frame


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

client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket.connect((IP_ADDRESS,8000))
x = threading.Thread(target=camera_capture_thead_function)
x.start()
current_frame = np.zeros((1080,1920,3),dtype="uint8")
#ntpc=ntplib.NTPClient()
#resp = ""

with mp_face_mesh.FaceMesh(min_detection_confidence=0.5,min_tracking_confidence=0.5) as face_mesh:
    while True:
        # We get a new frame from the webcam
        sucess, frame = webcam.read()
        blackFrame=current_frame
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
        

            if mouth_open(results.multi_face_landmarks[0])==1:  #maybe immediatly send command maybe not 
                text= "Mouth Open"
                cv2.rectangle(blackFrame,(495,270),(1485,810),(0,0,255),3)
                cv2.putText(blackFrame,"Mouth Open",(900,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                if(system_status=="Active"):
                    cv2.putText(blackFrame,"Active->Standby",(900,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                else:
                    cv2.putText(blackFrame,"Standby->Active",(900,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
               

                if last_action =="":  #first time running through loop 
                    last_action = "Mouth Open"
                    last_mouth_time = time.time()
                    #playsound('mouth-open.mp3',False)
                elif last_action == "Mouth Open":
                    cur_time = time.time()
                    if cur_time - last_mouth_time >=2:  
                        if system_status == "Active":
                            system_status = "Standby"
                        else:
                            system_status = "Active"
                        last_mouth_time = time.time()
                else:   #previously was a different action
                    last_action = "Mouth Open"
                    last_mouth_time = time.time()
                   # playsound('mouth-open.mp3',False)

            elif gaze.is_blinking():
                text = "Blinking"
                cv2.rectangle(blackFrame,(495,270),(1485,810),(0,0,255),3)
                cv2.putText(blackFrame,"Blinking",(900,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                if(system_mode=="Forward"):
                    cv2.putText(blackFrame,"Forward->Backward",(900,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                else:
                    cv2.putText(blackFrame,"Backward->Forward",(900,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
               

                if last_action =="":  #first time running through loop 
                    last_action = "Blinking"
                    last_blink_time = time.time()
                    #playsound('blinking.mp3',False)
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
                    #playsound('blinking.mp3',False)
                
            elif gaze.is_right():
                text = "Looking right"
                if last_action =="":  #first time running through loop 
                    last_action = "Looking right"
                    last_right_time = time.time()
                    #playsound('right.mp3',False)
                elif last_action == "Looking right": #previous action is the same
                    cur_time = time.time()
                   
                    if cur_time - last_right_time >=1:  # send control signal here
                        if system_status=="Active":
                            #resp=ntpc.request('pool.ntp.org')
                            #time = responce.tx_time-response.delay
                            
                            if system_mode == "Forward":
                                client_socket.send(bytes("right:1","utf-8"))
                                print("Moving: right")
                            else:
                                client_socket.send(bytes("backward right:1","utf-8"))
                                print("Moving: back right")
                        last_right_time = time.time()
                        
                else:   #previously was a different action
                    last_action = "Looking right"
                    last_right_time = time.time()
                    #playsound('right.mp3',False)
               
                if(system_status=="Standby"):  #setting up ui boxes and text 
                    cv2.rectangle(blackFrame,(1280,0),(1980,1080),(255,255,0),3)
                    cv2.putText(blackFrame,"Right",(1630,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                    cv2.putText(blackFrame,system_status,(1630,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                    cv2.putText(blackFrame,system_mode,(1630,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                else:
                    if((system_status=="Active")and(system_mode=="Forward")):
                        cv2.rectangle(blackFrame,(1280,0),(1980,1080),(0,255,0),3)
                        cv2.putText(blackFrame,"Right",(1630,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                        cv2.putText(blackFrame,system_status,(1630,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                        cv2.putText(blackFrame,system_mode,(1630,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                    elif((system_status=="Active")and(system_mode=="Backward")):
                        cv2.rectangle(blackFrame,(1280,0),(1980,1080),(0,0,255),3)
                        cv2.putText(blackFrame,"Right",(1630,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)
                        cv2.putText(blackFrame,system_status,(1630,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)
                        cv2.putText(blackFrame,system_mode,(1630,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)

               
                
                
            elif gaze.is_left():
                if last_action =="":  #first time running through loop 
                    last_action = "Looking left"
                    last_left_time = time.time()
                    #playsound('left.mp3',False)
                elif last_action == "Looking left": #previous action is the same
                    cur_time = time.time()
                   
                    if cur_time - last_left_time >=1:  # send control signal here
                        if system_status=="Active":
                           # resp=ntpc.request('pool.ntp.org')
                            
                            if system_mode == "Forward":
                                client_socket.send(bytes("left:1","utf-8"))
                                print("Moving: left")
                            else:
                                client_socket.send(bytes("backward left:1","utf-8"))
                                print("Moving: back left")
                        last_left_time = time.time()
                else:   #previously was a different action
                    last_action = "Looking left"
                    last_left_time = time.time()
                    #playsound('left.mp3',False)
               
                if(system_status=="Standby"):  #setting up ui boxes and text 
                    cv2.rectangle(blackFrame,(0,0),(640,1080),(255,255,0),3)
                    cv2.putText(blackFrame,"Left",(320,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                    cv2.putText(blackFrame,system_status,(320,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                    cv2.putText(blackFrame,system_mode,(320,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                else:
                    if((system_status=="Active")and(system_mode=="Forward")):
                        cv2.rectangle(blackFrame,(0,0),(640,1080),(0,255,0),3)
                        cv2.putText(blackFrame,"Left",(320,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                        cv2.putText(blackFrame,system_status,(320,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                        cv2.putText(blackFrame,system_mode,(320,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                    elif((system_status=="Active")and(system_mode=="Backward")):
                        cv2.rectangle(blackFrame,(0,0),(640,1080),(0,0,255),3)
                        cv2.putText(blackFrame,"Left",(320,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)
                        cv2.putText(blackFrame,system_status,(320,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)
                        cv2.putText(blackFrame,system_mode,(320,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)
                text = "Looking left"
               
                
                

            elif gaze.is_center():
                text = "Looking centre"
                if last_action =="":  #first time running through loop 
                    last_action = "Looking centre"
                    last_centre_time = time.time()
                    #playsound('center.mp3',False)
                elif last_action == "Looking centre": #previous action is the same
                    cur_time = time.time()
                   
                    if cur_time - last_centre_time >=1:  # send signal here
                        if system_status=="Active":
                            #resp=ntpc.request('pool.ntp.org')
                            
                            if system_mode == "Forward":
                                print("Moving: Forward")
                                client_socket.send(bytes("forward:1","utf-8"))
                            else:
                                print("Moving: Backward")
                                client_socket.send(bytes("backward:1","utf-8"))
                            last_centre_time = time.time()
                else:   #previously was a different action
                    last_action = "Looking centre"
                    last_centre_time = time.time()
                    #playsound('center.mp3',False)
              
                if(system_status=="Standby"):  #setting up ui boxes and text 
                    cv2.rectangle(blackFrame,(640,0),(1280,1080),(255,255,0),3)
                    cv2.putText(blackFrame,"Center",(960,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                    cv2.putText(blackFrame,system_status,(960,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                    cv2.putText(blackFrame,system_mode,(960,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(255,255,0),3)
                else:
                    if((system_status=="Active")and(system_mode=="Forward")):
                        cv2.rectangle(blackFrame,(640,0),(1280,1080),(0,255,0),3)
                        cv2.putText(blackFrame,"Center",(960,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                        cv2.putText(blackFrame,system_status,(960,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                        cv2.putText(blackFrame,system_mode,(960,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                    elif((system_status=="Active")and(system_mode=="Backward")):
                        cv2.rectangle(blackFrame,(640,0),(1280,1080),(0,0,255),3)
                        cv2.putText(blackFrame,"Center",(960,540), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)
                        cv2.putText(blackFrame,system_status,(960,580), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)
                        cv2.putText(blackFrame,system_mode,(960,620), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,0,255),3)

               
                
                
            cv2.putText(frame, last_action, (90, 60), cv2.FONT_HERSHEY_DUPLEX, 1.6, (147, 58, 31), 2)
            cv2.putText(frame, system_status, (90, 205), cv2.FONT_HERSHEY_DUPLEX,  0.9, (147, 58, 31), 1)
            cv2.putText(frame, system_mode, (90, 240), cv2.FONT_HERSHEY_DUPLEX,  0.9, (147, 58, 31), 1)
            cv2.putText(frame, str(gaze.horizontal_ratio()), (90, 300), cv2.FONT_HERSHEY_DUPLEX, 1.6, (147, 58, 31), 2)
           
            
            


            left_pupil = gaze.pupil_left_coords()
            right_pupil = gaze.pupil_right_coords()
            cv2.putText(frame, "Left pupil:  " + str(left_pupil), (90, 130), cv2.FONT_HERSHEY_DUPLEX, 0.9, (147, 58, 31), 1)
            cv2.putText(frame, "Right pupil: " + str(right_pupil), (90, 165), cv2.FONT_HERSHEY_DUPLEX, 0.9, (147, 58, 31), 1)
        
            cv2.imshow("Demo", frame)
            cv2.imshow("New Frame",blackFrame)
       

        if cv2.waitKey(1)& 0xFF==ord('q'):
            exit=True
            break
client_socket.close()
webcam.release()
#webcam2.release()
cv2.destroyAllWindows()