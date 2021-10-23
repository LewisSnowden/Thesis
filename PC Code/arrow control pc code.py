"""
Demonstration of the GazeTracking library.
Check the README.md for complete documentation.
"""

import threading
import cv2
import time
import numpy as np
import socket
import keyboard
import pickle
import socket,cv2, pickle,struct,imutils
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
    




webcam = cv2.VideoCapture(0)
system_status = "Active"
system_mode = "Forward"
last_key=""
last_key_time=""
exit = False

client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket.connect((IP_ADDRESS,8000))
x = threading.Thread(target=camera_capture_thead_function)
x.start()
current_frame = np.zeros((1080,1920,3),dtype="uint8")

while True:
    # We get a new frame from the webcam
    #blackFrame = np.zeros((1080,1920,3),dtype="uint8")
    blackFrame=current_frame
    cur_time=time.time()
    
    if system_mode=="Forward":
        cv2.putText(blackFrame,"FORWARD",(90,200), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)      
    else:
        cv2.putText(blackFrame,"BACKWARD",(90,200), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)      
    try:
        if keyboard.is_pressed('d'):
            if last_key=='d':
                if cur_time-last_key_time>0.2:
                    cv2.putText(blackFrame,"RIGHT",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                    if system_status=="Active":
                        if system_mode == "Forward":
                            client_socket.send(bytes("right","utf-8"))
                            print("Moving: right")
                        else:
                            client_socket.send(bytes("backward right","utf-8"))
                            print("Moving: back right")
                    last_key_time=time.time()
            else:
                cv2.putText(blackFrame,"RIGHT",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                if system_status=="Active":
                    if system_mode == "Forward":
                        client_socket.send(bytes("right","utf-8"))
                        print("Moving: right")
                    else:
                        client_socket.send(bytes("backward right","utf-8"))
                        print("Moving: back right")
                last_key='d'
                last_key_time=time.time()
            
        elif keyboard.is_pressed('a'):
            if last_key=='a':
                if cur_time-last_key_time>0.2:
                    cv2.putText(blackFrame,"LEFT",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                    if system_status=="Active":
                        if system_mode == "Forward":
                            client_socket.send(bytes("left","utf-8"))
                            print("Moving: left")
                        else:
                            client_socket.send(bytes("backward left","utf-8"))
                            print("Moving: back left")
                    last_key_time=time.time()
            else:
                cv2.putText(blackFrame,"LEFT",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                if system_status=="Active":
                    if system_mode == "Forward":
                        client_socket.send(bytes("left","utf-8"))
                        print("Moving: left")
                    else:
                        client_socket.send(bytes("backward left","utf-8"))
                        print("Moving: back left")
                last_key_time=time.time()
                last_key='a'
        elif keyboard.is_pressed('w'):
            if last_key=='w':
                if cur_time-last_key_time>0.5:
                    cv2.putText(blackFrame,"FORWARD",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                    if system_status=="Active":
                       
                        client_socket.send(bytes("forward","utf-8"))
                        print("Moving: forward")
                    last_key_time=time.time()
            else:
                cv2.putText(blackFrame,"FORWARD",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                if system_status=="Active":
                   
                    client_socket.send(bytes("forward","utf-8"))
                    print("Moving: forward")
                last_key_time=time.time()
                last_key='w'
        elif keyboard.is_pressed('s'):
            if last_key=='s':
                if cur_time-last_key_time>0.5:
                    cv2.putText(blackFrame,"FORWARD",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                    if system_status=="Active":
                        
                        client_socket.send(bytes("backward","utf-8"))
                        print("Moving: left")
                    last_key_time=time.time()
            else:
                cv2.putText(blackFrame,"FORWARD",(90,400), cv2.FONT_HERSHEY_DUPLEX, 1.6,(0,255,0),3)
                if system_status=="Active":   
                    client_socket.send(bytes("backward","utf-8"))
                    print("Moving: backward")
                last_key_time=time.time()
                last_key='s'

        elif keyboard.is_pressed('e'):
            if last_key=='e':
                if cur_time-last_key_time>1:
                    if system_mode=="Backward":
                        system_mode="Forward"
                    else:
                        system_mode="Backward"
                    last_key_time=time.time()
            else:
                if system_mode=="Backward":
                    system_mode="Forward"
                else:
                    system_mode="Backward"
                last_key_time=time.time()
                last_key='e'
    except:
        continue

    if cv2.waitKey(1)& 0xFF==ord('q'):
        exit=True
        break

    cv2.imshow("New Frame",blackFrame)
   
      
client_socket.send(bytes("exit","utf-8"))
webcam.release()
cv2.destroyAllWindows()
x.join()