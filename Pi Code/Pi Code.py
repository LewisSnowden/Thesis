import RPi.GPIO as GPIO
import time
import io
import socket
from time import sleep
import cv2, pickle,struct,imutils,threading,ntplib

#set GPIO direction (IN / OUT)
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER=18
GPIO_ECHO=24
in1=20
in2=21
in3=19
in4=26

GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
GPIO.setup(GPIO_ECHO,GPIO.IN)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)

GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
ntpc=ntplib.NTPClient()
resp = ""

def image_capture_thread():
    thread_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    thread_socket.bind(('10.1.1.34',8001))
    thread_socket.listen(1)
    while True:
        capturesocket, address = thread_socket.accept()
        exit_for_thread=False
        try:
            image_capture_process(capturesocket)
            capturesocket.close()
        except(ConnectionResetError,BrokenPipeError):
            print("client disconnected early image capture")
            pass
            
        
def image_capture_process(client_socket):
    if client_socket:
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FPS, 30.0)
        camera.set(cv2.CAP_PROP_BUFFERSIZE,3)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if camera.isOpened():
            while(not(exit_for_thread)):
                img,frame = camera.read()
                frame = imutils.resize(frame,width=320)
                a = pickle.dumps(frame)
                message = struct.pack("Q",len(a))+a
                client_socket.sendall(message)
                        
    
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

def forward():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    sleep(0.5)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    
def backward():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    sleep(0.5)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)    
    
def right():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    sleep(0.075)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    
def left():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    sleep(0.075)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    
        
def time_capture(msg_time,num_commands,total_time):
    resp = ntpc.request('pool.ntp.org')
    cur_time=resp.tx_time-resp.delay
    time_diff=cur_time-msg_time
    num_commands = num_commands +1
    total_time = total_time+time_diff
    ave_time = total_time/num_commands
    print("ave time: "+str(ave_time)+"num commands: "+str(num_commands))
    return num_commands,total_time
    
def process(clientsocket):
    msg = ""
    msg_command=""
    total_time = 0
    num_commands = 0
    time_diff=0
    while True:
        msg = clientsocket.recv(1024)
        if not msg:
            break
        msg_decoded = msg.decode("utf-8")
        msg_split = msg_decoded.split(":")
        msg_command = msg_split[0]
        if(len(msg_split)==2):
            msg_time= float(msg_split[1])
        else:
            msg_time=0.0
        if msg_command=="forward":
            
            #num_commands,total_time=time_capture(msg_time,num_commands,total_time)
            dist = distance()
            #print ("Measured Distance = %.1f cm" % dist)
            if dist>45.0:
               forward()
            else:
                print("obstruction detect")
          
        elif msg_command=="right":
            #num_commands,total_time=time_capture(msg_time,num_commands,total_time)
            right()
        
        elif msg_command=="left":
            #num_commands,total_time=time_capture(msg_time,num_commands,total_time)
            left()
          
        elif msg_command=="backward":
            #num_commands,total_time=time_capture(msg_time,num_commands,total_time)
            backward()
          
        elif msg_command=="backward left":
            #num_commands,total_time=time_capture(msg_time,num_commands,total_time)
            right()
            
        elif msg_command=="backward right":
            #num_commands,total_time=time_capture(msg_time,num_commands,total_time)
            left()
        
       
          
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    
serv_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
serv_socket.bind(('10.1.1.34',8000))
serv_socket.listen(1)
global exit_for_thread
exit_for_thread = False
x = threading.Thread(target=image_capture_thread)
x.start()
while True:
    clientsocket, address = serv_socket.accept()
    try:
        
        process(clientsocket)
        exit_for_thread = True
        clientsocket.close()
    except(ConnectionResetError):
        print("client disconnected early")
        pass
     
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    

     