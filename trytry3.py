#!/usr/bin/env python

"""main_feeding.py: Main executable file for auto robotic feeding."""

__author__      = "Edwin Foo"
__copyright__   = "Copyright 2019, NYP"
__credits__ = ["Liang HuiHui", "Edwin Foo", "Wong Yau"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Edwin Foo"
__email__ = "edwin_foo@nyp.edu.sg"
__status__ = "Beta"
#
# import herkulexpy3v1 as herkulex
import numpy as np
import dlib
from scipy.spatial import distance as dist
import time
import math
import cv2
from openni import openni2
from openni import _openni2 as c_api
from class44_mouth import astras
#from class_motor import*
# from test_motorv3v1 import*
from math import *
from threading import Thread
import sys
from select import select
from imutils import face_utils
#######################
# wiringPi
#######################
# import wiringpi as wpi
import time
 
#wpi.wiringPiSetup()
#wpi.pinMode(0, 1)
#wpi.pinMode(1, 1)
######
# variable decalration
########
home_angle = [0.0, 0.0, 0.0, 0.0]
K = [6.5308391993466671e+002, 0.0, 3.1950000000000000e+002,
     0.0, 6.5308391993466671e+002, 2.3950000000000000e+002,
     0.0, 0.0, 1.0]
D = [7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000]

cam_matrix = np.array(K).reshape(3, 3).astype(np.float32)
dist_coeffs = np.array(D).reshape(5, 1).astype(np.float32)

object_pts = np.float32([[6.825897, 6.760612, 4.402142],
                         [1.330353, 7.122144, 6.903745],
                         [-1.330353, 7.122144, 6.903745],
                         [-6.825897, 6.760612, 4.402142],
                         [5.311432, 5.485328, 3.987654],
                         [1.789930, 5.393625, 4.413414],
                         [-1.789930, 5.393625, 4.413414],
                         [-5.311432, 5.485328, 3.987654],
                         [2.005628, 1.409845, 6.165652],
                         [-2.005628, 1.409845, 6.165652],
                         [2.774015, -2.080775, 5.048531],
                         [-2.774015, -2.080775, 5.048531],
                         [0.000000, -3.116408, 6.097667],
                         [0.000000, -7.415691, 4.070434]])
#object_pts = np.float32([[6.825897, 6.760612, 4.402142],
                       #  [-1.330353, 7.122144, 6.903745],
                       #  [-6.825897, 6.760612, 4.402142],
                       #  [5.311432, 5.485328, 3.987654],
                       #  [1.789930, 5.393625, 4.413414],
                        # [2.005628, 1.409845, 6.165652],
                        # [2.774015, -2.080775, 5.048531],
                        # [0.000000, -3.116408, 6.097667],
                        # [0.000000, -7.415691, 4.070434]])

reprojectsrc = np.float32([[10.0, 10.0, 10.0],
                           [10.0, 10.0, -10.0],
                           [10.0, -10.0, -10.0],
                           [10.0, -10.0, 10.0],
                           [-10.0, 10.0, 10.0],
                           [-10.0, 10.0, -10.0],
                           [-10.0, -10.0, -10.0],
                           [-10.0, -10.0, 10.0]])

line_pairs = [[0, 1], [1, 2], [2, 3], [3, 0],
              [4, 5], [5, 6], [6, 7], [7, 4],
              [0, 4], [1, 5], [2, 6], [3, 7]]
#cap = cv2.VideoCapture(0)
#prediction ='shape_predictor_68_face_landmarks.dat'
#detector = dlib.get_frontal_face_detector()
#predictor = dlib.shape_predictor(prediction)
#complete=0


    ##########
def kbhit():
	''' Returns True if keyboard character was hit, False otherwise.
	'''
#        if os.name == 'nt':
#            return msvcrt.kbhit()

#        else:
	[dr,dw,de] = select([sys.stdin], [], [],0)
	return dr != []

def getch():
    ''' Returns a keyboard character after kbhit() has been called.
        Should not be called in the same program as getarrow().
    '''

    s = ''

#        if os.name == 'nt':
#            return msvcrt.getch().decode('utf-8')

#        else:
    return sys.stdin.read(1)
    ##########
""" setup() """
class Rotate_converter:
    #rotate theta 180 anticlock wise then rotate psi clockwise 90 degrees
    # xcos(theta) + zsin(theta)
    # ycos(psi) + [-sin(psi)][-xsin(theta) + z(cos(theta)]
    # ysin(psi) + [cos(psi)][-xsin(theta) + z(cos(theta)]

    def convert(self,coOrdinateX,coOrdinateY,coOrdinateZ):
        R_co_OrdintateX = -coOrdinateX
        R_co_OrdintateY = coOrdinateZ + 60
        R_co_OrdintateZ = coOrdinateY + 510
        
        return R_co_OrdintateX,R_co_OrdintateY,R_co_OrdintateZ
        
class MyThreadmotor(Thread):
    def __init__(self):

        #Thread.__init__(self)#initialize thread for motor
        
#    def threadstart(self):### motor tread start
        self.servos = servo() # intialize the servo
        self.pos_M = [0]*3

        #print('motor',self.pos_M)

###connect to serial port##
        herkulex.connect("/dev/ttyUSB0",115200)
        for motorID in motor : 
            print(motorID)
            herkulex.clear_error(motorID)
            herkulex.torque_on(motorID)    

        #input("Press anything to continue:")
        print("Cal postion A")
        self.servos.CalPos_A(home_angle)
            #
        old_pos = pos_A
            #print("old_pos",old_pos,"pos_A",pos_A)
            #input()
        pTime = homing_time
        goaltime = int(float(pTime / 11.2))
        print("home angle",home_angle,"pTime",pTime,"goalTime",goaltime)
        if goaltime >=255:
           goaltime = 255
        self.servos.moveMotors(home_angle, goaltime) # move the arm into home position. the angles are in degree.
        #input("Press anything to continue:")
            # read_angle()
        print("Cal postion B")
        self.servos.CalPos_B()  #determine the coordinates of the starting position (wrist coordinates) for the scooping action
            #input()
        print("PosB",pos_B)
        print("Initialisation complete")
        self.servos.moveTo(self.servos.pos_B, scoop_flag)
        #time.sleep(0.5)
        #print("Wait for 0.5s")
        #self.servos.read_angle()
        #input("Press anything to continue:")

#run the motor
    def run(self):
#            self.pos_M = [mouthPos[0],mouthPos[1],mouthPos[2]] #-66, -185, 400.
            #pTime = homing_time
#        while cont.upper() == 'Y':
            #self.servos.scoop()
            #time.sleep(5.0)
            #self.servos.read_angle()
            #self.servos.moveTo(self.pos_M, self.servos.scoop_flag)
            #time.sleep(5.0)
            #self.servos.read_angle()
            self.servos.CalPos_B()  # determine the coordinates of the starting position (wrist coordinates) for the scooping action
            self.servos.moveTo(self.servos.pos_B, self.servos.scoop_flag)
            #time.sleep(5.0)
            #self.servos.read_angle() 

            reach_flag=self.servos.checkReach(self.pos_M, self.servos.arm_attr)
            #print(self.pos_M,self.servos.pos_M)
            if (reach_flag):
           
                self.servos.scoop()
                self.servos.moveTo(self.pos_M,self.servos.scoop_flag)
                #time.sleep(5)
                self.mouth_status_1 = 0
                
                while self.mouth_status_1 is 0 :
                    capture = cv2.VideoCapture(0)
                    A1= time.time()
                    _,feeding = capture.read()
                    prediction ='best6_predictor.dat'
                    print('...')
                    detector = dlib.get_frontal_face_detector()
                    print('ok')
                    prediction = dlib.shape_predictor(prediction)   
                    print('PREDICTED')
                    #_, frame = cap.read()
                    feeding2= cv2.resize(feeding,(320,200))
                    rgb = cv2.cvtColor(feeding2, cv2.COLOR_BGR2GRAY)
                    faces = detector(rgb)
                    print('waiting')
                    for face in faces:
                        landmarks = prediction(rgb, face)
                        x2 =  landmarks.part(15).y 
                                
                        x3 =  landmarks.part(16).y
                        B =dist.euclidean(x2,x3)
                    if B >13:
                        A2=time.time()
                        print('time taken is' , A2-A1)
                        #self.mouth_status_1=1
                        capture.release()
                        break
                    
                    self.mouth_status_1=0
                    capture.release()
                    print('again')

                self.servos.CalPos_B()
                self.servos.moveTo(self.servos.pos_B, self.servos.scoop_flag)
                print('reached destination')
                    
                 
                        
                
                        
                    #self.servos.CalPos_B()
                    #self.servos.moveTo(self.servos.pos_B, self.servos.scoop_flag)
                    #print('reached destination')
                #else:
                    #print('still waiting')
            else:
                print("Re-enter new coordinates again")
#               cont=input("Press anything to Y to continue:")
    def pass_value(self,mouthPos):
         self.pos_M = [mouthPos[0],mouthPos[1],mouthPos[2]]
         print('pos m', self.pos_M)

# will stop the motor
    def stop(self):
        print('stop')
        # for motorID in motor :
        #  #print(motorID)
        #     herkulex.set_led(motorID,herkulex.LED_OFF)
        #     herkulex.clear_error(motorID)
        #     herkulex.torque_off(motorID)
        # herkulex.close()

            

class camera():
    def __init__(self):
        self.ccor=[-66,-185,400]

        Thread.__init__(self) #initialize thread for 3d camera
        #convert
        self.Rotate_convert = Rotate_converter()
        #self.file1 = open("Dmap.txt","w") 
    def run(self):
        print('start')
        self.cam_astras = astras()
        print('cam not captured')
        A=time.time()
     
        cap = cv2.VideoCapture(0)
        _, frame = cap.read()
        k=time.time()
        detector = dlib.get_frontal_face_detector()
        predictor = dlib.shape_predictor('best6_predictor.dat')
        frame = cv2.resize(frame, (640, 400))
        c=time.time()
        print('my_predictor load time is', c-A)
      
        self.my_head = False
        if self.my_head is False:
         
            face_rects = detector(frame, 0)
            if len(face_rects) > 0:
                    shape = predictor(frame, face_rects[0])
                    shape = face_utils.shape_to_np(shape)
                    image_pts = np.float32([shape[1], shape[2], shape[3], shape[4], shape[7],
                                shape[8], shape[9], shape[10], shape[5], shape[6],
                                shape[11], shape[13], shape[14], shape[0]])
                  
                    _, rotation_vec, translation_vec = cv2.solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs)

                    reprojectdst, _ = cv2.projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix,
                                            dist_coeffs)

                    reprojectdst = tuple(map(tuple, reprojectdst.reshape(8, 2)))
                    t=time.time()
                    print('time taken now is' , t-A)
                    rotation_mat, _ = cv2.Rodrigues(rotation_vec)
                    pose_mat = cv2.hconcat((rotation_mat, translation_vec))
                    _, _, _, _, _, _, euler_angle = cv2.decomposeProjectionMatrix(pose_mat)
                    y_value = float(str("{:7.2f}".format(euler_angle[1, 0])))
                    #x_value = float(str("{:7.2f}".format(euler_angle[0, 0])))
                    if y_value >= 15:
                            print("looking left y is",y_value)
                            self.my_head = False

                    elif y_value <= -15:
                            print("looking right y is", y_value)
                            self.my_head = False

                    elif y_value<15 and y_value>-15:
                                print("neutral position y is", y_value)
                                self.my_head = True
                                
                                
        l=time.time()
        print('final time is', l-A)
        #prediction ='face2_predictor.dat'
        print('...')
        #detector = dlib.get_frontal_face_detector()
        #print('ok')
       # prediction= dlib.shape_predictor(predictor)   
        print('PREDICTED')
        B=time.time()
        print('mouth detection is ', B-l)
        print('total delay is ', B-A)
        #_, frame = cap.read()
        frame= cv2.resize(frame,(640,400))
        
        #bgr = np.fromstring(frame, dtype=np.uint8).reshape(200,320)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.mouth_status = False
        faces = detector(rgb,0)
        
        
#        s = 0
#        done = False
#        while not done:
#            key = cv2.waitKey(1) & 255
             ## Read keystrokes
#            if key == 27:  # terminate
#                print ("\tESC key detected!")
#                done = True

#            elif key == ord("c"):
#                break

            ###rgb#####
#if mouth detection this (x,y) coordinate will transfer to depth
        try:
          for face in faces:
                print('detecting....')
                #a = face.left()
                #b = face.top()
                #c = face.right()
                #d = face.bottom()
                #cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                landmarks = predictor(rgb, face)
                
                #x2 =  landmarks.part(63).y 
                #y2 = landmarks.part(63).y
                #x3 =  landmarks.part(67).y
                #y3 = landmarks.part(67).y
                x4 = [landmarks.part(11).x , landmarks.part(11).y]
                #y4 = landmarks.part(49).y
                x5 = [landmarks.part(13).x , landmarks.part(13).y]
                print('landmarks')
                cap.release()
                #y5 = landmarks.part(65).y

                A=dist.euclidean(landmarks.part(15).y , landmarks.part(16).y)
                print('distance A',A)
                if A>10 and self.my_head is True :
                    print('mouth open')
                    self.mouth_status = True
                    #cap.release()
                    #cv2.destroyAllWindows()
                    print('cam closed')
                    
                else:
                    print('mouth closed')
                    self.mouth_status = False
          
          
               
            
############# take lanmark point shape 49 and 65 and dive 2 to find the mouth coordinate #####
          self.x1 =(x4[0]+x5[0]) // 2
           #print('x1',self.x1)  
          self.y1 =(x4[1]+x5[1]) // 2
           #print('y1',self.y1)
          self.distX =abs(x4[0]-x5[0])
           #print('distX',self.distX)
           

           
           #print("distX",self.distX)
#           self.y1 = (mouth1[1] + mouth2[1]) // 2
           #print('x1,y1',self.x1,self.y1,"mouth1",mouth1,"mouth2",mouth2)
# else mouth not detect use this coordinate to the depth
        except ValueError:
           #rgb,mouth1,mouth2 = self.cam_astras.get_rgb()
           self.x1 = 163
           self.y1 = 209
        
        #dmap, d4d = self.cam_astras.get_depth()
          # Mouth_status = 0
              # DEPTH
        #dmap, d4d = self.cam_astras.get_depth()
        #print('dmap',dmap[self.y1 ,self.x1],dmap[mouth1[1],mouth1[0]],dmap[mouth2[1],mouth2[0]])
        #if damp
        #for data in dmap:
        #  print("dmap",data)
        #print("Dmap dmap[self.x1:-5,self.y1:-5])",d4d[self.x1:self.x1-5,self.y1:self.y1-5])
        #save text file as numpy arrary
        #np.savetxt('dmap.txt', d4d, delimiter=',') 
        
        #time.sleep(2.0)
        #tmp = self.cam_astras.mouth_status
       

	#self.ccor = [-66,-185,400]  
        return(self.ccor)


    def endtread(self): 
    ##image  relise
        cv2.destroyAllWindows()#destory windown
        #self.cam_astras.rgb_stream.stop()#stop 3D rbg_stream
        #self.cam_astras.depth_stream.stop()#stop 3D depth stream
        openni2.unload()
        print ("camera closed")

  
def main():
 try:
    done = False
#    smotor = MyThreadmotor()
#    wpi.digitalWrite(0, 0)
#    wpi.digitalWrite(1, 1)
 #   while not done:
    while True: 
       #while not(kbhit()):
            print("Press Ctrl-C to stop")
            #key = getch()
            ## Read keystrokes
            #if ord(key) == 27:  # terminate
            #    print ("\tESC key detected!")
            #    done = True
            #    break
            d3camera = camera()

#            wpi.digitalWrite(0, 1)

            detect_pos = d3camera.run()
       # initialise the start
            d3camera.endtread()
            #d3camera.join() #join thread
            #loc=[38.0, 356.0, 224.0]
            #print("mouth location: ", detect_pos)
            #loc = detect_pos
#           loc=[0, 500, 460.0]
#           detect_pos = [0,300,300]
#           smotor = MyThreadmotor(d3camera.ccor)
#            smotor.pass_value(detect_pos)
#            wpi.digitalWrite(1, 1)
#            smotor.run()
#            wpi.digitalWrite(1, 0)
            '''smotor.pass_value(d3camera.ccor)
            smotor.start()# initialise the motor
            smotor.join()# join thread 
            '''
 except KeyboardInterrupt:
    input("Please get ready to hold motors")
 #   smotor.stop() #motor will stop enter press
   

if __name__== "__main__":
    main()


