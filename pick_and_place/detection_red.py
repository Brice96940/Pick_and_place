#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2 as cv
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import numpy as np
       
            
        

class Image_traitment(Node):
    def __init__(self):
        super().__init__('Image_traitment')
        self.logger = self.get_logger()
        self.logger.info("Initializing Capture image and executing motion...")
        self.subscription = self.create_subscription(Image, '/head_front_camera/image', self.listener_callback,10)
        self.subscript= self.create_subscription(Image,'/head_front_camera/depth_image',self.listener1_callback,10)
        self.publisher = self.create_publisher(JointTrajectory, 'head_controller/joint_trajectory', 10)
        self.pub = self.create_publisher(Twist,"key_vel", 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        self.red = [0,0,255]
        self.color_info = (0, 255, 255)
        self.subscription
    
    
    def get_limits(self,color):

        c =  np.uint8([[color]])
        hsvc = cv.cvtColor(c, cv.COLOR_BGR2HSV)

        lowerLimit = hsvc[0][0][0] -10, 10, 10
        UpperLimit = hsvc[0][0][0] +10, 255, 255

        if lowerLimit[0] < 0:
            lowerLimit = hsvc[0][0][0] -0, 20, 20

        #print(lowerLimit,UpperLimit)

        lowerLimit = np.array(lowerLimit, dtype= np.uint8)
        UpperLimit = np.array(UpperLimit, dtype= np.uint8)

        return lowerLimit, UpperLimit


    def Opencv_red(self, frame):
        
        #charger une image en mode hsv
        image_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        #cv.inRange(image_hsv, lower_couleur, upper_couleur)
        #creer une image binaire en fonction de la couleur choisie
        Lower_red,upper_red = self.get_limits(self.red)

        #Lower_red,upper_red = np.array([30,20,20]) , np.array([100,255,255])
        #Lower_red,upper_red = np.array([0,20,20]) , np.array([10,255,255])
        maskred = cv.inRange(image_hsv ,Lower_red ,upper_red )
        
        
        if Lower_red[0] == 0 :
            Lower_red1 = np.array([168,20,20])
            upper_red1 = np.array([180,255,255])
            masked1 =cv.inRange(image_hsv ,Lower_red1 ,upper_red1 )
        else:
            masked1 = maskred

        kernel = np.ones((3,3), np.uint8)


        #eroder l'image
        #binary_image = cv.erode(maskred,kernel,iterations = 1)

        #optimisation de la binarisation
        #ret,binnary_image = cv.threshold(maskred, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        #recuperer la couleur rouge 
        result_red = cv.bitwise_or(maskred ,masked1)

        #ouverture 
        binary_image = cv.morphologyEx(result_red , cv.MORPH_OPEN,kernel)

        #determination des contours 
        element = cv.findContours(binary_image,cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[-2]

        #determination du barycentre

        if len(element) > 0:
            c = max(element, key = cv.contourArea)
            ((x,y), rayon) = cv.minEnclosingCircle(c)

            if rayon > 10:
                cv.circle(binary_image, (int(x),int(y)), int(rayon), self.color_info, 2)
                #print(x,y)
                cv.circle(frame, (int(x),int(y)), 5, self.color_info, 10)
                cv.line(frame, (int(x),int(y)), (int(x)+150,int(y)), self.color_info, 2)
                cv.putText (frame, "objet rouge",(int(x)+10, int(y)-10), cv.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv.LINE_AA)

                #parametre du deplacement de la tete
                jtg = JointTrajectory()
                jtg.joint_names = ['head_1_joint','head_2_joint']
                jtpg = JointTrajectoryPoint()
                jtpg.positions = [0.1, 0.0]
                jtg.points.append(jtpg)

                jtd = JointTrajectory()
                jtpd = JointTrajectoryPoint()
                jtd.joint_names = ['head_1_joint','head_2_joint']
                jtpd.positions= [-0.1, 0.0]
                jtd.points.append(jtpd)

                #parametre du deplacement du corps
                Kvg = Twist()
                Kvg.angular.z=0.3

                Kvd = Twist()
                Kvd.angular.z=-0.3


                print(y)
                if x <= 300:
                    print ("gauche")
                    #self.publisher.publish(jtg)
                    self.pub.publish(Kvg)
                    print(jtg)
                elif x >= 400:
                    print("Droite")
                    #self.publisher.publish(jtd)
                    self.pub.publish(Kvd)
                    print(jtd)
                else:
                    print("center")


        # Display the resulting frame
        cv.imshow('binary_image', binary_image )
        cv.imshow('frame', frame)
        cv.waitKey(1)



    def transform_image_ros_to_opencv_img(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
        return cv_image
    

    def pose_of_color_red(self,msg):
        frame = self.transform_image_ros_to_opencv_img(msg)
        self.Opencv_red(frame)

    
    def find_depth(self,frame_depth):
        #frame= self.transform_image_ros_to_opencv_img(frame_depth)
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(frame_depth, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32)
        cv.imshow('pic', depth_array )
        cv.waitKey(0)



    def listener_callback(self,msg):
        #self.get_logger().info('I heard: "%s"' %msg.data)
        self.pose_of_color_red(msg)
        #self.trackbar()
    
    def listener1_callback(self,msg):
        #self.get_logger().info('I heard: "%s"' %msg.data)
        #self.find_depth(msg)
        print("reussie")






def main():
    rclpy.init()
    node = Image_traitment()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
