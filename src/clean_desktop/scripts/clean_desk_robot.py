#!/usr/bin/env python3
from ast import Not
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from zoo_bringup.msg import SingleServo, MultipleServo
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from math import pi
import math
import tf
import numpy as np

font = cv2.FONT_HERSHEY_SIMPLEX

class CleanDeskRobot:
    def __init__(self):
        # 初始化桌面清洁机器人节点
        rospy.init_node('clean_desk_robot_node', anonymous=True)
        # 颜色形状识别相关
        self.bridge = CvBridge()
        self.gray_frame = None
        self.hsv_frame = None
        self.color = 1
 
        cv2.namedWindow('img')
        cv2.createTrackbar("H_MIN","img",19,180,self.nothing)
        cv2.createTrackbar("H_MAX","img",41,180,self.nothing)
        cv2.createTrackbar("S_MIN","img",154,255,self.nothing)
        cv2.createTrackbar("S_MAX","img",221,255,self.nothing)
        cv2.createTrackbar("V_MIN","img",120,255,self.nothing)
        cv2.createTrackbar("V_MAX","img",161,255,self.nothing)
        cv2.setMouseCallback("img", self.mouse_click)

        # 单个舵机控制发布
        self.single_servo_pub = rospy.Publisher('/single_servo_topic', SingleServo, queue_size=5)
        # 多个舵机控制发布
        self.multiple_servo_pub = rospy.Publisher('/multiple_servo_topic', MultipleServo, queue_size=5)
        # 底盘控制节点发布
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # 发布导航任务
        self.slam_task_pub = rospy.Publisher('slam_task',String, queue_size = 10)
        # 里程计信息订阅
        self.odom_sub = rospy.Subscriber("/wheel_odom", Odometry, self.odom_callback)
        # 舵机控制回调
        self.servo_control_sub = rospy.Subscriber("/servo_controller", String, self.callback)
        # 导航结果回调
        self.slam_result_sub = rospy.Subscriber("/slam_result", String, self.slam_callback)
        # 摄像头订阅
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # 是否已经识别对准
        self.isDetect = True
        # 是否投放成功is
        self.upSUCCEEDED = False
        # 里程计信息
        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0
        # 桌面抓取任务状态 0=空闲中 1=任务执行中
        self.clean_status = 0
        # 底盘msg
        self.twist = Twist()
        # upcount
        self.UpCount = 0

        rospy.sleep(10)
        # self.slam_task_pub.publish("desk_point1")  

    # 鼠标点击事件
    def mouse_click(self, event, x, y, flags, para):
        if event == cv2.EVENT_LBUTTONDOWN:
            print ('PIX: ', x, y)
            print ('GRAY: ', self.gray_frame[y, x])
            print ('HSV: ', self.hsv_frame[y, x])
    def nothing(self, x):
        pass
    
    # 摄像头数据回调+颜色形状识别
    def image_callback(self,data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        src_frame = img
        
        if self.color == 0:
            # 定义黄色的HSV范围
            lower_color = np.array([20, 154, 120])
            upper_color = np.array([41, 221, 161])
        elif self.color == 1:
            # 定义红色的HSV范围
            lower_color1 = np.array([0, 120, 100])
            upper_color1 = np.array([15, 170, 150])
            lower_color2 = np.array([156, 133, 131])
            upper_color2 = np.array([180, 255, 255])
        
        self.gray_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2GRAY)
        self.hsv_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2HSV)
         
        if self.color == 0:
            # 获取黄色掩膜
            mask_color = cv2.inRange(self.hsv_frame, lower_color, upper_color)
        elif self.color == 1:
            # 获取红色掩膜
            mask1 = cv2.inRange(self.hsv_frame, lower_color1, upper_color1)
            mask2 = cv2.inRange(self.hsv_frame, lower_color2, upper_color2)
            mask_color = cv2.bitwise_or(mask1, mask2)
        
        # 形态学操作
        mask_color = cv2.medianBlur(mask_color, 7)
        s = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        opened = cv2.morphologyEx(mask_color, cv2.MORPH_OPEN, s, iterations=2)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, s, iterations=2)
        
        # 轮廓检测
        edges = cv2.Canny(opened, 50, 100)
        circles = cv2.HoughCircles(
            edges, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=10, minRadius=10, maxRadius=500)
        cv2.imshow("img", src_frame)
        cv2.imshow("edges", edges)
        if circles is not None:  # 如果识别出圆
            # print("find circles!!!")
            for circle in circles[0]:
                #  获取圆的坐标与半径
                x = int(circle[0])
                y = int(circle[1])
                r = int(circle[2])
                cv2.circle(src_frame, (x, y), r, (0, 0, 255), 3)  # 标记圆
                cv2.circle(src_frame, (x, y), 3, (255, 255, 0), -1)  # 标记圆心
                # x，y表示物料中心点在图像中的位置
                text = 'x:  '+str(x)+' y:  '+str(y)
                #print(text)
                # 识别成功后，计算机械臂需要转动的角度，这里识别成功一次后就不再计算，后续在slam_callback函数中直接进行抓取动作调用
                if self.isDetect is not True:
                    print("send servo")
                    single_servo1 = SingleServo()
                    single_servo1.ID = 1
                    single_servo1.Rotation_Speed = 50
                    if x > 320:
                        offset_x = (x - 320) * 0.08 
                        offset_angle = int(math.atan2(offset_x, 30) * 180 / pi)
                        single_servo1.Target_position_Angle = -offset_angle * 10
                        self.single_servo_pub.publish(single_servo1)
                    else:
                        offset_x = (320 - x) * 0.08 
                        offset_angle = int(math.atan2(offset_x, 30) * 180 / pi)
                        single_servo1.Target_position_Angle = offset_angle * 10
                        self.single_servo_pub.publish(single_servo1)
                    self.isDetect = True

                cv2.putText(src_frame, text, (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA, 0)  # 显示圆心位置

                # 检测是否投放成功
                if self.upSUCCEEDED == True:
                    self.upSUCCEEDED = False
                    print("upFailed!!!")
        else:
            # 如果识别不出，显示圆心不存在
            cv2.putText(src_frame, 'x: None y: None', (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA, 0)
        cv2.waitKey(1)
        
    # 接收slam导航状态回调
    def slam_callback(self,msg):
        flag = msg.data
        #self.isDetect = False
        #rospy.sleep(5)
            
        if flag == "desk_point1":
            print("start arm pick desk_point1")
            self.clean_status = 1
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.5)
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.5)
            while self.clean_status == 1:
                self.single_arm_init()
                rospy.sleep(0.5)
                self.single_arm_six_open()
                rospy.sleep(0.2)              
                self.isDetect = False
                rospy.sleep(1)
                self.isDetect = True
                rospy.sleep(0.5)
                self.single_arm_pick()
                rospy.sleep(0.5)
                self.clean_status = 0             
             
                # 发布下个点导航
                msg = String()
                msg.data = "rubbish_bin1"
                self.slam_task_pub.publish(msg)

                single_servo1 = SingleServo()
                single_servo1.ID = 1
                single_servo1.Rotation_Speed = 50
                single_servo1.Target_position_Angle = -900    
                self.single_servo_pub.publish(single_servo1)

        if flag == "rubbish_bin1":
            print("start throw rubbish 1")
            self.clean_status = 1
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.1)
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.1)
            while self.clean_status == 1:
                self.single_arm_up()
                rospy.sleep(3)
                self.clean_status = 0

                #检测是否投放成功
                self.upSUCCEEDED = 1
                single_servo1 = SingleServo()
                single_servo1.ID = 1
                single_servo1.Rotation_Speed = 50
                single_servo1.Target_position_Angle = -1600    
                self.single_servo_pub.publish(single_servo1)
                rospy.sleep(1.2)
                if self.upSUCCEEDED == False:        
                    self.UpCount = self.UpCount +1
                    print(self.UpCount)
                    if self.UpCount < 3:
                        # 回到第一个桌子
                        print("Go Back To desk1")
                        msg = String()
                        msg.data = "desk_point1"
                        self.slam_task_pub.publish(msg)
                    else:
                        # 发布下个点导航
                        self.upSUCCEEDED = 0 
                        self.UpCount = 0               
                        msg = String()
                        msg.data = "desk_point2"
                        self.slam_task_pub.publish(msg)
                else:
                    # 发布下个点导航
                    self.upSUCCEEDED = 0 
                    self.UpCount = 0               
                    msg = String()
                    msg.data = "desk_point2"
                    self.slam_task_pub.publish(msg)
                single_servo1 = SingleServo()
                single_servo1.ID = 1
                single_servo1.Rotation_Speed = 50
                single_servo1.Target_position_Angle = 0
                self.single_servo_pub.publish(single_servo1)   

        if flag == "desk_point2":
            self.color = 1
            print("start arm pick desk_point2")
            self.clean_status = 1
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.5)
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.5)
            while self.clean_status == 1:
                self.single_arm_init()
                rospy.sleep(0.5)
                self.single_arm_six_open()
                rospy.sleep(0.2)
                self.isDetect = False
                print("isDetect = false")
                rospy.sleep(1)
                self.isDetect = True
                rospy.sleep(0.5)
                self.single_arm_pick()
                rospy.sleep(1)
                self.clean_status = 0

                # 发布下个点导航
                msg = String()
                msg.data = "rubbish_bin2"
                self.slam_task_pub.publish(msg)

                single_servo1 = SingleServo()
                single_servo1.ID = 1
                single_servo1.Rotation_Speed = 50
                single_servo1.Target_position_Angle = 900    
                self.single_servo_pub.publish(single_servo1)

        if flag == "rubbish_bin2":
            print("start throw rubbish 2")
            self.clean_status = 1
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.5)
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.5)
            while self.clean_status == 1:
                self.single_arm_up()
                rospy.sleep(3)
                self.clean_status = 0

                #检测是否投放成功
                self.upSUCCEEDED = 1
                single_servo1 = SingleServo()
                single_servo1.ID = 1
                single_servo1.Rotation_Speed = 50
                single_servo1.Target_position_Angle = 1600    
                self.single_servo_pub.publish(single_servo1)
                rospy.sleep(1.6)
                if self.upSUCCEEDED == False:
                    self.UpCount = self.UpCount +1
                    if self.UpCount < 3:
                        # 回到第2个桌子
                        msg = String()
                        msg.data = "desk_point2"
                        self.slam_task_pub.publish(msg)
                    else:
                        # 发布下个点导航
                        self.upSUCCEEDED = 0
                        self.UpCount = 0                
                        msg = String()
                        msg.data = "gohome"
                        self.slam_task_pub.publish(msg)
                else:
                    # 发布下个点导航
                    self.upSUCCEEDED = 0 
                    self.UpCount = 0               
                    msg = String()
                    msg.data = "gohome"
                    self.slam_task_pub.publish(msg)
                single_servo1 = SingleServo()
                single_servo1.ID = 1
                single_servo1.Rotation_Speed = 50
                single_servo1.Target_position_Angle = 0
                self.single_servo_pub.publish(single_servo1)   

        if flag == "gohome":
            # self.send_cmd_vel(-0.1, 0, 0)
            # rospy.sleep(0.1)
            # self.send_cmd_vel(-0.1, 0, 0)
            # rospy.sleep(0.9)
            self.send_cmd_vel(-0.0, 0, 0)
            rospy.sleep(0.1)  
            self.send_cmd_vel(0, 0, 0) 
            print("clean complete")

    # 发送底盘运动指令
    def send_cmd_vel(self,vx,vy,vyaw):
        self.twist.linear.x = vx
        self.twist.linear.y = vy
        self.twist.angular.z = vyaw
        self.cmd_vel_pub.publish(self.twist)
            
        
    # 里程计信息回调
    def odom_callback(self,msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        r, p, y = self.quart_to_rpy(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.odom_yaw = y

    # 四元数转欧拉角
    def quart_to_rpy(self,x, y, z, w):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw

    # 所有舵机回到中位
    def servo_mid(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        for i in range(1, 21):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 50
            single_servo.Target_position_Angle = 0
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)

    # 舵机转到检查位置
    def servo_check(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []


    # 机械臂运行到初始位置
    def single_arm_init(self):
        print ("init begin")
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        for i in range(1, 8):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 50
            if i == 2:
                single_servo.Target_position_Angle = 900
            if i == 3:
                single_servo.Target_position_Angle = -900
            if i == 4:
                single_servo.Target_position_Angle = -1100
            if i == 6:
                single_servo.Target_position_Angle = -500
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)
        print("init over")

        # 机械臂运行到初始位置
    def single_arm_init_except1(self):
        print ("init begin")
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        for i in range(2, 8):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 50
            if i == 2:
                single_servo.Target_position_Angle = 900
            if i == 3:
                single_servo.Target_position_Angle = -900
            if i == 4:
                single_servo.Target_position_Angle = -1100
            if i == 6:
                single_servo.Target_position_Angle = -500
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)
        print("init over")

    # 机械臂运行到初始位置
    def single_arm_pick_after(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        for i in range(1, 8):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 30
            if i == 2:
                single_servo.Target_position_Angle = 900
            if i == 3:
                single_servo.Target_position_Angle = -900
            if i == 4:
                single_servo.Target_position_Angle = -1100
            if i == 6:
                single_servo.Target_position_Angle = -500
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)

    def single_arm_six_open(self):
        print("open")
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        single_servo6 = SingleServo()

        single_servo6.ID = 6
        single_servo6.Rotation_Speed = 80
        single_servo6.Target_position_Angle = 750
        self.single_servo_pub.publish(single_servo6)


    # 机械臂抓取
    def single_arm_pick(self):
        print("pick pick")
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        single_servo1 = SingleServo()
        single_servo2 = SingleServo()
        single_servo3 = SingleServo()
        single_servo4 = SingleServo()
        single_servo6 = SingleServo()

        single_servo2.ID = 2
        single_servo2.Rotation_Speed = 70
        single_servo2.Target_position_Angle = -750
        multiple_servo.servo_gather.append(single_servo2)

        single_servo3.ID = 3
        single_servo3.Rotation_Speed = 70
        single_servo3.Target_position_Angle = -572
        multiple_servo.servo_gather.append(single_servo3)

        single_servo4.ID = 4
        single_servo4.Rotation_Speed = 70
        single_servo4.Target_position_Angle = 410#420
        multiple_servo.servo_gather.append(single_servo4)

        single_servo6.ID = 6
        single_servo6.Rotation_Speed = 70
        single_servo6.Target_position_Angle = 200
        multiple_servo.servo_gather.append(single_servo6)

        self.multiple_servo_pub.publish(multiple_servo)
        rospy.sleep(2.5)
        self.send_cmd_vel(0.1, 0, 0)
        rospy.sleep(0.1)
        self.send_cmd_vel(0.1, 0, 0)
        rospy.sleep(0.1)
        # self.isDetect = True
        self.offset_angle = 0
        self.send_cmd_vel(0.1, 0, 0)
        rospy.sleep(1.1)
        self.send_cmd_vel(0.0, 0, 0)
        rospy.sleep(0.1)
        self.send_cmd_vel(0, 0, 0)
        rospy.sleep(0.9)
        single_servo6.ID = 6
        single_servo6.Rotation_Speed = 60
        single_servo6.Target_position_Angle = -500
        #rospy.sleep(1)
        self.single_servo_pub.publish(single_servo6)
        rospy.sleep(0.1)#2
        self.single_servo_pub.publish(single_servo6)
        rospy.sleep(0.1)
        self.single_servo_pub.publish(single_servo6)
        rospy.sleep(1)

        single_servo2.Target_position_Angle = -400
        self.single_servo_pub.publish(single_servo2)
        rospy.sleep(0.5)

        single_servo6.ID = 6
        single_servo6.Rotation_Speed = 60
        single_servo6.Target_position_Angle = -500
        self.single_servo_pub.publish(single_servo6)
        rospy.sleep(0.1)
        self.single_servo_pub.publish(single_servo6)
        
        print("pick is over")
        self.send_cmd_vel(-0.1, 0, 0)
        rospy.sleep(0.1)
        self.send_cmd_vel(-0.1, 0, 0)
        rospy.sleep(1)
        self.send_cmd_vel(0, 0, 0)
        rospy.sleep(0.01)
        self.send_cmd_vel(0, 0, 0)
        rospy.sleep(0.01)
    # 机械臂投掷
    def single_arm_up(self):
        print("arm up start")
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        # single_servo1 = SingleServo()
        # single_servo1.ID = 1
        # single_servo1.Rotation_Speed = 30
        # single_servo1.Target_position_Angle = 0
        # multiple_servo.servo_gather.append(single_servo1)

        single_servo2 = SingleServo()
        single_servo2.ID = 2
        single_servo2.Rotation_Speed = 50
        single_servo2.Target_position_Angle = -790 #-734

        multiple_servo.servo_gather.append(single_servo2)

        single_servo3 = SingleServo()
        single_servo3.ID = 3
        single_servo3.Rotation_Speed = 50
        single_servo3.Target_position_Angle = -400 #-567
        multiple_servo.servo_gather.append(single_servo3)

        single_servo4 = SingleServo()
        single_servo4.ID = 4
        single_servo4.Rotation_Speed = 50
        single_servo4.Target_position_Angle = 320
        multiple_servo.servo_gather.append(single_servo4)

        self.multiple_servo_pub.publish(multiple_servo)

        single_servo5 = SingleServo()
        single_servo5.ID = 5
        single_servo5.Rotation_Speed = 80
        single_servo5.Target_position_Angle = 0
        multiple_servo.servo_gather.append(single_servo5)

        self.multiple_servo_pub.publish(multiple_servo)
        #self.send_cmd_vel(0, 0, 0)
        single_servo6 = SingleServo()
        single_servo6.ID = 6
        single_servo6.Rotation_Speed = 60
        single_servo6.Target_position_Angle = 750
        #print ("begin sleep 3 sec")
        rospy.sleep(1)
        multiple_servo.servo_gather.append(single_servo6)
        #print("I'm going to publish servo 6")
        self.multiple_servo_pub.publish(multiple_servo)
        #print("publish servo 6")
        rospy.sleep(0.1)
        self.multiple_servo_pub.publish(multiple_servo)
        #print("publish servo 6")
        rospy.sleep(0.9)
        #print("third sleep 3 sec")
        single_servo2.Target_position_Angle = 0
        self.single_servo_pub.publish(single_servo2)
        rospy.sleep(0.1)
        self.single_servo_pub.publish(single_servo2)
        print("publish servo2 to 0")
        rospy.sleep(1)
        print("up is over")
        self.single_arm_init_except1()

    # 接收舵机控制回调
    def callback(self, msg):
        if msg.data == "mid":
            self.servo_mid()
        if msg.data == "init":
            self.single_arm_init()
            rospy.sleep(3)
            self.isDetect = False
        if msg.data == "pick":
            self.single_arm_pick()
        if msg.data == "up":
            self.single_arm_up()
        if msg.data == "pick_after":
            self.single_arm_pick_after()

if __name__ == "__main__":
    clean_desk_robot = CleanDeskRobot()
    rospy.spin()
    r = rospy.Rate(0.2)
    r.sleep()







