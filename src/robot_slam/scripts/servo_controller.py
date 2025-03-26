#!/usr/bin/env python
import rospy
from zoo_bringup.msg import SingleServo,MultipleServo 
from std_msgs.msg import String
import time


class ServoController:
    def __init__(self):
        rospy.init_node('servo_controller_node',anonymous=True)
        self.success_count = 0
        self.single_servo_pub = rospy.Publisher('/single_servo_topic', SingleServo, queue_size = 5)
        self.multiple_servo_pub = rospy.Publisher('/multiple_servo_topic', MultipleServo, queue_size = 5)
        rospy.Subscriber("servo_controller", String,self.callback)
        rospy.spin()
        
    def servo_mid(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        for i in range(1,21):
            if i != 6:
                single_servo = SingleServo()
                single_servo.ID = i
                single_servo.Rotation_Speed = 50
                single_servo.Target_position_Angle = 0
                multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)
    

    def single_arm_init(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        for i in range(1,10):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 30

            if i == 2:
                single_servo.Target_position_Angle = 900
            if i == 3:
                single_servo.Target_position_Angle = -900
            if i == 4:
                single_servo.Target_position_Angle = -1000
            if i == 6:
                single_servo.Target_position_Angle = 500
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)


    def double_arm_init(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []
        for i in range(1,10):
            single_servo = SingleServo()
            single_servo.ID = i
            single_servo.Rotation_Speed = 150
            if i == 3:
                single_servo.Target_position_Angle = 900
            elif i == 4:
                single_servo.Target_position_Angle = -900
            elif i == 2:
                single_servo.Target_position_Angle = -900
            elif i == 5:
                single_servo.Target_position_Angle = 900
            else:
                single_servo.Target_position_Angle = 0
            print(single_servo)
            multiple_servo.servo_gather.append(single_servo)
        self.multiple_servo_pub.publish(multiple_servo)

    

    def single_arm_pick(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        single_servo1 = SingleServo()
        single_servo1.ID = 1
        single_servo1.Rotation_Speed = 30
        single_servo1.Target_position_Angle = 0

        multiple_servo.servo_gather.append(single_servo1)


        single_servo2 = SingleServo()
        single_servo2.ID = 2
        single_servo2.Rotation_Speed = 50
        single_servo2.Target_position_Angle = -580

        multiple_servo.servo_gather.append(single_servo2)

        single_servo3 = SingleServo()
        single_servo3.ID = 3
        single_servo3.Rotation_Speed = 50
        single_servo3.Target_position_Angle = -477
        multiple_servo.servo_gather.append(single_servo3)

        # self.multiple_servo_pub.publish(multiple_servo)

        single_servo4 = SingleServo()
        single_servo4.ID = 4
        single_servo4.Rotation_Speed = 50
        single_servo4.Target_position_Angle = 0
        multiple_servo.servo_gather.append(single_servo4)

        self.multiple_servo_pub.publish(multiple_servo)

        single_servo5 = SingleServo()
        single_servo5.ID = 5
        single_servo5.Rotation_Speed = 50
        single_servo5.Target_position_Angle = 0
        multiple_servo.servo_gather.append(single_servo5)

        self.multiple_servo_pub.publish(multiple_servo)


        single_servo6 = SingleServo()
        single_servo6.ID = 6
        single_servo6.Rotation_Speed = 50
        single_servo6.Target_position_Angle = -580
        multiple_servo.servo_gather.append(single_servo6)
        time.sleep(4)
        self.multiple_servo_pub.publish(multiple_servo)

    def single_arm_up(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        single_servo1 = SingleServo()
        single_servo1.ID = 1
        single_servo1.Rotation_Speed = 30
        single_servo1.Target_position_Angle = 0

        multiple_servo.servo_gather.append(single_servo1)


        single_servo2 = SingleServo()
        single_servo2.ID = 2
        single_servo2.Rotation_Speed = 50
        single_servo2.Target_position_Angle = -580

        multiple_servo.servo_gather.append(single_servo2)

        single_servo3 = SingleServo()
        single_servo3.ID = 3
        single_servo3.Rotation_Speed = 50
        single_servo3.Target_position_Angle = -477
        multiple_servo.servo_gather.append(single_servo3)

        # self.multiple_servo_pub.publish(multiple_servo)

        single_servo4 = SingleServo()
        single_servo4.ID = 4
        single_servo4.Rotation_Speed = 50
        single_servo4.Target_position_Angle = 0
        multiple_servo.servo_gather.append(single_servo4)

        self.multiple_servo_pub.publish(multiple_servo)

        single_servo5 = SingleServo()
        single_servo5.ID = 5
        single_servo5.Rotation_Speed = 50
        single_servo5.Target_position_Angle = 0
        multiple_servo.servo_gather.append(single_servo5)

        self.multiple_servo_pub.publish(multiple_servo)


        single_servo6 = SingleServo()
        single_servo6.ID = 6
        single_servo6.Rotation_Speed = 50
        single_servo6.Target_position_Angle = 0
        multiple_servo.servo_gather.append(single_servo6)
        time.sleep(4)
        self.multiple_servo_pub.publish(multiple_servo)
    
    def double_arm_pick(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        single_servo2 = SingleServo()
        single_servo2.ID = 2
        single_servo2.Rotation_Speed = 100
        single_servo2.Target_position_Angle = -1124

        multiple_servo.servo_gather.append(single_servo2)

        single_servo3 = SingleServo()
        single_servo3.ID = 3
        single_servo3.Rotation_Speed = 100
        single_servo3.Target_position_Angle = 0

        single_servo4 = SingleServo()
        single_servo4.ID = 4
        single_servo4.Rotation_Speed = 100
        single_servo4.Target_position_Angle = 0

        single_servo5 = SingleServo()
        single_servo5.ID = 5
        single_servo5.Rotation_Speed = 100
        single_servo5.Target_position_Angle = 1100
        multiple_servo.servo_gather.append(single_servo5)

        self.multiple_servo_pub.publish(multiple_servo)


    def double_arm_open(self):
        multiple_servo = MultipleServo()
        multiple_servo.servo_gather = []

        single_servo2 = SingleServo()
        single_servo2.ID = 2
        single_servo2.Rotation_Speed = 150
        single_servo2.Target_position_Angle = -900

        multiple_servo.servo_gather.append(single_servo2)

        single_servo5 = SingleServo()
        single_servo5.ID = 5
        single_servo5.Rotation_Speed = 150
        single_servo5.Target_position_Angle = 980
        multiple_servo.servo_gather.append(single_servo5)

        self.multiple_servo_pub.publish(multiple_servo)
    

    def callback(self,msg):
        if msg.data == "mid":
            self.servo_mid()
        if msg.data == "init":
            self.double_arm_init()
        if msg.data == "pick":
            self.servo_mid()
            time.sleep(1)
            self.double_arm_pick()
        if msg.data == "open":
            self.double_arm_open()
        if msg.data == "single_init":
            self.single_arm_init()
        if msg.data == "single_pick":
            self.single_arm_pick()
        if msg.data == "up":
            self.single_arm_up()
    
    
if __name__ == "__main__":
    servo = ServoController()
    # rospy.spin()
    # r = rospy.Rate(0.2)
    # r.sleep()
    
    




