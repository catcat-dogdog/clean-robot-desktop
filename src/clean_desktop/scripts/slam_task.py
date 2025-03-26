#!/usr/bin/env python3
from operator import le
from turtle import goto
import rospy
import actionlib
import roslaunch
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from xml.dom.minidom import parse
from math import pi
from geometry_msgs.msg import Twist
import tf

class SlamTask:
    def __init__(self):
        # 底盘控制节点发布
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # 初始化ros节点
        rospy.init_node('slam_task_node', anonymous=True)
        # 订阅接收导航任务发布信息
        rospy.Subscriber("/slam_task", String, self.slam_task_callback)
        # slam结果消息发布
        self.slam_result_pub = rospy.Publisher('/slam_result', String, queue_size=5)
        # 舵机动作消息发布
        self.servo_controller_pub = rospy.Publisher('/servo_controller', String, queue_size=5)
        # move_base simple action
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.tf_listener = tf.TransformListener()
        self.success_count = 0
        # home点位置信息
        self.home_point_x = rospy.get_param('home_point_x', -0.1)
        self.home_point_y = rospy.get_param('home_point_y', -0.1)
        self.home_point_yaw = rospy.get_param('home_point_yaw', -145)
        # 第一个桌子位置信息
        self.desktop1_point_x = rospy.get_param('desktop1_point_x', 2.12)
        self.desktop1_point_y = rospy.get_param('desktop1_point_y', 0.16)
        self.desktop1_point_yaw = rospy.get_param('desktop1_point_yaw', 0)
        # 第二个桌子位置信息
        self.desktop2_point_x = rospy.get_param('desktop2_point_x', 2.09)
        self.desktop2_point_y = rospy.get_param('desktop2_point_y', 2.17)
        self.desktop2_point_yaw = rospy.get_param('desktop2_point_yaw',0)
        # 垃圾桶1点位置信息
        self.rubbish_bin1_x = rospy.get_param('rubbish_bin1_x', self.desktop1_point_x + 0.07)
        self.rubbish_bin1_y = rospy.get_param('rubbish_bin1_y', self.desktop1_point_y + 0.50)
        self.rubbish_bin1_yaw = rospy.get_param('rubbish_bin1_yaw', 90)
        # 垃圾桶2点位置信息
        self.rubbish_bin2_x = rospy.get_param('rubbish_bin2_x', self.desktop2_point_x + 0.08)
        self.rubbish_bin2_y = rospy.get_param('rubbish_bin2_y', self.desktop2_point_y - 0.50)
        self.rubbish_bin2_yaw = rospy.get_param('rubbish_bin2_yaw', -90)
        self.try_time = 0
        # 底盘msg
        self.twist = Twist()

    def _done_cb(self, status, result):
        a=1
        # rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        a=1
        # rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        a=1
        # rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    # 导航到某个目标点
    def goto(self, p, flag):
        print(p)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        goal.target_pose.pose.position.z = 0
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        state = self.move_base.get_state()

        # 通知机器人，到达目标点
        if state == GoalStatus.PENDING:
            rospy.loginfo("目标待处理")
        elif state == GoalStatus.ACTIVE:
            rospy.loginfo("目标处理中")
        elif state == GoalStatus.PREEMPTED:
            rospy.logwarn("目标被抢占")
        elif state == GoalStatus.SUCCEEDED:
            rospy.loginfo("目标达成")
        elif state == GoalStatus.ABORTED:
            rospy.logerr("目标中止")
        elif state == GoalStatus.REJECTED:
            rospy.logerr("目标被拒绝")
        elif state == GoalStatus.PREEMPTING:
            rospy.logwarn("目标正在被抢占")
        elif state == GoalStatus.RECALLING:
            rospy.logwarn("目标正在撤回")
        elif state == GoalStatus.RECALLED:
            rospy.logwarn("目标已被撤回")
        elif state == GoalStatus.LOST:
            rospy.logerr("目标状态未知或丢失")
        else:
            rospy.logerr("未知目标状态")

        if state == GoalStatus.SUCCEEDED:
            print("arrive {}".format(flag))
            msg = String()
            msg.data = flag
            self.slam_result_pub.publish(msg)
            print("I'm the next line of result pub")
            self.try_time = 0
        elif self.try_time <= 3:
            self.send_cmd_vel(-0.1, 0, 0)
            rospy.sleep(0.5)
            self.send_cmd_vel(0, 0, 0)
            rospy.sleep(0.01)
            self.send_cmd_vel(0, 0, 0)
            print("try times:"+str(self.try_time))
            self.try_time = self.try_time + 1
            self.goto(p,flag)
     
        else :
            print("not arrive but go on{}".format(flag))
            msg = String()
            msg.data = flag
            self.slam_result_pub.publish(msg)
            self.try_time = 0

    def send_cmd_vel(self,vx,vy,vyaw):
        self.twist.linear.x = vx
        self.twist.linear.y = vy
        self.twist.angular.z = vyaw
        self.cmd_vel_pub.publish(self.twist)
    
    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    # slam任务导航回调
    def slam_task_callback(self, msg):
        flag = msg.data
        if flag == "desk_point1":
            print("start go to desk_point1")
            point1 = (self.desktop1_point_x, self.desktop1_point_y, self.desktop1_point_yaw)
            self.goto(point1, flag)
        if flag == "desk_point2":
            print("start go to desk_point2")
            point2 = (self.desktop2_point_x, self.desktop2_point_y, self.desktop2_point_yaw)
            self.goto(point2, flag)
        if flag == "gohome":
            print("gohome")
            home = (self.home_point_x, self.home_point_y, self.home_point_yaw)
            self.goto(home, flag)
        if flag == "rubbish_bin1":
            print("start go to rubbish_bin1")
            rubbish_bin1 = (self.rubbish_bin1_x, self.rubbish_bin1_y, self.rubbish_bin1_yaw)
            self.goto(rubbish_bin1, flag)
        if flag == "rubbish_bin2":
            print("start go to rubbish_bin2")
            rubbish_bin2 = (self.rubbish_bin2_x, self.rubbish_bin2_y, self.rubbish_bin2_yaw)
            self.goto(rubbish_bin2, flag)

if __name__ == "__main__":
    nav = SlamTask()
    rospy.spin()
    r = rospy.Rate(0.2)
    r.sleep()