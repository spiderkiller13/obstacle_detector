#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
from nav_msgs.msg import Odometry
from math import atan2,acos,sqrt,pi,sin,cos,tan
from lucky_utility.ros.rospy_utility import get_tf, send_tf


class Odom_fuser():
    def __init__(self):
        rospy.init_node('odom_fuser',anonymous=False)
        #rospy.Subscriber("/car1/zed2/zed_node/odom", Odometry, self.car1_odom_cb)
        #rospy.Subscriber("/car2/zed2/zed_node/odom", Odometry, self.car2_odom_cb)
        # self.pub_odom_big_car = rospy.Publisher("/big_car/odom", Odometry,queue_size = 1,latch=False)
        self.car1_xyt = None # (x,y)
        self.car1_xy_last = None # 
        self.car1_xyt = None # (x,y)
        self.car2_xy_last = None # 
        self.carB_xyt = [0,0,0] # [x,y,theta]
        # TODO Tmp
        self.init = False
        # For getting Tf
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)


    def run_once(self):
        # Update TF 
        car1_xyt = get_tf(self.tfBuffer, "car1/odom", "car1/base_link")
        car2_xyt = get_tf(self.tfBuffer, "car2/odom", "car2/base_link")
        if car1_xyt == None or car2_xyt == None:
            return False
        else:
            self.car1_xyt = car1_xyt
            self.car2_xyt = car2_xyt

        if self.car1_xy_last == None or self.car2_xy_last == None:
            self.car1_xy_last = self.car1_xyt[:2]
            self.car2_xy_last = self.car2_xyt[:2]
            return False
        if not self.init:
            tf_xyt = get_tf(self.tfBuffer, "car1/map", "car1/center_big_car")
            if tf_xyt == None:
                return False
            else:
                self.carB_xyt = list(tf_xyt)
                self.init = True
        
        dcar1 = (self.car1_xyt[0] - self.car1_xy_last[0], self.car1_xyt[1] - self.car1_xy_last[1])
        dcar2 = (self.car2_xyt[0] - self.car2_xy_last[0], self.car2_xyt[1] - self.car2_xy_last[1])

        # TODO Reverse the polarity -- Doctor Who, WHY?
        
        self.carB_xyt[0] += (dcar1[0]-dcar2[0])/2.0
        self.carB_xyt[1] += (dcar1[1]-dcar2[1])/2.0
        # carB_xyt[2] += (dcar1+dcar2)/2.0 # TODO do atan2()
        
        # 
        self.car1_xy_last = self.car1_xyt[:2]
        self.car2_xy_last = self.car2_xyt[:2]
        return True 

    '''
    def car1_odom_cb(self, data):
        self.car1_xy = (data.pose.pose.position.x, data.pose.pose.position.y)
        if self.car1_xy_last == None:
            self.car1_xy_last = self.car1_xy

    def car2_odom_cb(self, data):
        self.car2_xy = (data.pose.pose.position.x, data.pose.pose.position.y)
        if self.car2_xy_last == None:
            self.car2_xy_last = self.car2_xy
    '''
    def publish(self):
        send_tf(self.carB_xyt, "car1/map", "car1/base_link_big_car")
def main(args):
    # Init naive controller
    odom_fuser = Odom_fuser()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if odom_fuser.run_once():
            odom_fuser.publish()
            # print (odom_fuser.carB_xyt)
            # TOOD publisher
        rate.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
