#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from math import atan2,acos,sqrt,pi,sin,cos,tan
from lucky_utility.ros.rospy_utility import get_tf, send_tf, vec_trans_coordinate, normalize_angle

L = 0.93 # m

class Odom_fuser():
    def __init__(self):
        rospy.init_node('odom_fuser',anonymous=False)
        rospy.Subscriber("/car1/theta", Float64, self.car1_theta_cb)
        rospy.Subscriber("/car2/theta", Float64, self.car2_theta_cb)
        self.theta1 = None
        self.theta2 = None
        #rospy.Subscriber("/car1/zed2/zed_node/odom", Odometry, self.car1_odom_cb)
        #rospy.Subscriber("/car2/zed2/zed_node/odom", Odometry, self.car2_odom_cb)
        # self.pub_odom_big_car = rospy.Publisher("/big_car/odom", Odometry,queue_size = 1,latch=False)
        self.car1_xyt = None # (x,y)
        self.car1_xy_last = None # 
        self.car1_xyt = None # (x,y)
        self.car2_xy_last = None # 
        self.carB_odom = [0,0,0]
        # self.carB_odom_1 = [0,0,0] # [x,y,theta]
        # self.carB_odom_2 = [0,0,0] # [x,y,theta]
        # self.carB_odom_3 = [0,0,0] # [x,y,theta]
        self.carB_map = [0,0,0]
        # TODO Tmp
        self.init = False
        # For getting Tf
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

    def car1_theta_cb(self, data):
        self.theta1 = data.data
    
    def car2_theta_cb(self, data):
        self.theta2 = normalize_angle(data.data + pi) # TODO add pi ?

    def run_once(self):
        # Update TF
        # car1_map = get_tf(self.tfBuffer, "car1/map", "car1/odom")
        # car2_map = get_tf(self.tfBuffer, "car2/map", "car2/odom")
        car1_xyt = get_tf(self.tfBuffer, "car1/odom", "car1/base_link")
        car2_xyt = get_tf(self.tfBuffer, "car2/odom", "car2/base_link")
        
        # Check TF and thetas are valid
        if car1_xyt == None or car2_xyt == None or\
           self.theta1 == None or self.theta2 == None: # or car1_map == None or car2_map == None:
            return False
        else:
            self.car1_xyt = car1_xyt
            self.car2_xyt = car2_xyt

        # Check last data
        if self.car1_xy_last == None or self.car2_xy_last == None:
            self.car1_xy_last = self.car1_xyt[:2]
            self.car2_xy_last = self.car2_xyt[:2]
            return False
        
        # Initialize odom
        '''
        if not self.init:
            tf_xyt = get_tf(self.tfBuffer, "car1/map", "car1/center_big_car")
            if tf_xyt == None:
                return False
            else:
                self.carB_map = list(tf_xyt)
                # self.carB_odom = [0,0,0] # TODO tmp
                self.init = True
        '''
        # Start calculate odometry
        dx1 = self.car1_xyt[0] - self.car1_xy_last[0]
        dx2 = self.car2_xyt[0] - self.car2_xy_last[0]
        dxB = (dx1*sin(self.theta1) - dx2*sin(self.theta2))/2.0
        dyB = (dx1*cos(self.theta1) - dx2*cos(self.theta2))/2.0
        
        vec_cars_x = -dx2*sin(self.theta2) - dx1*sin(self.theta1)
        vec_cars_y = -dx2*cos(self.theta2) - dx1*cos(self.theta1) - L
        dtB = atan2(vec_cars_y, vec_cars_x) - pi/2

        # Increment add to carB_odom
        self.carB_odom[0] += dxB
        self.carB_odom[1] += dyB
        self.carB_odom[2] += dtB

        # dcar1 = (self.car1_xyt[0] - self.car1_xy_last[0],
        #          self.car1_xyt[1] - self.car1_xy_last[1])
        # dcar2 = (self.car2_xyt[0] - self.car2_xy_last[0],
        #          self.car2_xyt[1] - self.car2_xy_last[1])


        # self.carB_odom_1[0] += (dcar1[0])
        # self.carB_odom_1[1] += (dcar1[1])
        # self.carB_odom_2[0] += (-dcar2[0])
        # self.carB_odom_2[1] += (-dcar2[1])
        
        # self.carB_odom_3[0] += (dcar1[0]-dcar2[0])/2
        # self.carB_odom_3[1] += (dcar1[1]-dcar2[1])/2
        # self.carB_odom_3[2] += dtheta

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
        # send_tf(self.carB_odom_1, "carB/odom", "carB/base_link_1")
        # send_tf(self.carB_odom_2, "carB/odom", "carB/base_link_2")
        # send_tf(self.carB_odom_3, "carB/odom", "carB/base_link_3")
        send_tf(self.carB_map, "carB/map", "carB/odom")
        send_tf(self.carB_odom, "carB/odom", "carB/base_link")

def main(args):
    # Init naive controller
    odom_fuser = Odom_fuser()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if odom_fuser.run_once():
            odom_fuser.publish()
            # TOOD publisher
        rate.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass