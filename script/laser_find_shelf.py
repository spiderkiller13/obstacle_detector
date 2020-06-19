#!/usr/bin/env python
import rospy
import sys
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import TransformStamped, Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from std_msgs.msg import String
import tf
import tf2_ros
from obstacle_detector.msg import Obstacles
import math 
import pprint
from math import atan2,acos,sqrt,pi,sin,cos
import tf_conversions
##################
### Parameters ###
##################
SHEFT_LENGTH_TOLERANCE = 0.13 # Meter
ANGLE_TOLERANCE   = 0.04 # RANDIUS
MAX_CIRCLE_RADIUS = 0.125 # Meter

L = 0.65
L_diagonal = L * sqrt(2)
#----- Global variable --------# 
marker_sphere = MarkerArray()
marker_line   = MarkerArray()
is_need_pub = False 
center = None
heading = None
output_angle = 0
def set_sphere(point , RGB = None  , size = 0.05, id = 0):
    '''
    Set Point at MarkArray 
    Input : 
        point - (x,y)
        RGB - (r,g,b)
    '''
    global marker_sphere
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = id
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 1.0
    if RGB == None : 
        marker.color.r = random.randint(0,255) / 255.0
        marker.color.g = random.randint(0,255) / 255.0
        marker.color.b = random.randint(0,255) / 255.0
    else: 
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
    marker.pose.orientation.w = 1.0
    (marker.pose.position.x , marker.pose.position.y) = point
    marker_sphere.markers.append(marker)

def set_line( points , RGB = None , size = 0.2, id = 0):
    '''
    Set line at MarkArray
    Input : 
        points = [p1,p2....] 
    '''
    global marker_line

    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = id
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.a = 1.0
    if RGB == None : 
        marker.color.r = random.randint(0,255) / 255.0
        marker.color.g = random.randint(0,255) / 255.0
        marker.color.b = random.randint(0,255) / 255.0
    else: 
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
    marker.pose.orientation.w = 1.0
    for i in points : 
        p = Point()
        p.x = i[0]
        p.y = i[1]
        marker.points.append(p)
    marker_line.markers.append(marker)

def cal_distance(c1,c2):
    '''
    Input:
        c1: (x1,y1)
        c2: (x2,y2)
    '''
    dx = c1[0] - c2[0]
    dy = c1[1] - c2[1]
    return sqrt(dx**2 + dy**2)

def cal_angle(c1,c2,c3):
    '''
    Calculate angle between c1c2 and c1c3
    Input:
        c1: (x1,y1)
        c2: (x2,y2)
        c3: (x3,y3)
    '''
    a = (c2[0] - c1[0] , c2[1] - c1[1])
    b = (c3[0] - c1[0] , c3[1] - c1[1])
    
    ans = acos( round( (a[0]*b[0] + a[1]*b[1]) / (cal_distance(c1,c2) * cal_distance(c1,c3) ) ,5) )
    #print ( ans )
    #ans = a[0]*b[0] + a[1]*b[1]
    return ans

def cal_center(c1,c2,c3):
    '''
    Input: 
        c1 : (x,y) - this is a coner
        c2 : - Two vertice adjcency to coner
        c3 : 
    '''
    a = (c2[0] - c1[0] , c2[1] - c1[1])
    b = (c3[0] - c1[0] , c3[1] - c1[1])
    return ( ( c1[0] + a[0]/2 + b[0]/2 , c1[1] + a[1]/2 + b[1]/2 ) )

def cal_avg_points(c_list):
    x_sum = 0
    y_sum = 0
    for i in c_list:
        x_sum += i[0]
        y_sum += i[1]
    return ( x_sum/len(c_list) , y_sum/len(c_list) )

def cal_heading(c1,c2,c3,ref):
    '''
    Input: 
        c1 : (x,y) - this is a coner
        c2 : - Two vertice adjcency to coner
        c3 : 
        ref : reg angle, output angle should be as near as possible to ref
    '''
    a = (c2[0] - c1[0] , c2[1] - c1[1])
    b = (c3[0] - c1[0] , c3[1] - c1[1])
    if ref != None:
        ang1 = find_nearest_angle_to_ref ( atan2(a[1], a[0]) , ref)
        ang2 = find_nearest_angle_to_ref ( atan2(b[1], b[0]) , ref)
    else:
        ang1 = atan2(a[1], a[0])
        ang2 = find_nearest_angle_to_ref ( atan2(b[1], b[0]) , ang1)
    return [ang1,ang2]

def find_nearest_angle_to_ref(angle ,ref):
    '''
    Find nearest angle to ref, posible angle : 
    [angle , angle+pi/2 + angle +pi + angle + 3*pi/2]
    '''
    possible_angles = [angle, angle+(pi/2) , angle+pi, angle+(3*pi/2) ]

    ans = None
    min_dtheta = float('inf')
    for i in possible_angles:
        dtheta = abs ( ( cos(i)*cos(ref) + sin(i)*sin(ref) ) -1 )
        if dtheta < min_dtheta:
            min_dtheta = dtheta
            ans = i
    return ans
    # 
def callback(data):
    global is_need_pub, center, heading
    # ---- Get Rid of too large circle -----#
    c_list = []
    for c in data.circles:
        if c.true_radius <= MAX_CIRCLE_RADIUS:
            c_list.append( (c.center.x, c.center.y) )
    #print (str(len(c_list)))
    #print (c_list)
    #----- Searching radius -------#
    #----- Get adj_dict -------#
    adj_dict = {} # {c1:[ (L1,c2) , (L2,c3) ], c2: .....}
    for c1 in c_list:
        for c2 in c_list:
            l = cal_distance(c1,c2)
            if  abs(l - L)          < SHEFT_LENGTH_TOLERANCE or\
                abs(l - L_diagonal) < SHEFT_LENGTH_TOLERANCE :
                if c1 not in adj_dict: # New circle
                    adj_dict[c1] = []
                adj_dict[c1].append( (l , c2) )
    # pprint.pprint(adj_dict)
    
    #------ Get conner in adj_list ------# 
    coner_dict = {} # {c1 : center1}
    heading_list = []
    ID = 0 
    for c in adj_dict:
        if len(adj_dict[c]) <= 1:
            continue # Too few edge, definitely not corner
        # Find two edge that has 90 degree and both with lenth L 
        for e1 in adj_dict[c]:
            for e2 in adj_dict[c]:
                if  abs(e1[0] - L) < SHEFT_LENGTH_TOLERANCE and\
                    abs(e2[0] - L) < SHEFT_LENGTH_TOLERANCE: # Both edge has length L
                    angle = cal_angle(c , e1[1] ,e2[1] )
                    if abs(angle - pi/2) < ANGLE_TOLERANCE and c not in coner_dict:
                        #---- Found coner !!! -----#
                        coner_dict[c] = cal_center(c, e1[1] ,e2[1])
                        # heading angle 
                        if heading_list == []:
                            heading_list.extend( cal_heading(c, e1[1] ,e2[1], None) )
                        else: 
                            heading_list.extend( cal_heading(c, e1[1] ,e2[1], heading_list[0]) )

                        # rviz
                        set_sphere( c ,     (0,0,255) , 0.1, ID)
                        set_line([c,e1[1]], (255,255,0), 0.02, ID)
                        set_line([c,e2[1]], (255,255,0), 0.02, ID+1)
                        ID += 2

    #------ Get avg center of sheft ----#
    centers = []
    if len(coner_dict) != 0:
        # Calculate avg heading
        heading = [0,0]
        for i in heading_list:
            heading[0] += cos(i)
            heading[1] += sin(i)
        heading[0] /= len(heading_list)
        heading[1] /= len(heading_list)
        heading = atan2(heading[1],heading[0])
        #Calculate avg center 
        for i in coner_dict:
            centers.append( coner_dict[i] )
        center = cal_avg_points(centers)
        rospy.loginfo("[Obstacle_detector] Found shelft center : " + str( center ) + ", with heading angle : " + str(heading))
        is_need_pub = True 

def main(args):
    global marker_sphere , marker_line, is_need_pub, center, heading, output_angle
    #----- Init node ------#
    # Name of this node, anonymous means name will be auto generated.
    rospy.init_node('laser_find_shelf', anonymous=False)
    #----- Subscriber -------# 
    rospy.Subscriber("raw_obstacles", Obstacles, callback)
    #----- Markers --------# 
    pub_marker_sphere     = rospy.Publisher('marker_sphere', MarkerArray,queue_size = 1,latch=False)
    pub_marker_line       = rospy.Publisher('marker_line'  , MarkerArray,queue_size = 1,latch=False)
    #----- TF -----------#
    tfBuffer = tf2_ros.Buffer()
    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        if is_need_pub:
            # clean all markers
            marker = Marker()
            marker.header.frame_id = "map"
            marker.action = marker.DELETEALL
            m = MarkerArray()
            m.markers.append(marker)
            pub_marker_sphere.publish(m)
            pub_marker_line.publish(m)
            #---- update center of rotations -----# 
            pub_marker_sphere.publish(marker_sphere)
            pub_marker_line.publish(marker_line)
            #---- listen tf base_link->s_center -----#
            marker_sphere = MarkerArray()
            marker_line   = MarkerArray()
            '''
            output_angle = None
            try:
                t = tfBuffer.lookup_transform('map', 'shelf_center_laser', rospy.Time()) 
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print (e)
                output_angle = 0
            else:
                output_angle = find_nearest_angle_to_ref (heading, atan2(t.transform.translation.y, t.transform.translation.x)) 
            '''
            output_angle = find_nearest_angle_to_ref(heading, output_angle)
            #---- update center tf -----#
            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "shelf_center_laser"
            t.transform.translation.x = center[0]
            t.transform.translation.y = center[1]
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, output_angle)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            br.sendTransform(t)
            # ---- Reset ------# 
            is_need_pub = False 
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
