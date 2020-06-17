#!/usr/bin/env python
import rospy
import sys
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from std_msgs.msg import String
import tf
from obstacle_detector.msg import Obstacles
import math 
import pprint
##################
### Parameters ###
##################
SHEFT_LENGTH_TOLERANCE = 0.1 # Meter
ANGLE_TOLERANCE   = 0.04 # RANDIUS
MAX_CIRCLE_RADIUS = 0.125 # Meter

L = 0.65
L_diagonal = 0.65 * math.sqrt(2)

marker_sphere = MarkerArray()
marker_line   = MarkerArray()
is_need_pub = False 

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
    return math.sqrt(dx**2 + dy**2)

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
    
    ans = math.acos( round( (a[0]*b[0] + a[1]*b[1]) / (cal_distance(c1,c2) * cal_distance(c1,c3) ) ,5) )
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

def callback(data):
    global is_need_pub
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
                    if abs(angle - math.pi/2) < ANGLE_TOLERANCE and c not in coner_dict:
                        #---- Found coner !!! -----#
                        coner_dict[c] = cal_center(c, e1[1] ,e1[1])
                        # TODO rviz  
                        set_sphere( c ,     (0,255,255) , 0.2, ID)
                        set_line([c,e1[1]], (255,255,0), 0.02, ID)
                        set_line([c,e2[1]], (255,255,0), 0.02, ID+1)
                        ID += 2

    # print (coner_dict)

    #------ Get avg center of sheft ----#
    centers = []
    if len(coner_dict) != 0:
        for i in coner_dict:
            centers.append( coner_dict[i] )
        center = cal_avg_points(centers)
        rospy.loginfo("[Obstacle_detector] Found shelft center : " + str( center ))

    is_need_pub = True 




def main(args):
    global marker_sphere , marker_line, is_need_pub
    #----- Init node ------#
    # Name of this node, anonymous means name will be auto generated.
    rospy.init_node('laser_find_shelf', anonymous=False)
    #----- Subscriber -------# 
    rospy.Subscriber("raw_obstacles", Obstacles, callback)
    #----- Markers --------# 
    pub_marker_sphere     = rospy.Publisher('marker_sphere', MarkerArray,queue_size = 1,latch=True)
    pub_marker_line       = rospy.Publisher('marker_line'  , MarkerArray,queue_size = 1,latch=True)
    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        if is_need_pub:
            #---- update center of rotations -----# 
            pub_marker_sphere.publish(marker_sphere)
            #---- update lines -----# 
            pub_marker_line.publish(marker_line)

            # ---- Reset ------# 
            marker_sphere = MarkerArray()
            marker_line   = MarkerArray()
            is_need_pub = False 
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
