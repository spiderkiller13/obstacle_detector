#!/usr/bin/env python
import rospy
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import TransformStamped, Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from std_msgs.msg import String
import tf
import tf2_ros
from obstacle_detector.msg import Obstacles
from math import atan2,acos,sqrt,pi,sin,cos
import tf_conversions

class Laser_find_shelf():
    def __init__(self,sheft_length_tolerance, angle_tolerance, max_circle_radius, search_radius, robot_name, role): 
        # self.is_need_pub = False
        self.scan = None
        self.center = None # (x,y,theta) - centers of shelf
        self.center_peer = None  # (x,y,theta) - centers of peer shelf
        self.heading = None # [ang1,ang2, ... ] heading of shelf
        self.output_angle = 0
        self.base_link_xy = None # (x,y), transformation map -> base_link
        self.direction_list = [(TOW_CAR_LENGTH, 0),(-TOW_CAR_LENGTH, 0),(0, TOW_CAR_LENGTH),(0, -TOW_CAR_LENGTH)]

        #------ Parameters --------#
        self.sheft_length_tolerance = sheft_length_tolerance # Meter, Tolerance of detecing shelf's length
        self.angle_tolerance = angle_tolerance # Radian, Tolerance of detecing right angle
        self.max_circle_radius = max_circle_radius # Meter, Ignore clusters's radius bigger than MAX_CIRCLE_RADIUS
        self.search_radius = search_radius
        self.robot_name = robot_name
        self.role = role
        #----- Init node ------#
        # Name of this node, anonymous means name will be auto generated.
        rospy.init_node('laser_find_shelf', anonymous=False)
        #----- Subscriber -------# 
        rospy.Subscriber("raw_obstacles", Obstacles, self.callback)
        #----- Markers --------# 
        self.pub_marker_sphere     = rospy.Publisher('marker_sphere', MarkerArray,queue_size = 1,latch=False)
        self.pub_marker_line       = rospy.Publisher('marker_line'  , MarkerArray,queue_size = 1,latch=False)
        self.marker_sphere = MarkerArray()# Sphere markers show on RIVZ
        self.marker_line   = MarkerArray()# Line markers show on RIVZ
        self.marker_id = 0
        #----- TF -----------#
        # For getting Tf map -> base_link
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
    
    def set_mode(self,sheft_length_tolerance, angle_tolerance, max_circle_radius, search_radius):
        '''
        '''
        self.sheft_length_tolerance = sheft_length_tolerance 
        self.angle_tolerance = angle_tolerance 
        self.max_circle_radius = max_circle_radius 
        self.search_radius = search_radius
    
    def get_base_link(self):
        try:
            t = self.tfBuffer.lookup_transform(self.robot_name + "/map", self.robot_name + "/base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass 
            # return None 
        else:
            self.base_link_xy = (t.transform.translation.x, t.transform.translation.y)

    def set_sphere(self, point ,frame_id , RGB = (255,0,0)  , size = 0.05, id = 0):
        '''
        Set Point at MarkArray 
        Input : 
            point - (x,y)
            RGB - (r,g,b)
        '''
        marker = Marker()
        marker.header.frame_id = frame_id # self.robot_name+"/map"
        marker.id = id
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
        marker.pose.orientation.w = 1.0
        (marker.pose.position.x , marker.pose.position.y) = point
        self.marker_sphere.markers.append(marker)

    def set_line(self, points ,frame_id, RGB = (255,0,0) , size = 0.2, id = 0):
        '''
        Set line at MarkArray
        Input : 
            points = [p1,p2....] 
        '''
        marker = Marker()
        marker.header.frame_id = frame_id # self.robot_name+"/map"
        marker.id = id
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        marker.color.r = RGB[0]/255.0
        marker.color.g = RGB[1]/255.0
        marker.color.b = RGB[2]/255.0
        marker.pose.orientation.w = 1.0
        for i in points : 
            p = Point()
            p.x = i[0]
            p.y = i[1]
            marker.points.append(p)
        self.marker_line.markers.append(marker)

    def cal_distance(self,c1,c2):
        '''
        Calculate Eucleides distance between c1 and c2 
        Input:
            c1: (x1,y1)
            c2: (x2,y2)
        Return: 
            float - Eucleides distance
        '''
        dx = c1[0] - c2[0]
        dy = c1[1] - c2[1]
        return sqrt(dx**2 + dy**2)

    def cal_angle(self,c1,c2,c3):
        '''
        Calculate angle between c1c2 and c1c3
        Input:
            c1: (x1,y1)
            c2: (x2,y2)
            c3: (x3,y3)
        Output: 
            angle - Radian
        '''
        a = (c2[0] - c1[0] , c2[1] - c1[1])
        b = (c3[0] - c1[0] , c3[1] - c1[1])
        return acos( round( (a[0]*b[0] + a[1]*b[1]) / (self.cal_distance(c1,c2) * self.cal_distance(c1,c3) ) ,5) )

    def cal_center(self,c1,c2,c3):
        '''
        Calculate center of shelf by right triangle (c1,c2,c3)
        Input: 
            c1 : (x,y) - this is a right angle coner
            c2 : 
            c3 : - Two vertice adjcency to coner
        Output:
            center: (x,y)
        '''
        a = (c2[0] - c1[0] , c2[1] - c1[1])
        b = (c3[0] - c1[0] , c3[1] - c1[1])
        return ( ( c1[0] + a[0]/2 + b[0]/2 , c1[1] + a[1]/2 + b[1]/2 ) )

    def cal_avg_points(self,c_list):
        '''
        Calculate avarage center from c_list
        Input:
            c_list: [c1,c2,c3,c4,....]
        OutPut:
            avg_center:(x,y)
        '''
        x_sum = 0
        y_sum = 0
        for i in c_list:
            x_sum += i[0]
            y_sum += i[1]
        return ( x_sum/len(c_list) , y_sum/len(c_list) )

    def cal_heading(self,c1,c2,c3,ref):
        '''
        Calculate heading of shelft by a right triangle(c1,c2,c3)
        Because shelf is a sqare, there're four possible directions (NSWE)
        we choose direction that is cloest to ref angle.
        Arguments:
            c1 : (x,y) - this is a coner
            c2 : (x,y) - Two vertice adjcency to coner
            c3 : (x,y)
            ref : referance angle, output angle should be as near as possible to ref
        Return:
            float - heading
        Note that if ref is None, it will pick a direction arbitrarily
        '''
        a = (c2[0] - c1[0] , c2[1] - c1[1])
        b = (c3[0] - c1[0] , c3[1] - c1[1])
        if ref != None: # two sides of the right triangle, give us two values of heading
            ang1 = self.find_nearest_angle_to_ref ( atan2(a[1], a[0]) , ref)
            ang2 = self.find_nearest_angle_to_ref ( atan2(b[1], b[0]) , ref)
        else:
            ang1 = atan2(a[1], a[0])
            ang2 = self.find_nearest_angle_to_ref ( atan2(b[1], b[0]) , ang1)
        # return [ang1,ang2]
        return (ang1 + ang2)/2.0

    def find_nearest_angle_to_ref(self, angle ,ref):
        '''
        Find nearest angle to ref
        All four possible angle : 
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

    def check_search_region(self, search_center, search_radius, test_point):
        '''
        Check test point is in search region or not
        Arguments:
            search_center - (x,y), center of search region
            search_radius - float, radius of search region 
            test_point - (x,y), point that need to be checked.
        Return: 
            True - Inside search region
            False - Outside search region 
        Note that if search_center or search_radiusa are None, then return Ture without checking(disable checking).
        '''
        if search_center == None or search_radius == None: # skip checking 
            return True 
        else: # need to check
            if self.cal_distance(test_point, search_center) > search_radius: #Outside search region
                return False
            else:
                return True

    def get_center(self, raw_data, search_center):
        '''
        Arguement:
            raw_data - obstacle_detector/Obstacles
            search_center - (x,y)
        Return:
            (x,y, theta) - center of shelft
        If can't find center, return None 
        '''
        # global MARKER_ID
        # Ignore too large circle, and those outside the search region
        c_list = []
        for c in raw_data.circles:
            c_point = (c.center.x, c.center.y)
            if c.true_radius <= self.max_circle_radius and\
               self.check_search_region(search_center, self.search_radius, c_point):
                c_list.append(c_point)

        # Get adj_dict, indicate adjecency relationship between pionts.
        adj_dict = {} # {c1:[ (L1,c2) , (L2,c3) ], c2: .....}
        for c1 in c_list:
            for c2 in c_list:
                l = self.cal_distance(c1,c2)
                if  abs(l - SHELF_LEN)          < self.sheft_length_tolerance or\
                    abs(l - SHELF_LEN_DIAGONAL) < self.sheft_length_tolerance :
                    if c1 not in adj_dict: # New circle
                        adj_dict[c1] = []
                    adj_dict[c1].append( (l , c2) )
        
        # Get conner in adj_list
        coner_dict = {} # {c1 : center1}
        heading_list = []
        
        for c in adj_dict:
            if len(adj_dict[c]) <= 1: # Too few edge, definitely not a corner
                continue 
            # Find two edge that has 90 degree and both with lenth L 
            for e1 in adj_dict[c]:
                for e2 in adj_dict[c]:
                    if  abs(e1[0] - SHELF_LEN) < self.sheft_length_tolerance and\
                        abs(e2[0] - SHELF_LEN) < self.sheft_length_tolerance: # Both edge has length L
                        angle = self.cal_angle(c , e1[1] ,e2[1] )
                        if abs(angle - pi/2) < self.angle_tolerance and c not in coner_dict:# Angle is 90 degree
                            # Found coner!!!
                            coner_dict[c] = self.cal_center(c, e1[1] ,e2[1])
                            # heading angle
                            try:
                                # heading_list.extend( self.cal_heading(c, e1[1] ,e2[1], None) )
                                heading_list.append(self.cal_heading(c, e1[1] ,e2[1], heading_list[0]))
                            except IndexError:
                                # heading_list.extend( self.cal_heading(c, e1[1] ,e2[1], heading_list[0]) )
                                heading_list.append(self.cal_heading(c, e1[1] ,e2[1], None))

                            # RVIZ set marker
                            self.set_sphere( c ,     (0,0,255) , 0.1, self.marker_id)
                            self.set_line([c,e1[1]], (255,255,0), 0.02, self.marker_id)
                            self.set_line([c,e2[1]], (255,255,0), 0.02, self.marker_id+1)
                            self.marker_id += 2

        #------ Get avg center of sheft ----#
        if len(coner_dict) != 0:
            # Calculate avg heading
            heading = [0,0]
            for i in heading_list:
                heading[0] += cos(i)
                heading[1] += sin(i)
            heading[0] /= len(heading_list)
            heading[1] /= len(heading_list)
            heading = atan2(heading[1],heading[0])
            # Calculate avg center 
            centers = []
            for i in coner_dict:
                centers.append(coner_dict[i])
            center = self.cal_avg_points(centers)
            rospy.logdebug("[Obstacle_detector] Found shelft center")
            return (center[0], center[1], heading)
        else:
            return None

    def callback(self,data):
        '''
        This callback will be called when /raw_obstacle is published
        Get self.center - center of shelft neer base_link
        and self.center_peer - center of shelft neer base_link_peer
        '''
        self.scan = data

    def publish(self):
        '''
        '''
        # global MARKER_ID
        #---- Update all markers on RVIZ -----# 
        # clean all markers
        marker = Marker()
        marker.header.frame_id = self.robot_name+"/map"
        marker.action = marker.DELETEALL
        m = MarkerArray()
        m.markers.append(marker)
        self.pub_marker_sphere.publish(m)
        self.pub_marker_line.publish(m)
        # update center of shelf
        self.pub_marker_sphere.publish(self.marker_sphere)
        self.pub_marker_line.publish(self.marker_line)
        # Reset markers
        self.marker_id = 0
        self.marker_sphere = MarkerArray()
        self.marker_line   = MarkerArray()
        
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        #---- update s_center_laser tf -----#
        if self.center != None:
            #br = tf2_ros.TransformBroadcaster()
            #t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.robot_name+"/map"
            t.child_frame_id = self.robot_name+"/s_center_laser"
            t.transform.translation.x = self.center[0]
            t.transform.translation.y = self.center[1]
            t.transform.translation.z = 0.0
            # TODO what on earth is the output angle
            q = tf_conversions.transformations.quaternion_from_euler(0, 0,\
                self.find_nearest_angle_to_ref(self.center[2], self.output_angle))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)
        
        if self.center_peer != None:
            #---- update s_center_laser tf -----#
            # br = tf2_ros.TransformBroadcaster()
            # t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.robot_name+"/map"
            t.child_frame_id = self.robot_name+"/center_peer"
            t.transform.translation.x = self.center_peer[0]
            t.transform.translation.y = self.center_peer[1]
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0,
                self.find_nearest_angle_to_ref(self.center_peer[2], self.output_angle))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)
        
        if self.center != None and self.center_peer != None:
            #---- update s_center_laser tf -----#
            try: # TODO DEBUG use
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.robot_name+"/map"
                t.child_frame_id = self.robot_name+"/center_big_car"
                t.transform.translation.x = (self.center[0] + self.center_peer[0])/2.0
                t.transform.translation.y = (self.center[1] + self.center_peer[1])/2.0
                t.transform.translation.z = 0.0
                if self.role == "leader": # center - center_peer
                    vec_big_car = (self.center[0] - self.center_peer[0], self.center[1] - self.center_peer[1])
                elif self.role == "follower": # center_peer - center
                    vec_big_car = (self.center_peer[0] - self.center[0], self.center_peer[1] - self.center[1])
                q = tf_conversions.transformations.quaternion_from_euler(0, 0,atan2(vec_big_car[1], vec_big_car[0]))
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                br.sendTransform(t)
            except Exception as e:
                print (self.center)
                print (self.center_peer)
                print (e)
        
    def run_once(self):
        '''
        return True: Allow publish
        '''
        if self.scan == None: # No scan data
            return False
        center = self.get_center(self.scan, self.base_link_xy)
        if center != None:
            self.center = center
            theta = self.find_nearest_angle_to_ref(self.center[2], self.output_angle)
            for p_tem in self.direction_list:
                # Iterate all possible direction
                (x, y) = (self.center[0] + cos(theta)*p_tem[0] - sin(theta)*p_tem[1],
                        self.center[1] + sin(theta)*p_tem[0] + cos(theta)*p_tem[1])
                tentative_center = self.get_center(self.scan, (x, y))
                if tentative_center != None: # Found the center of peer
                    self.center_peer = tentative_center
                    # Cache the direction
                    self.direction_list.remove(p_tem)
                    self.direction_list.insert(0,p_tem)
                    break
            
            if self.center_peer == None:
                rospy.logerr("[laser_finder] Can't find center_peer.")
            return True# let main publish
        else:
            return False
# def main(args):

    

#     r = rospy.Rate(10) #call at 10HZ
#     while (not rospy.is_shutdown()):
#         laser_find_shelf.get_base_link()
#         if laser_find_shelf.is_need_pub:
#             laser_find_shelf.update_publish()
#             laser_find_shelf.is_need_pub = False # Reset flag 
#         r.sleep()
    
if __name__ == '__main__':
    rospy.init_node('laser_find_shelf',anonymous=False)
    # Get launch file parameters
    # System
    ROBOT_NAME    = rospy.get_param(param_name="~robot_name", default="car1")
    ROLE          = rospy.get_param(param_name="~role", default="leader")
    FREQUENCY     = rospy.get_param(param_name="~frequency", default="10")
    # Kinematics
    TOW_CAR_LENGTH =rospy.get_param(param_name="~two_car_length", default="0.93")
    SHELF_LEN      =rospy.get_param(param_name="~shelf_length", default="0.73")
    MAX_CIRCLE_RADIUS = rospy.get_param(param_name="~max_circle_radius", default="0.20")
    SEARCH_RADIUS = rospy.get_param(param_name="~search_radius", default="0.55")
    # Tolerance
    SHEFT_LENGTH_TOLERANCE = rospy.get_param(param_name="~sheft_length_tolerance", default="0.1")
    ANGLE_TOLERANCE =  rospy.get_param(param_name="~angle_tolerance", default="0.08")

    SHELF_LEN_DIAGONAL = SHELF_LEN * sqrt(2)
    # Init laser_finder
    LASER_FIND_SHELF = Laser_find_shelf(sheft_length_tolerance = SHEFT_LENGTH_TOLERANCE,
                                        angle_tolerance = ANGLE_TOLERANCE,
                                        max_circle_radius = MAX_CIRCLE_RADIUS,
                                        search_radius = SEARCH_RADIUS,
                                        robot_name = ROBOT_NAME,
                                        role = ROLE) # After combine # TODO Use topic or service 
    
    rate = rospy.Rate(FREQUENCY)
    while not rospy.is_shutdown():
        LASER_FIND_SHELF.get_base_link()
        if LASER_FIND_SHELF.run_once():
            LASER_FIND_SHELF.publish()
        rate.sleep()