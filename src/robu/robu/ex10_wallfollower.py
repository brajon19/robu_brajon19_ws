import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data #um 

import numpy as np



from enum import IntEnum 

ROBOT_DIRECTION_FRONT_INDEX = 0
ROBOT_DIRECTION_RIGHT_FRONT_INDEX = 315
ROBOT_DIRECTION_RIGHT_INDEX = 270
ROBOT_DIRECTION_RIGHT_REAR_INDEX = 225
ROBOT_DIRECTION_REAR_INDEX = 180
ROBOT_DIRECTION_LEFT_REAR_INDEX =135
ROBOT_DIRECTION_LEFT_INDEX = 90
ROBOT_DIREKTION_LEFT_FRONT_INDEX = 45


#bestandteile der Klasse
class WallFollowerStates(IntEnum):
    WF_STATE_IVALID = -1,
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL = 2,
    WF_STATE_FALLOWWALL = 3
    

#Node
class WallFollower(Node):
    def __init__(self):   #__ steht für private
        super().__init__('WallFollower') #super heist: die klasse darüber
        self.scan_subsciber = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile_sensor_data)

        self.left_dist = 99999999.9 # Initialisiere die Variable auf einen ungültigen Wert
        self.front_dist = 999999999.9
        self.leftfront_dist = 999999999.9
        self.right_dist = 999999999.9
        self.rightfront_dist = 999999999.9
        self.rear_dist = 999999999.9
        self.distances = () #ein leeres array

        self.wallfollower_state = WallFollowerStates.WF_STATE_IVALID

        self.forward_speed_wf_slow = 0.05
        self.forward_speed_wf_fast = 0.10

        self.truning_speed_wf_slow = 0.1
        self.truning_speed_wf_fast = 1.0

        self.dist_trashhold_wf = 0.3
        self.dist_hysteresis_wf = 0.02
        self.vallid_lider_data = False
        self.dist_laser_offset = 0.03

        self.timer = self.create_timer(0.2, self.timer_callback)



    def timer_callback(self):
        if self.vallid_lider_data: #Beim erstmaligen Aufruf liegen noch  eine Lidar Daren vor
            self.fallow_wall()

    def fallow_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if(self.wallfollower_state==WallFollowerStates.WF_STATE_IVALID):
            print("WF_STATE_DETECTWALL")
            self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL



        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DETECTWALL:
            dist_min = min (self.distances)
            if self.front_dist > (dist_min + self.dist_laser_offset): # Dreh Roboter solange auf der x-Achse bis der Mindestabstand erreicht ist
                #Wenn der Roboter kurz vor dem Mindestabstand ist, verringert sich die Drehgeschwindigkeit
                print("Roboter dreht sich")
                if(self.front_dist-dist_min) < 0.2:
                    msg.angular.z = self.truning_speed_wf_slow
                else:
                    msg.angular.z = self.truning_speed_wf_fast

            else:
                print("WF_STATE_DRIVE2WALL")
                self.wallfollower_state = WallFollowerStates.WF_STATE_DRIVE2WALL
                self.cmd_vel_publisher.publish(msg)
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DRIVE2WALL
            fd_thresh =self.dist_thrashhold_wf+self.dist_laser_offset

            forward_speed_wf = self.calc_linear_speed()

            if self.front_dist > (fd_thresh +self.dist_hysteresis_wf):
                msg.linear.x = forward_speed_wf
            elif self.front_dist < (fd_thresh -self.dist_hysteresis_wf):
                msg.linear.x =-forward_speed_wf
            else:
                turn_direction = self.align_front()
                msg.angular.z = self.truning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_ROTATE2WALL")
                    #safe the current distances a input for the state ROTATE2WALL
                    self.wallfollower_state_input_dist = self.distances
                    self.wallfollower_state = WallFollowerStates.WF_STATE_ROTATE2WALL

        elif self.wallfollower_state == WallFollowerStates.WF_STATE_ROTATE2WALL:
            sr = self.wallfollower_state_input_dist[ROBOT_DIRECTION_RIGHT_INDEX]
            if((sr != np.inf) and (abs(self.front_dist - self.dist_laser_offset -sr)>0.05)) or ((self.front_dist != np.inf) and (sr == np.inf)):
                msg.angular.z = -self.truning_speed_wf_fast
            else:
                turn_direction = self.align_left()
                msg.angular.z = self.truning_speed_wf_slow *turn_direction
                if turn_direction == 0:
                    print("WF_STATE_FOLLOWWALL")
                    self.wallfollower_state = WallFollowerStates.WF_STATE_FALLOWWALL

        
        


        self.cmd_vel_publisher.publish(msg)
    def align_front(self):
        fl= self.distances[ROBOT_DIREKTION_LEFT_FRONT_INDEX]
        rl = self.distances[ROBOT_DIRECTION_LEFT_REAR_INDEX]

        if (fl-rl) > self.dist_hysteresis_wf:
            return 1 #turning left
        elif (fl-lf)< self.dist_hysteresis_wf:
            return -1 #turning right
        else:
            return 0#aligned
        
    def align_front(self):
        fl = self.distances[ROBOT_DIREKTION_LEFT_FRONT_INDEX]
        fr = self.distances[ROBOT_DIRECTION_RIGHT_FRONT_INDEX]

        if (fl -fr) > self.dist_hysteresis_wf:
            return 1 #turn left
        elif (fl -fr) < self.dist_hysteresis_wf:
            return -1 #turn right
        else:
            return 0#aligned

    def calc_linear_speed(self):
        fd_thresh = self.dist_thresh_wf+ self.dist_laser_offset
        if self.front_dist > (1.2 * fd_thresh):
            forward_speed_wf = self.forward_speed_wf_fast
        else:
            forward_speed_wf = self.forward_speed_wf_slow

        return forward_speed_wf

    def scan_callback(self, msg): 
        self.left_dist = msg.ranges[ROBOT_DIRECTION_LEFT_INDEX]
        self.front_dist = msg.ranges[ROBOT_DIRECTION_FRONT_INDEX]
        self.leftfront_dist = msg.ranges[ROBOT_DIREKTION_LEFT_FRONT_INDEX]
        self.right_dist = msg.ranges[ROBOT_DIRECTION_RIGHT_INDEX]
        self.rightfront_dist = msg.ranges[ROBOT_DIRECTION_RIGHT_FRONT_INDEX]
        self.rear_dist = msg.ranges[ROBOT_DIRECTION_REAR_INDEX]
        self.distances = msg.ranges
        self.vallid_lider_data = True

        print("ld: %.2f m\n"% self.left_dist,
             "lfd: %.2f m\n"% self.leftfront_dist, 
             "fd: %.2f m\n"% self.front_dist,
             "rfd: %.2f m\n"% self.rightfront_dist,
             "rd: %.2f m\n"% self.right_dist,
             "rrd: %.2f m\n"% self.rear_dist)



def main(args=None):
    rclpy.init(args = args)
    wallfollower= WallFollower()

    rclpy.spin(wallfollower)

    wallfollower.destroy_node()
    rclpy.shutdown()