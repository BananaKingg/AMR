#! /usr/bin/env python
# import dependencies
import rospy
import time  # needed to sleep between every phase
from tf.transformations import euler_from_quaternion  # needed for conversion of position angles
import numpy as np
from math import atan2, pi

# import messages
from geometry_msgs.msg import Twist, Pose  # command message to publish the velocity, know position part of Odom
from nav_msgs.msg import Odometry  # reading the position of the robot
from sensor_msgs.msg import LaserScan  # know what the bot is seeing
from std_msgs.msg import Float64MultiArray  # listen to goal publisher


class Point:
    def __init__(self, p_x=0, p_y=0):
        self.x = p_x
        self.y = p_y

    def print_point(self):
        print("{},{}".format(self.x, self.y))


class MazeSolver:
    # initialise your node, publisher and subscriber as well as some member variables
    def __init__(self):
        # initiate node
        rospy.init_node('maze_solver')

        self.rate = rospy.Rate(10)

        # subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/laserscan', LaserScan, self.laser_callback)
        rospy.Subscriber('/maze_goal_master', Float64MultiArray, self.goal_callback)

        # publishers
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # declare member variables
        self.speed_linear = .3
        self.speed_angular = .1
        self.msg_pose = Pose()
        self.msg_laser = LaserScan()
        self.rate = rospy.Rate(1.5)  # set a publish rate of 1.5 Hz
        self.memory = set()  # remember visited points TODO and movement decision?
        self.goal = Point()  # goal position desired to be reached
        # self.goal = [44.3, 10]  # goal position desired to be reached # TODO read from publisher

        # various constants incoming
        self.CHECK_STRAIGHT_LASER_DATA = 10  # the number of datasets considered for driving decision
        # command parameters for publish_movement
        self.CMD_LINEAR = 1
        self.CMD_ANGULAR = 2
        self.SAVE_DISTANCE = 1  # minimum distance to closest obstacle

        rospy.logdebug("init complete, waiting for service call...")

    def laser_callback(self, request):
        """
        TODO
        :param request:
        :return:
        """
        self.msg_laser = request

    def odom_callback(self, request):
        """
        callback method for the Odometry: writes content of the message to member variable
        :param request: Odometry message
        :return:
        """
        # only saving the part of the Odometry message that is relevant for the project's objective:
        # position & orientation (both being part of msg_pose)
        self.msg_pose = request.pose.pose

    def goal_callback(self, request):
        """
        TODO
        :param request:
        :return:
        """
        tmp_point = Point(request.data[0], request.data[1])
        self.goal = tmp_point

    def start_runner(self):
        """
        "Main" function of the runner, coordinates the other functions
        :return:
        """
        rospy.loginfo("start")

        while not rospy.is_shutdown():
            # phase 1: orient towards goal position
            # at the beginning, take some time listening to the goal publisher
            tmp_point = Point()  # create (0,0)-Point and compare to goal
            while self.goal.x == tmp_point.x and self.goal.y == tmp_point.y:
                self.rate.sleep()
                rospy.loginfo("waiting for goal publisher...")
            self.turn_to_goal()

            # phase 2: drive (if no obstacle)
            self.remember_position()
            self.drive_forward()

        rospy.spin()

    def turn_to_goal(self):
        """
        Turns the robot until it is facing towards the goal
        :return:
        """
        rospy.loginfo("turn_to_goal ({},{})".format(self.goal.x, self.goal.y))

        # calculate starting position; convert quaternion to euler first
        orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                        self.msg_pose.orientation.z, self.msg_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)

        # calculate with array --> goal angle will be an array itself
        # x = np.array([self.msg_pose.position.x, self.goal[0]])
        # y = np.array([self.msg_pose.position.y, self.goal[1]])
        goal_angle = np.arctan2(self.goal.y, self.goal.x)

        while abs(goal_angle - yaw) > 0.05:
            self.publish_movement(self.CMD_ANGULAR)

            # calc current orientation
            orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                            self.msg_pose.orientation.z, self.msg_pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientations)

            # calc orientation to goal
            goal_angle = np.arctan2(self.goal.y, self.goal.x)
            print("desired: yaw ({}) - goal_angle ({}) = {}".format(round(yaw, 4), round(goal_angle, 4),
                                                                    round(abs(yaw - goal_angle), 4)))

            self.rate.sleep()

        # turning completed, halt robot
        self.publish_movement(0)

    def drive_forward(self):
        """
        Drive robot forward if no obstacle is in it's way
        :return:
        """
        while True:
            # sleep to prevent flooding console
            self.rate.sleep()

            # at this point check only number of CHECK_STRAIGHT_LASER_DATA elements from msg
            compressed_msg = self.msg_laser.ranges[len(self.msg_laser.ranges) / 2 - self.CHECK_STRAIGHT_LASER_DATA / 2:
                                                   len(self.msg_laser.ranges) / 2 + self.CHECK_STRAIGHT_LASER_DATA / 2]
            rospy.logdebug("compressed_msg: '{}'".format(compressed_msg))
            nan_count = len(filter(lambda x: np.isnan(x), compressed_msg))
            rospy.logdebug("nan_count = {}".format(nan_count))

            # nothing in sight -> green light
            if nan_count == self.CHECK_STRAIGHT_LASER_DATA:
                self.publish_movement(self.CMD_LINEAR)
                continue

            # Filter out nan's in order to calc average distance
            if nan_count > 0:
                data = filter(lambda x: np.isnan(x) is False, compressed_msg)
            else:
                data = compressed_msg
            rospy.logdebug("data: {}".format(data))

            print("num elements in data: {}".format(len(data)))
            mini = min(data)
            rospy.logdebug("mini: {}".format(mini))
            if mini > self.SAVE_DISTANCE:
                self.publish_movement(self.CMD_LINEAR)
                continue

            # if avg < self.SAVE_DISTANCE:  # for every other situation: halt robot
            self.publish_movement(0)

    def publish_movement(self, p_direction):
        """
        Publishes a velocity movement command to the robot
        :param p_direction: the kind of movement that is desired: linear/angular/halt (default)
        :return:
        """
        command = Twist()
        if p_direction == self.CMD_LINEAR:
            command.linear.x = self.speed_linear
            command.angular.z = 0  # ensure the bot is moving straight only
        elif p_direction == self.CMD_ANGULAR:
            command.linear.x = 0  # ensure the bot is moving angular only
            command.angular.z = self.speed_angular
        else:
            # default: stop the bot
            command.linear.x = 0
            command.angular.z = 0
        self.vel_pub.publish(command)

    def remember_position(self):
        """
        write current position to list 'memory' and check if position has already been visited (circle detection)
        :return:
        """
        rospy.loginfo("never forget a thing...")

        # remember current position
        tmp_pnt = Point(self.msg_pose.position.x, self.msg_pose.position.y)
        self.memory.add(tmp_pnt)

        # print("memory: {}".format(self.memory))

        point_cloud = []
        for pnt in set(self.memory):
            point_cloud.append(pnt)
            # if abs(pnt.x - tmp_pnt.x) < 1 and abs(pnt.y - tmp_pnt.y) < 1:
                # self.overcome_obstacle()
        print("all points: ".format(point_cloud))

    def overcome_obstacle(self):
        """
        TODO
        :return:
        """
        rospy.loginfo("challenge accepted!")


if __name__ == "__main__":
    """
    Main, create instance and launch
    """
    maze_runner = MazeSolver()
    maze_runner.start_runner()
