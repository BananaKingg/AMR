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


class MazeSolver:
    # initialise your node, publisher and subscriber as well as some member variables
    def __init__(self):
        # initiate node
        rospy.init_node('maze_solver')

        self.rate = rospy.Rate(10)

        # subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/laserscan', LaserScan, self.laser_callback)

        # publishers
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # declare member variables
        self.speed_linear = .3
        self.speed_angular = .1
        self.msg_pose = Pose()
        self.msg_laser = LaserScan()
        self.rate = rospy.Rate(1.5)  # set a publish rate of 1.5 Hz
        self.memory = [0][0]  # remember visited points TODO and movement decision?
        self.goal = [44.3, 10]  # goal position desired to be reached # TODO read from publisher

        # various constants incoming
        self.CHECK_STRAIGHT_LASER_DATA = 10  # the number of datasets considered for driving decision
        # command parameters for publish_movement
        self.CMD_LINEAR = 1
        self.CMD_ANGULAR = 2
        self.SAVE_DISTANCE = 1

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
        self.goal = request

    def start_runner(self):
        """
        TODO
        :return:
        """
        rospy.loginfo("start")

        while not rospy.is_shutdown():
            # orient towards goal position
            self.turn_to_goal()

            # if nothing is in the way, start driving
            # at this point check only number of CHECK_STRAIGHT_LASER_DATA elements from msg
            while True:
                # sleep to prevent flooding console
                self.rate.sleep()

                compressed_msg = self.msg_laser.ranges[len(self.msg_laser.ranges) / 2 - self.CHECK_STRAIGHT_LASER_DATA / 2:
                                                       len(self.msg_laser.ranges) / 2 + self.CHECK_STRAIGHT_LASER_DATA / 2]
                print("compressed_msg: '{}'".format(compressed_msg))
                nan_count = len(filter(lambda x: np.isnan(x), compressed_msg))
                print("nan_count = {}".format(nan_count))

                # nothing in sight -> green light
                if nan_count == self.CHECK_STRAIGHT_LASER_DATA:
                    self.publish_movement(self.CMD_LINEAR)
                    continue

                # Filter out nan's in order to calc average distance
                if nan_count > 0:
                    data = filter(lambda x: np.isnan(x) is False, compressed_msg)
                else:
                    data = compressed_msg
                ''' original'''
                # data = filter(lambda x: np.isnan(x) is False, compressed_msg)
                ''' end original'''
                print("data: {}".format(data))
                avg = sum(data) / float(len(data))
                if avg > self.SAVE_DISTANCE:
                    self.publish_movement(self.CMD_LINEAR)
                    continue

                # if avg < self.SAVE_DISTANCE:  # for every other situation: halt robot
                break

        rospy.spin()

    def turn_to_goal(self):
        """
        Turns the robot until it is facing towards the goal
        :return:
        """
        # calculate starting position; convert quaternion to euler first
        orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                        self.msg_pose.orientation.z, self.msg_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)

        # calculate with array --> goal angle will be an array itself
        # x = np.array([self.msg_pose.position.x, self.goal[0]])
        # y = np.array([self.msg_pose.position.y, self.goal[1]])
        goal_angle = np.arctan2(self.goal[1], self.goal[0])

        while abs(goal_angle - yaw) > 0.05:
            self.publish_movement(self.CMD_ANGULAR)

            # calc current orientation
            orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                            self.msg_pose.orientation.z, self.msg_pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientations)

            # calc orientation to goal
            goal_angle = np.arctan2(self.goal[1], self.goal[0])
            print("desired: yaw ({}) - goal_angle ({}) = {}".format(round(yaw, 4), round(goal_angle, 4),
                                                                    round(abs(yaw - goal_angle), 4)))

            self.rate.sleep()

        # turning completed, halt robot
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


if __name__ == "__main__":
    """
    Main, create instance and launch
    """
    maze_runner = MazeSolver()
    maze_runner.start_runner()
