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
        self.CHECK_STRAIGHT_LASER_DATA = 10
        # command parameters for publish_movement
        self.CMD_LINEAR = 1
        self.CMD_ANGULAR = 2
        self.SAVE_DISTANCE = 5

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
            self.turn_to_goal()
            # sleep to prevent flooding console
            self.rate.sleep()

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
        print("DEBUG: curr pos '{}', \ngoal '{}', yaw '{}', goal_angle '{}'".format(self.msg_pose, self.goal, yaw,
                                                                                    goal_angle))
        while abs(goal_angle - yaw) > 0.05:
            # print("diff: '{}'".format(abs(new_goal-yaw)))
            self.publish_movement(self.CMD_ANGULAR)

            # calc current orientation
            orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                            self.msg_pose.orientation.z, self.msg_pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientations)

            # calc orientation to goal
            goal_angle = np.arctan2(self.goal[1], self.goal[0])
            yaw = round(yaw, 4)
            print("desired: yaw ({}) - goal_angle ({}) = {}".format(yaw, goal_angle, round(abs(yaw - goal_angle), 4)))

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
