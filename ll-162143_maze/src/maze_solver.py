#! /usr/bin/env python
# import dependencies
import rospy
import time  # needed to sleep between every phase
from tf.transformations import euler_from_quaternion  # needed for conversion of position angles
import numpy as np

# import external files
import PID as pid

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

    def equals_point(self, p_point):
        return True if self.x == p_point.x and self.y == p_point.y else False


class MazeSolver:
    # initialise your node, publisher and subscriber as well as some member variables
    def __init__(self):
        # initiate node
        rospy.init_node('maze_solver')

        self.rate = rospy.Rate(5)

        # subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/laserscan', LaserScan, self.laser_callback)
        rospy.Subscriber('/maze_goal_master', Float64MultiArray, self.goal_callback)

        # publishers
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # declare member variables
        self.speed_linear = .3
        self.speed_angular = .03
        self.msg_pose = Pose()
        self.msg_laser = LaserScan()
        self.rate = rospy.Rate(1.5)  # set a publish rate of 1.5 Hz
        # self.memory = set()  # remember visited points TODO and movement decision?
        self.memory = {Point(100, 100)}  # remember visited points TODO and movement decision?
        self.goal = Point()  # goal position desired to be reached

        # various constants incoming
        self.CHECK_STRAIGHT_LASER_DATA = 10  # the number of datasets considered for driving decision
        # command parameters for publish_movement
        self.CMD_LINEAR = 1
        self.CMD_ANGULAR = 2
        self.SAVE_DISTANCE = 1  # minimum distance to closest obstacle

        self.SLEEP_TIME = 1  # sleep duration (sec)

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
        try:
            rospy.loginfo("start")

            while not rospy.is_shutdown():
                ''' phase 1: orient towards goal position '''
                self.turn_to_goal()

                # sleep between phases
                time.sleep(self.SLEEP_TIME)

                ''' phase 2: drive till an obstacle appears '''
                # nothing in sight -> green light
                (compressed, nan_count, min_val) = self.eval_laserdata()
                while nan_count == self.CHECK_STRAIGHT_LASER_DATA:
                    print("no obstacle close, let's go!")
                    self.publish_movement(self.CMD_LINEAR)
                    (compressed, nan_count, min_val) = self.eval_laserdata()

                (compressed, nan_count, min_val) = self.eval_laserdata()
                while min_val > self.SAVE_DISTANCE:
                    print("no obstacle close (closest one is at '{}'), let's go!".format(round(min_val, 3)))
                    self.publish_movement(self.CMD_LINEAR)
                    (compressed, nan_count, min_val) = self.eval_laserdata()

                # in every other situation: halt robot and remember position
                self.publish_movement(0)
                self.remember_position()  # TODO check if remembering position is correct here

                ''' phase 3: overcome obstacle '''
                # turning and driving didn't lead to success -> we have an obstacle here
                self.overcome_obstacle()

            rospy.spin()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("received ROSInterruptException, shutting down maze solver...")
            return False

    def turn_to_goal(self):
        """
        Turns the robot until it is facing towards the goal
        :return:
        """
        rospy.loginfo("turn_to_goal ({},{})".format(self.goal.x, self.goal.y))

        tmp_pnt = Point()
        '''
        ##### Approach for orienting towards a different goal than the one received from the goal subscriber
        add to fct definition: (self, goal=Point())
        if goal.equals_point(tmp_pnt):
            pass
        '''
        while abs(self.goal.x - tmp_pnt.x) < .01 and abs(self.goal.y - tmp_pnt.y) < .01:
            rospy.loginfo("waiting for goal publisher, current goal ({},{})".format(self.goal.x, self.goal.y))
            time.sleep(self.SLEEP_TIME)

        print("turning to goal: ({},{})".format(self.goal.x, self.goal.y))

        # calculate starting position; convert quaternion to euler first
        orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                        self.msg_pose.orientation.z, self.msg_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)

        # calculate with array --> goal angle will be an array itself
        # x = np.array([self.msg_pose.position.x, self.goal[0]])
        # y = np.array([self.msg_pose.position.y, self.goal[1]])
        goal_angle = np.arctan2(self.goal.y, self.goal.x)

        while abs(goal_angle - yaw) > 0.01:
            self.publish_movement(self.CMD_ANGULAR)

            # calc current orientation
            orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                            self.msg_pose.orientation.z, self.msg_pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientations)

            # calc orientation to goal
            goal_angle = np.arctan2(self.goal.y, self.goal.x)
            print("desired: yaw ({}) - goal_angle ({}) = {}".format(round(yaw, 4), round(goal_angle, 4),
                                                                    round(abs(yaw - goal_angle), 4)))

        # turning completed, halt robot
        self.publish_movement(0)

    def eval_laserdata(self):
        """
        TODO
        :return:
        """
        ranges = self.msg_laser.ranges
        print("len laser data = {}".format(len(ranges)))
        while len(ranges) == 0:  # if there is no data from laserscanner: wait till there is some
            rospy.loginfo("no data from laserscanner, waiting to receive some...")
            time.sleep(self.SLEEP_TIME)
        compressed_msg = ranges[len(ranges) / 2 - self.CHECK_STRAIGHT_LASER_DATA / 2:
                                len(ranges) / 2 + self.CHECK_STRAIGHT_LASER_DATA / 2]
        # print("compressed_msg: '{}'".format(compressed_msg))
        nan_count = len(filter(lambda x: np.isnan(x), compressed_msg))
        print("nan_count = {}".format(nan_count))
        # Filter out nan's in order to calc average distance
        if nan_count > 0:
            compressed_msg = filter(lambda x: np.isnan(x) is False, compressed_msg)
        # print("compressed msg: {}".format(compressed_msg))
        # avg = sum(compressed_msg) / float(len(compressed_msg))
        # print("avg: {}".format(avg))
        min_val = min(x for x in compressed_msg if x != 0.0)
        print("min_val: {}".format(min_val))
        return compressed_msg, nan_count, min_val

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
        self.rate.sleep()

    def remember_position(self):
        """
        write current position to list 'memory' and check if position has already been visited (circle detection)
        :return:
        """
        rospy.loginfo("never forget a thing...")

        ''' DEBUG '''
        print("Before adding:")
        self.print_memory()
        ''' DEBUG '''

        tmp_pnt = Point(self.msg_pose.position.x, self.msg_pose.position.y)

        duplicate = False
        for pnt in set(self.memory):
            # check if current point is already known
            if abs(pnt.x - tmp_pnt.x) < .5 and abs(pnt.y - tmp_pnt.y) < .5:
                duplicate = True
                print("duplicate found!!!")
                # self.circle_detection_reaction()
                continue
        if not duplicate:
            # remember current position
            self.memory.add(tmp_pnt)

        ''' DEBUG '''
        print("...and after adding:")
        self.print_memory()
        ''' DEBUG '''

    def overcome_obstacle(self):
        """
        TODO
        :return:
        """
        rospy.loginfo("challenge accepted!")
        ranges = self.msg_laser.ranges
        # Approach: check laser data starting from the center step-by-step till a jump in distances appears.
        # That position means there is some sort of edge, most likely the end of the obstacle.
        # Orient and drive to that position
        for x in range(int(len(ranges) / 2)):
            diff_a = ranges[len(ranges) / 2 - x] - ranges[len(ranges) / 2 - x - 1]
            diff_b = ranges[len(ranges) / 2 + x] - ranges[len(ranges) / 2 + x + 1]
            print("a = {}; b = {}".format(diff_a, diff_b))
            if abs(diff_a) or abs(diff_b) > 1:
                target_difference = diff_a if abs(diff_a) > abs(diff_b) else diff_b
                target_index = ranges.index(target_difference)
                print("target_index = {}".format(target_index))

                # points start at -90 degree, therefore we start with negative values...
                if target_index == len(ranges) / 2:
                    angle = 0
                # ... and switch to positive after half of the points are processed
                elif target_index > len(ranges) / 2:
                    angle = self.msg_laser.angle_max - target_index * self.msg_laser.angle_increment
                else:
                    angle = self.msg_laser.angle_min + target_index * self.msg_laser.angle_increment
                break

    def print_memory(self):
        """
        Helper function for a (at least half way) pretty printing of all visited points stored in memory
        :return:
        """
        point_cloud = "All points (" + str(len(self.memory)) + "): {"
        for pnt in set(self.memory):
            point_cloud += "(" + str(round(pnt.x, 3)) + "," + str(round(pnt.y, 3)) + ")"
        point_cloud += "}"
        print(point_cloud)


if __name__ == "__main__":
    """
    Main, create instance and launch
    """
    maze_runner = MazeSolver()
    maze_runner.start_runner()
