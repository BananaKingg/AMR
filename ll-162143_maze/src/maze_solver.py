#! /usr/bin/env python
# import dependencies
import rospy
import time  # needed to sleep between every phase
from tf.transformations import euler_from_quaternion  # needed for conversion of position angles
import numpy as np
from math import sin, cos

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
        return True if abs(self.x - p_point.x) < 0.1 and abs(self.y - p_point.y) < 0.1 else False


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
        self.speed_angular = .1
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
        self.CMD_ANGULAR_LEFT = 2
        self.CMD_ANGULAR_RIGHT = 3
        self.SAVE_DISTANCE_TOTAL = .2  # minimum distance to closest obstacle (overall)
        self.SAVE_DISTANCE_FRONT = 1.5  # minimum distance to closest obstacle (at front)

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
                (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()
                driven = False
                while min_val_total > self.SAVE_DISTANCE_TOTAL and min_val_front > self.SAVE_DISTANCE_FRONT:
                    print("no obstacle close (closest total one is at '{}', front '{}'), let's go!".format(
                        round(min_val_total, 3), round(min_val_front, 3)))
                    print("current position ({},{})".format(round(self.msg_pose.position.x, 4),
                                                            round(self.msg_pose.position.y, 4)))
                    self.publish_movement(self.CMD_LINEAR)
                    (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()
                    driven = True

                # in every other situation: halt robot and remember position
                self.publish_movement(0)
                if driven:
                    self.remember_position()  # TODO check if remembering position is correct here

                ''' phase 3: overcome obstacle '''
                # turning and driving didn't lead to success -> we have an obstacle here
                self.overcome_obstacle()

            rospy.spin()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("received ROSInterruptException, shutting down maze solver...")
            return False

    def turn_to_goal(self, p_goal=Point()):
        """
        Turns the robot until it is facing towards the goal
        :param p_goal: optional parameter, allows to face a different point than the one received from the subsrcriber
        :return:
        """
        print("received goal ({},{})".format(round(p_goal.x, 3), round(p_goal.y, 3)))

        tmp_pnt = Point()
        # no point has been passed to function, use goal from publisher
        if p_goal.equals_point(tmp_pnt):
            while self.goal.equals_point(tmp_pnt):
                # abs(self.goal.x - tmp_pnt.x) < .01 and abs(self.goal.y - tmp_pnt.y) < .01:
                rospy.loginfo("waiting for goal publisher, current goal ({},{})".format(self.goal.x, self.goal.y))
                time.sleep(self.SLEEP_TIME)
            goal = self.goal
        # custom goal has been passed via parameter
        else:
            goal = p_goal

        rospy.loginfo("turn_to_goal: ({},{})".format(round(goal.x, 3), round(goal.y, 3)))

        # calculate starting position; convert quaternion to euler first
        yaw = self.get_orientation()
        goal_angle = np.arctan2(goal.y, goal.x)

        # keep turning until the desired angle is reached
        while abs(goal_angle - yaw) > 0.05:
            self.publish_movement(self.CMD_ANGULAR_LEFT)

            # calc current orientation
            yaw = self.get_orientation()
            # calc orientation to goal
            goal_angle = np.arctan2(goal.y, goal.x)
            print("desired: yaw ({}) - goal_angle ({}) = {}".format(round(yaw, 4), round(goal_angle, 4),
                                                                    round(abs(yaw - goal_angle), 4)))

        # turning completed, halt robot
        self.publish_movement(0)

    def get_orientation(self):
        """
        TODO
        :return:
        """
        # calc current orientation
        orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                        self.msg_pose.orientation.z, self.msg_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        return yaw

    def eval_laserdata(self):
        """
        TODO
        :return:
        """
        ranges = self.msg_laser.ranges
        # self.SAVE_DISTANCE_FRONT = self.SAVE_DISTANCE_TOTAL
        while len(ranges) == 0:  # if there is no data from laserscanner: wait till there is some
            rospy.loginfo("no data from laserscanner, waiting to receive some...")
            time.sleep(self.SLEEP_TIME)
        compressed_msg_total = ranges[::12]  # use 40 instead of 360 scan results to gain overview  TODO necessary?
        compressed_msg_front = ranges[len(ranges) / 2 - self.CHECK_STRAIGHT_LASER_DATA / 2:
                                      len(ranges) / 2 + self.CHECK_STRAIGHT_LASER_DATA / 2]

        min_val_total = min(x for x in compressed_msg_total if x != 0.0)
        min_val_front = min(x for x in compressed_msg_front if x != 0.0)
        print("len laser data = {}; min_val_total = {}; min_val_front = {}".format(len(ranges),
                    round(min_val_total, 4), round(min_val_front, 4)))
        return compressed_msg_total, compressed_msg_front, min_val_total, min_val_front

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
        elif p_direction == self.CMD_ANGULAR_LEFT or p_direction == self.CMD_ANGULAR_RIGHT:
            command.linear.x = 0  # ensure the bot is moving angular only
            command.angular.z = self.speed_angular if p_direction == self.CMD_ANGULAR_LEFT else -self.speed_angular
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
            if abs(pnt.x - tmp_pnt.x) < .3 and abs(pnt.y - tmp_pnt.y) < .3:
                duplicate = True
                print("\n########## duplicate found!!! ##########\n")
                self.overcome_obstacle_with_circle()
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
        rospy.loginfo("overcome obstacle")
        ranges = self.msg_laser.ranges
        for x in range(len(ranges) - 1):
            diff = abs(ranges[x] - ranges[x + 1])
            if diff > .1:
                print("val(index) = {}".format(ranges[x]))
                print("val(index + 1) = {}".format(ranges[x + 1]))
                print("val(index - 1) = {}".format(ranges[x - 1]))
                distance = ranges[x + 1]

                # calc current orientation
                yaw = self.get_orientation()

                # use some maths to determine the coordinates of the destination point: we have a straight triangle
                # with the distance from the current position to the destination as hypotenuse (c = distance)
                a = sin(yaw) * distance
                b = cos(yaw) * distance

                target = Point(self.msg_pose.position.x + a, self.msg_pose.position.y + b)
                print("calculated new target at ({},{})".format(round(target.x, 3), round(target.y, 3)))
                self.turn_to_goal(target)

                ''' phase 2: drive till an obstacle appears '''
                # nothing in sight -> green light
                (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()
                driven = False
                while min_val_total > self.SAVE_DISTANCE_TOTAL and min_val_front > (self.SAVE_DISTANCE_FRONT - .6):  # .5
                    print("no obstacle close (closest total one is at '{}', front '{}'), let's go!".format(
                        round(min_val_total, 3), round(min_val_front, 3)))
                    print("current position ({},{})".format(round(self.msg_pose.position.x, 4),
                                                            round(self.msg_pose.position.y, 4)))
                    self.publish_movement(self.CMD_LINEAR)
                    (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()
                    driven = True

                # in every other situation: halt robot and remember position
                self.publish_movement(0)
                if driven:
                    self.remember_position()  # TODO check if remembering position is correct here

    def overcome_obstacle_with_circle(self):
        """
        TODO
        :return:
        """
        # orient to goal
        self.turn_to_goal()
        sum_angles_turned = 0

        while sum_angles_turned < 360:
            yaw1 = self.get_orientation()

            self.publish_movement(self.CMD_ANGULAR_RIGHT)

            # calc current orientation
            yaw2 = self.get_orientation()
            sum_angles_turned += (yaw2 - yaw1)

            # invert condition for driving: turn until driving is allowed again
            (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()
            while min_val_total < self.SAVE_DISTANCE_TOTAL and min_val_front < (self.SAVE_DISTANCE_FRONT - .5):
                print("turning till driving")
                print("no obstacle close (closest total one is at '{}', front '{}'), let's go!".format(
                    round(min_val_total, 3), round(min_val_front, 3)))
                self.publish_movement(self.CMD_ANGULAR_RIGHT)
                (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()

            # now that condition for driving is fulfilled, do so
            (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()
            while min_val_total > self.SAVE_DISTANCE_TOTAL and min_val_front > (self.SAVE_DISTANCE_FRONT - .5):
                print("driving till turning")
                print("no obstacle close (closest total one is at '{}', front '{}'), let's go!".format(
                    round(min_val_total, 3), round(min_val_front, 3)))
                print("current position ({},{})".format(round(self.msg_pose.position.x, 4),
                                                        round(self.msg_pose.position.y, 4)))
                self.publish_movement(self.CMD_LINEAR)
                (compressed_total, compressed_front, min_val_total, min_val_front) = self.eval_laserdata()

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
