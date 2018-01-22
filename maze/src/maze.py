#! /usr/bin/env python
# import dependencies
import rospy
import time  # needed to sleep between every phase
import numpy as np
from tf.transformations import euler_from_quaternion  # needed for conversion of position angles

# import services
from maze.srv import MazeSrvMsg, MazeSrvMsgResponse

# import messages
from geometry_msgs.msg import Twist, Pose  # command message to publish the velocity, know position part of Odom
from nav_msgs.msg import Odometry  # reading the position of the robot
from sensor_msgs.msg import LaserScan  # know what the bot is seeing


class MazeSolver:
    def __init__(self):
        try:
            rospy.loginfo("entered init")
            # initiate node
            rospy.init_node('maze_solver')

            # initiate subscriber
            # TODO
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
            rospy.Subscriber('/laserscan', LaserScan, self.laser_callback)

            # initiate publisher
            self.pub_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

            # rospy service
            rospy.service('/maze_solver_service', MazeSrvMsg, self.service_callback)

            # declare member variables
            self.speed_linear = .3
            self.speed_angular = .2
            self.msg_pose = Pose()
            self.msg_laser = LaserScan()
            self.rate = rospy.Rate(1.5)  # set a publish rate of 1.5 Hz
            self.memory = [0][0]  # remember visited points and movement decision
            self.goal = [44.3, 10]  # goal position desired to be reached # TODO read from publisher

            # various constants incoming
            self.CHECK_STRAIGHT_LASER_DATA = 10
            # command parameters for publish_movement
            self.CMD_LINEAR = 1
            self.CMD_ANGULAR = 2
            self.SAVE_DISTANCE = 5

            rospy.logdebug("init complete, waiting for service call...")

            rospy.spin()  # maintain the service open.
        except KeyboardInterrupt:
            print("Shutting down maze runner...")

    def service_callback(self, p_service_msg):
        """
        Callback of the service that starts the maze solver
        :param p_service_msg: contains goal (TODO) and success (bool)
        :return: MyServiceResponse() - sets the success (bool) variable
        """
        rospy.loginfo("entered service_callback")
        rospy.logdebug("service callback message: '{}'".format(p_service_msg))

        # write the target position  # TODO check data type
        self.goal = (p_service_msg.x, p_service_msg.y)

        my_response = MazeSrvMsgResponse()
        my_response.success = self.move_robot(p_service_msg)
        return my_response

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

    def move_robot(self, p_service_msg):
        """
        TODO adjust copied comment
        Here goes all the logic. Move the turtlebot in a square.
        Command: Twist Message published to mobile_base/commands/velocity
        Verify movement/positioning: Read data from Odometry
        Every "move" consists of two actions: drive p_service_msg.radius + turn 90 degrees
        :param p_service_msg: contains the values radius (float64), repetitions (int32) and success (bool)
        :return: True - if everything worked as supposed to; False - else
        """
        try:
            rospy.loginfo("entered move_robot")

            rospy.logdebug("received message: '{}'".format(p_service_msg))

            # halt bot for a short moment before entering next phase, avoiding unexpected movement
            time.sleep(self.rate)

            # starting phase: wall follower
            # step 1: move till wall is reached

            while True:
                # at this point check only number of CHECK_STRAIGHT_LASER_DATA elements from msg
                compressed_msg = self.msg_laser[len(self.msg_laser / 2) - self.CHECK_STRAIGHT_LASER_DATA / 2:
                                                len(self.msg_laser / 2) - self.CHECK_STRAIGHT_LASER_DATA / 2]
                nan_count = len(filter(lambda x: np.isnan(x), compressed_msg))

                if nan_count == self.CHECK_STRAIGHT_LASER_DATA:
                    self.publish_movement(self.CMD_LINEAR)
                    continue

                # Filter out nan's in order to calc average distance
                data = filter(lambda x: np.isnan(x) is False, compressed_msg)
                avg = sum(data) / float(len(data))
                if avg > self.SAVE_DISTANCE:
                    # determine desired orientation
                    self.turn_to_goal()
                    self.publish_movement(self.CMD_LINEAR)
                    continue

                if avg < self.SAVE_DISTANCE:
                    break

                # start moving straight till running into an obstacle/wall

        except KeyboardInterrupt:
            rospy.logwarn("received KeyboardInterrupt, shutting down maze runner...")
            return False

    def turn_to_goal(self):
        """
        TODO
        :return:
        """
        origin = self.msg_pose
        # calculate starting position; convert quaternion to euler first
        orientations = [self.msg_pose.orientation.x, self.msg_pose.orientation.y,
                        self.msg_pose.orientation.z, self.msg_pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        helping_angle = yaw
        starting_point = [origin.position.x, origin.position.y]
        goal_point = [self.goal.x, self.goal.y]
        angle1 = np.arctan(*starting_point[::-1])
        angle2 = np.arctan(*goal_point[::-1])
        goal_angle = np.rad2deg((angle1 - angle2) % (2 * np.pi))

    def circle_detection(self):
        """
        TODO
        :return:
        """
        pass

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
        self.pub_vel.publish(command)


if __name__ == "__main__":
    """
    Main
    Do some preparations, then launch things up
    """
    rospy.loginfo("entered main")
    maze = MazeSolver()
