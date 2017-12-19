#! /usr/bin/env python
# import dependencies
import rospy
import time  # needed to sleep between every phase
from tf.transformations import euler_from_quaternion  # needed for conversion of position angles

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

            # declare member variables
            self.speed_linear = .3
            self.speed_angular = .2
            self.msg_pose = Pose()
            self.msg_laser = LaserScan()
            self.rate = rospy.Rate(1.5)  # set a publish rate of 2 Hz
            # command parameters for publish_movement
            self.CMD_HALT = 0

            rospy.logdebug("init complete, waiting for service call...")

            rospy.spin()  # maintain the service open.
        except KeyboardInterrupt:
            print("Shutting down multi square mover...")

    def laser_callback(self, request):
        """
        TODO
        :param request:
        :return:
        """
        # TODO
        self.msg_laser = request

    def odom_callback(self, request):
        """
        callback method for the Odometry: writes content of the message to member variable
        :param request: Odometry message
        :return:
        """
        # only saving the part of the Odometry message that is relevant for the project's objective:
        # position & orientation
        self.msg_pose = request.pose.pose

    def move_robot(self, p_service_msg):
        """
        Here goes all the logic. Move the turtlebot in a square.
        Command: Twist Message published to mobile_base/commands/velocity
        Verify movement/positioning: Read data from Odometry
        Every "move" consists of two actions: drive p_service_msg.radius + turn 90 degrees
        :param p_service_msg: contains the values radius (float64), repetitions (int32) and success (bool)
        :return: True - if everything worked as supposed to; False - else
        """
        try:
            rospy.loginfo("entered move_robot")

            self.rate.sleep()  # wait a moment to get a more exact position

            # halt bot for a short moment before entering next phase, avoiding unexpected movement
            time.sleep(2)

        except KeyboardInterrupt:
            rospy.logwarn("received KeyboardInterrupt, shutting down square mover...")
            return False

    def publish_movement(self, p_direction):
        """
        Publishes a velocity movement command to the robot
        :param p_direction: the kind of movement that is desired: linear/angular/halt (default)
        :return:
        """
        command = Twist()
        if True:
            command.linear.x = self.speed_linear
            command.angular.z = 0  # ensure the bot is moving straight only
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

