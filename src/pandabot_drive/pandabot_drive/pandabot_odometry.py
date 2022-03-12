import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from picar import back_wheels, front_wheels
import picar
import rclpy
from tf2_ros import TransformBroadcaster
import tf_transformations
from rclpy.node import Node
from threading import Thread

import RPi.GPIO as GPIO

ROBOT_WHEEL_SEPARATION = 0.305 # Distance between the two wheels in meters
ROBOT_WHEEL_RADIUS = 0.034  # Wheel radius in meters
LEFT_ENCODER_PIN = 6
RIGHT_ENCODER_PIN = 5
ODOMETRY_PERIOD = 0.2
ODOMETRY_FRAME = "odom"
ODOMETRY_CHILD_FRAME = "base_footprint"

class PandabotOdometry(Node):

    def __init__(self):
        super().__init__('pandabot_odometry')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) 
        self.odom_bd = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setup Picar
        picar.setup()
        self.left_wheel = back_wheels.Wheel(27, 4, 'left', debug=False, bus_number=1)
        self.right_wheel = back_wheels.Wheel(17, 5, 'right', debug=False, bus_number=1)

        # wheel directions. 0 means stop, 1 means forward, -1 means backward
        self.left_direction = 0
        self.right_direction = 0

        # set timer
        self.encoder_period = 0.02
        self.encoder_timer = self.create_timer(self.encoder_period, self.encoder_callback)
        self.odometry_period = ODOMETRY_PERIOD
        self.odometry_timer = self.create_timer(self.odometry_period, self.odometry_callback)

        # set up encoder
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEFT_ENCODER_PIN, GPIO.IN)
        GPIO.setup(RIGHT_ENCODER_PIN, GPIO.IN)
        
        # initialize encoder
        self.left_encoder = GPIO.input(LEFT_ENCODER_PIN)
        self.right_encoder = GPIO.input(RIGHT_ENCODER_PIN)
   
        # counter for left wheel speed
        self.left_counter_prev = 0
        self.left_counter = 0
        self.left_prev = 0

        # counter for right wheel speed
        self.right_counter_prev = 0
        self.right_counter = 0
        self.right_prev = 0

        # position
        self.x = 0
        self.y = 0
        self.th = 0

        self.v = 0
        self.w = 0

    def twist_callback(self, msg):
        """ Set direction for left and right wheel from Twist
        """

        # Cap values at [-1 .. 1]
        v = max(min(msg.linear.x, 1.0), -1.0)
        w = max(min(msg.angular.z, 1.0), -1.0)

        if v - w > 0:
            self.left_direction = 1
        elif v == w:
            self.left_direction = 0
        else:
            self.left_direction = -1

        if v + w > 0:
            self.right_direction = 1
        elif v + w == 0:
            self.right_direction = 0
        else:
            self.right_direction = -1

    def encoder_callback(self):
        # Left wheel
        value = GPIO.input(LEFT_ENCODER_PIN)
        if self.left_prev == 0 and value == 1:
            self.left_counter += 1
        self.left_prev = value
        # Right wheel
        value = GPIO.input(RIGHT_ENCODER_PIN)
        if self.right_prev == 0 and value == 1:
            self.right_counter += 1
        self.right_prev = value

    def calculate_odometry(self):
        wheel_l = 1.0 * (self.left_counter - self.left_counter_prev) / 20 * 2 * math.pi * ROBOT_WHEEL_RADIUS
        self.left_counter_prev = self.left_counter
        wheel_r = 1.0 * (self.right_counter - self.right_counter_prev) / 20 * 2 * math.pi * ROBOT_WHEEL_RADIUS
        self.right_counter_prev = self.right_counter

        if self.left_direction == -1:
            wheel_l = -wheel_l
        if self.right_direction == -1:
            wheel_r = -wheel_r

        delta_s = (wheel_l + wheel_r) / 2
        delta_th = (wheel_r - wheel_l) / ROBOT_WHEEL_SEPARATION

        dx = delta_s * math.cos(self.th + (delta_th)/2.0)
        dy = delta_s * math.sin(self.th + (delta_th)/2.0)
        self.x += dx
        self.y += dy
        self.th = (self.th + delta_th) % (2 * math.pi)

        self.v = delta_s / ODOMETRY_PERIOD
        self.w = delta_th / ODOMETRY_PERIOD

    def publish_odometry(self):
        current_time = self.get_clock().now().to_msg()

        # Publish odometry topic
        odom_msg = Odometry()

        odom_msg.header.frame_id = ODOMETRY_FRAME
        odom_msg.child_frame_id = ODOMETRY_CHILD_FRAME
        odom_msg.header.stamp = current_time

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom_msg)

        # Publish tf
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = ODOMETRY_FRAME
        t.child_frame_id = ODOMETRY_CHILD_FRAME

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.odom_bd.sendTransform(t)

    def odometry_callback(self):
        self.calculate_odometry()
        self.publish_odometry()

def main(args=None):
    rclpy.init(args=args)

    pandabot_odometry = PandabotOdometry()

    rclpy.spin(pandabot_odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pandabot_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
