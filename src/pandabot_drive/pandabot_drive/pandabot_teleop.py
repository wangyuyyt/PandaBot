import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy 

from std_msgs.msg import String


class TeleopJoy(Node):

    def __init__(self):
        super().__init__('pandabot_teleop_joy')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) 

        self.subscription = self.create_subscription(
                Joy, 'joy', self.joy_callback, 10) 

        # Mapping used to convert Joy message to Twist
        self.linear_map = { 'x': 3, 'y': -1, 'z': -1 }
        self.linear_scale_map = { 'x': 0.5, 'y': 0, 'z': 0 } 
        self.angular_map = { 'yaw': 2, 'pitch': -1, 'roll': -1 }
        self.angular_scale_map = { 'yaw': 0.5, 'pitch': 0, 'roll': 0 } 

    def get_linear_value(self, msg, field):
        if field not in self.linear_map:
            return 0
        return msg.axes[self.linear_map[field]] * self.linear_scale_map[field]

    def get_angular_value(self, msg, field):
        if field not in self.angular_map:
            return 0
        return msg.axes[self.angular_map[field]] * self.angular_scale_map[field]

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = self.get_linear_value(msg, 'x')
        twist.linear.y = self.get_linear_value(msg, 'y')
        twist.linear.z = self.get_linear_value(msg, 'z')
        twist.angular.z = self.get_angular_value(msg, 'yaw')
        twist.angular.y = self.get_angular_value(msg, 'pitch')
        twist.angular.x = self.get_angular_value(msg, 'roll')
        self.publisher_.publish(twist)
        #self.get_logger().info('Published')

def main(args=None):
    rclpy.init(args=args)

    publisher = TeleopJoy()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

