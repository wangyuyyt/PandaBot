from geometry_msgs.msg import Twist, TwistStamped
from picar import back_wheels, front_wheels
import picar
import rclpy
from rclpy.node import Node

ROBOT_WHEEL_SEPARATION = 0.115 # Distance between the two wheels in meters
ROBOT_WHEEL_RADIUS = 0.034  # Wheel radius in meters
MAX_SPEED = (2 + ROBOT_WHEEL_SEPARATION) / ( 2 * ROBOT_WHEEL_RADIUS)
print(MAX_SPEED)

class PandabotDriver(Node):

    def __init__(self):
        super().__init__('pandabot_driver')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setup Picar
        picar.setup()
        self.left_wheel = back_wheels.Wheel(27, 4, 'left', debug=False, bus_number=1)
        self.right_wheel = back_wheels.Wheel(17, 5, 'right', debug=False, bus_number=1)

    def map_speed(self, val):
        """
        Map val between [0, MAX_SPEED] to [0, 100]
        """
        return (val * 1.0 / MAX_SPEED) * 100

    def listener_callback(self, msg):
        # Cap values at [-1 .. 1]
        v = max(min(msg.linear.x, 1.0), -1.0)
        w = max(min(msg.angular.z, 1.0), -1.0)

        #v_l = (2 * v - w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);
        #v_r = (2 * v + w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);
        v_r = (v + w) * 100
        v_l = (v - w) * 100

        left_speed = int(abs(v_l))
        right_speed = int(abs(v_r))

        if v_l > 0:
            self.left_wheel.forward()
        else:
            self.left_wheel.backward()

        if v_r > 0:
            self.right_wheel.forward()
        else:
            self.right_wheel.backward()
        self.left_wheel.speed = left_speed
        self.right_wheel.speed = right_speed

def main(args=None):
    rclpy.init(args=args)

    pandabot_driver = PandabotDriver()

    rclpy.spin(pandabot_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pandabot_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
