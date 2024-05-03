import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_twist = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.circle_walk)
        self.i = 0


    def circle_walk(self):
        msg = Twist()
        msg.linear.x = 1.0 #velocidade na direcao x
        msg.linear.y = 0.0 #velocidade na direcao y
        msg.linear.z = 0.0 #velocidade na direcao z
        msg.angular.z = 0.5 #velocidade angular
        self.publisher_twist.publish(msg)
        self.get_logger().info('Publishing: "square walk"')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()