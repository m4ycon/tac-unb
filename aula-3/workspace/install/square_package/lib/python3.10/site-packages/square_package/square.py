import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi as PI


class MinimalPublisher(Node):

  def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_twist = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    timer_period = 2
    self.timer = self.create_timer(timer_period, self.square_walk)
    self.i = 0

  def square_walk(self):
    if self.i == 1:
      self.walk_foward(0.)
      self.rotate(PI/2)
    elif self.i == 2:
      self.walk_foward(2.)
    else:
      self.i = 0

    self.get_logger().info(f'Publishing: "square walk", i: {self.i}')
    self.i += 1

  def walk_foward(self, x: float = 1.0):
    self.set_vel(x, 0., 0.)

  def rotate(self, angular: float = PI/2):
    self.set_ang(angular)

  def set_vel(self, x: float, y: float, z: float):
    msg = Twist()
    msg.linear.x = float(x)
    msg.linear.y = float(y)
    msg.linear.z = float(z)
    self.publisher_twist.publish(msg)

  def set_ang(self, angular: float):
    msg = Twist()
    msg.angular.z = float(angular)
    self.publisher_twist.publish(msg)


def main(args=None):
  rclpy.init(args=args)

  minimal_publisher = MinimalPublisher()

  rclpy.spin(minimal_publisher)

  minimal_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
