import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi as PI
from sensor_msgs.msg import LaserScan


class Publisher(Node):

  def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_twist = self.create_publisher(Twist, '/cmd_vel', 10)
    self.subscription_scan = self.create_subscription(
        LaserScan, '/scan', self.listener_callback, 10)
    self.laser_scan = None

    timer_period = .5
    self.timer = self.create_timer(timer_period, self.explore_walk)
    self.i = 0

  def explore_walk(self):
    if self.has_obstacle_foward():
      self.stop_and_rotate()
    else:
      self.walk_foward(.2)

    self.i += 1

  def listener_callback(self, msg: LaserScan):
    self.laser_scan = msg

  def walk_foward(self, x: float = 1.0):
    self.set_vel(x, 0., 0.)
    self.get_logger().info(f'Publishing: "walk_foward", i: {self.i}')

  def has_obstacle_foward(self, threshold: float = 1.0):
    if self.laser_scan is None:
      return False

    closest = min(self.laser_scan.ranges)
    if closest < threshold:
      min_index = self.laser_scan.ranges.index(closest)
      angle = self.laser_scan.angle_min + min_index * self.laser_scan.angle_increment
      mid_angle = (self.laser_scan.angle_min + self.laser_scan.angle_max) / 2
      if abs(angle - mid_angle) < 2:
        return True

  def stop_and_rotate(self, angular: float = PI/2):
    self.set_ang(angular)
    self.get_logger().info(f'Publishing: "stop_and_rotate", i: {self.i}')

  def set_vel(self, x: float, y: float, z: float):
    msg = Twist()
    msg.linear.x = float(x)
    msg.linear.y = float(y)
    msg.linear.z = float(z)
    self.publisher_twist.publish(msg)

  def set_ang(self, angular: float):
    msg = Twist()
    msg.linear.x = .0
    msg.linear.y = .0
    msg.linear.z = .0
    msg.angular.z = float(angular)
    self.publisher_twist.publish(msg)


def main(args=None):
  rclpy.init(args=args)

  minimal_publisher = Publisher()

  rclpy.spin(minimal_publisher)

  minimal_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
