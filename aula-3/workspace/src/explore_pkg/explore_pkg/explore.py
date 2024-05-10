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
        LaserScan, '/scan', self.scan_callback, 10)

    timer_period = .5
    self.timer = self.create_timer(timer_period, self.explore_walk)
    self.i = 0

  def explore_walk(self):
    if self.has_obstacle_foward:
      self.stop_and_rotate()
    else:
      self.walk_foward(.2)

    self.i += 1

  def walk_foward(self, x: float = 1.0):
    self.set_vel(x, 0., 0.)
    self.get_logger().info(f'Publishing: "walk_foward", i: {self.i}')

  def scan_callback(self, scan: LaserScan):
    self.get_logger().info(f'mid_dist: {scan.ranges[len(scan.ranges)//2]}')

    regional_angle = 30
    mid_angle = len(scan.ranges) // 2
    front_region = scan.ranges[:int(regional_angle/2)]

    dist_threshold = 1.
    closests = [x for x in front_region if x <= dist_threshold and x != 'inf']
    self.get_logger().info(f'Closests: {closests}')
    self.has_obstacle_foward = len(closests) > 0

  def stop_and_rotate(self, angular: float = PI/4):
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
