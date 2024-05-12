import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
import math

PI = math.pi


class Publisher(Node):

  def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
    self.subscription_scan = self.create_subscription(
        LaserScan, '/scan', self.scan_callback, 10)
    self.subscription_odom = self.create_subscription(
        Odometry, '/odom', self.odom_callback, 10)

    self.i = 0
    self.scan = LaserScan()
    self.actual_pos = Point()
    self.actual_ori = Quaternion()
    self.destiny = Point()

    self.has_obstacle_foward = False
    self.turn_left = False
    self.should_adjust = False
    self.arrived = False

    timer_period = .1
    self.timer = self.create_timer(timer_period, self.run)

  def run(self):
    if self.arrived:
      self.set_vel(0., 0., 0.)
      self.set_ang(0.)
      self.get_logger().info(
          f'Arrived ({self.actual_pos.x}, {self.actual_pos.y})')
      return
    elif self.has_obstacle_foward or self.should_adjust:
      self.stop_and_rotate()
    else:
      self.walk_foward(.2)

    self.get_logger().info(
        f'Actual position: {self.actual_pos.x}, {self.actual_pos.y}, {self.actual_pos.z}')
    self.get_logger().info(
        f'Actual orientation: {self.actual_ori.x}, {self.actual_ori.y}, {self.actual_ori.z}, {self.actual_ori.w}')

    self.i += 1

  def scan_callback(self, scan: LaserScan):
    self.scan = scan

    regional_ang = 30
    mid_ang = -90
    front_region = self.get_range_interval(
        mid_ang - regional_ang, mid_ang + regional_ang)

    dist_threshold = .5
    closests = [x for x in front_region if x <= dist_threshold and x != 'inf']
    self.has_obstacle_foward = len(closests) > 0

  def odom_callback(self, odom: Odometry):
    self.actual_pos = odom.pose.pose.position
    self.actual_ori = odom.pose.pose.orientation

    co = self.actual_pos.x - self.destiny.x
    ca = self.actual_pos.y - self.destiny.y
    hip = math.sqrt(co**2 + ca**2)
    # cos = co / hip
    # sin = ca / hip
    tan = ca / co

    self.turn_left = tan - self.actual_ori.z > 0
    self.should_adjust = abs(tan - self.actual_ori.z) > self.ang_to_rad(10)
    self.arrived = hip < .05

  def walk_foward(self, x: float = 1.0):
    self.set_vel(x, 0., 0.)
    self.get_logger().info(f'Publishing: "walk_foward", i: {self.i}')

  def stop_and_rotate(self, angular: float = 45):
    self.set_vel(0., 0., 0.)
    self.set_ang(angular if self.turn_left else -angular)
    self.get_logger().info(f'Publishing: "stop_and_rotate", i: {self.i}')

  def set_vel(self, x: float, y: float, z: float):
    msg = Twist()
    msg.linear.x = float(x)
    msg.linear.y = float(y)
    msg.linear.z = float(z)
    self.publisher_vel.publish(msg)

  def set_ang(self, angular: float):
    msg = Twist()
    msg.angular.z = float(self.ang_to_rad(angular))
    self.publisher_vel.publish(msg)

  def set_destiny(self, x: float, y: float, z: float = 0.):
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    self.destiny = point

  def get_range_interval(self, start_ang: float, end_ang: float):
    start_rad, end_rad = self.ang_to_rad(start_ang), self.ang_to_rad(end_ang)
    start_index, end_index = self.calc_range_index_by_rad(
        start_rad), self.calc_range_index_by_rad(end_rad)
    return self.scan.ranges[int(start_index):int(end_index)]

  def calc_range_index_by_rad(self, rad: float):
    if self.scan.angle_increment == 0:
      return 0
    # ang = angle_min + angle_increment * index => index = (ang - angle_min) / angle_increment
    return (rad - self.scan.angle_min) / self.scan.angle_increment

  def ang_to_rad(self, ang: float):
    return ang * PI / 180


def main(args=None):
  rclpy.init(args=args)

  minimal_publisher = Publisher()

  minimal_publisher.set_destiny(5., 5.)

  rclpy.spin(minimal_publisher)

  minimal_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
