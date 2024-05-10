import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi as PI
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry


class Publisher(Node):

  def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
    self.subscription_scan = self.create_subscription(
        LaserScan, '/scan', self.scan_callback, 10)
    self.subscription_odom = self.create_subscription(
        Odometry, '/odom', self.odom_callback, 10)

    timer_period = .5
    self.timer = self.create_timer(timer_period, self.run)

    self.i = 0
    self.has_obstacle_foward = False
    self.scan = LaserScan()
    self.angle_min = 0
    self.angle_max = 0
    self.angle_increment = 0
    self.actual_pos = Point()
    self.actual_ori = Quaternion()
    self.destiny = Point()

  def run(self):
    if self.has_obstacle_foward:
      self.stop_and_rotate()
    else:
      self.walk_foward(.2)

    self.i += 1

  def scan_callback(self, scan: LaserScan):
    self.scan = scan

    regional_angle = 30
    dist_threshold = .5

    mid_index = self.calc_range_index_by_rad(-PI/2)
    front_region = scan.ranges[int(
        mid_index - regional_angle):int(mid_index + regional_angle)]

    closests = [x for x in front_region if x <= dist_threshold and x != 'inf']
    self.has_obstacle_foward = len(closests) > 0

  def odom_callback(self, odom: Odometry):
    self.actual_pos = odom.pose.pose.position
    self.actual_ori = odom.pose.pose.orientation
    self.get_logger().info(
        f'Actual position: {self.actual_pos.x}, {self.actual_pos.y}')

  def walk_foward(self, x: float = 1.0):
    self.set_vel(x, 0., 0.)
    self.get_logger().info(f'Publishing: "walk_foward", i: {self.i}')

  def stop_and_rotate(self, angular: float = PI/4):
    self.set_ang(angular)
    self.get_logger().info(f'Publishing: "stop_and_rotate", i: {self.i}')

  def set_vel(self, x: float, y: float, z: float):
    msg = Twist()
    msg.linear.x = float(x)
    msg.linear.y = float(y)
    msg.linear.z = float(z)
    self.publisher_vel.publish(msg)

  def set_ang(self, angular: float):
    msg = Twist()
    msg.linear.x = .0
    msg.linear.y = .0
    msg.linear.z = .0
    msg.angular.z = float(angular)
    self.publisher_vel.publish(msg)

  def calc_range_index_by_rad(self, rad: float):
    if self.scan.angle_increment == 0:
      return 0
    # ang = angle_min + angle_increment * index
    # index = (ang - angle_min) / angle_increment
    return (rad - self.scan.angle_min) / self.scan.angle_increment

  def set_destiny(self, x: float, y: float):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0
    self.destiny = point


def main(args=None):
  rclpy.init(args=args)

  minimal_publisher = Publisher()

  rclpy.spin(minimal_publisher)

  minimal_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
