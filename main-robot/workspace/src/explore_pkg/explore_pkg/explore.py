import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
import time
import math
import tf_transformations as tf


PI = math.pi


class PID:
  def __init__(self, kp: float, ki: float, kd: float):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.prev_error = 0
    self.integral = 0
    self.prev_time = time.time()

  def update(self, error: float):
    current_time = time.time()
    dt = current_time - self.prev_time

    self.integral += error * dt
    derivative = (error - self.prev_error) / dt
    self.prev_error = error
    self.prev_time = current_time

    p = self.kp * error
    i = self.ki * self.integral
    d = self.kd * derivative

    return p + i + d


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
    self.should_adjust = False
    self.arrived = False

    self.angle_pid = PID(.5, 1e-5, 0)
    self.vel_pid = PID(.5, 1e-5, 0)
    self.ang = 0. # rad
    self.vel = 0.

    timer_period = .1
    self.timer = self.create_timer(timer_period, self.run)

  def run(self):
    if self.arrived:
      self.set_vel(0., 0., 0.)
      self.get_logger().info(
          f'Arrived ({self.actual_pos.x}, {self.actual_pos.y})')
      return
    elif self.has_obstacle_foward or self.should_adjust:
      self.stop_and_rotate()
    else:
      self.walk_foward()

    self.get_logger().info(
        f'Actual position: {self.actual_pos.x}, {self.actual_pos.y}, {self.actual_pos.z}')
    self.get_logger().info(
        f'Actual orientation: {self.actual_ori.x}, {self.actual_ori.y}, {self.actual_ori.z}, {self.actual_ori.w}')

    self.i += 1

  def scan_callback(self, scan: LaserScan):
    self.scan = scan

    regional_ang = 30
    mid_ang = -90
    start_rad = math.radians(mid_ang - regional_ang)
    end_rad = math.radians(mid_ang + regional_ang)
    front_region = self.get_range_interval(start_rad, end_rad)

    dist_threshold = .5
    closests = [x for x in front_region if x <= dist_threshold and x != 'inf']
    self.has_obstacle_foward = len(closests) > 0

  def odom_callback(self, odom: Odometry):
    self.actual_pos = odom.pose.pose.position
    self.actual_ori = odom.pose.pose.orientation

    dx, dy = self.destiny.x - self.actual_pos.x, self.destiny.y - self.actual_pos.y
    dist = math.sqrt(dx**2 + dy**2)
    ref_yaw = math.atan2(dy, dx)

    (roll, pitch, actual_yaw) = tf.euler_from_quaternion(
        [self.actual_ori.x, self.actual_ori.y, self.actual_ori.z, self.actual_ori.w])

    yaw_diff = ref_yaw - actual_yaw
    self.turn_left = yaw_diff > 0
    self.should_adjust = abs(yaw_diff) > math.radians(2)
    self.arrived = dist < .01

    self.ang = self.angle_pid.update(yaw_diff)
    self.vel = self.vel_pid.update(dist)

  def walk_to_destiny(self):
    self.set_vel(self.vel, 0., 0., self.ang)
    self.get_logger().info(f'Publishing: "walk_to_destiny", i: {self.i}')

  def walk_foward(self):
    self.set_vel(self.vel, 0., 0., self.ang)
    self.get_logger().info(f'Publishing: "walk_foward", i: {self.i}')

  def stop_and_rotate(self):
    self.set_vel(0., 0., 0., self.ang)
    self.get_logger().info(f'Publishing: "stop_and_rotate", i: {self.i}')

  def set_vel(self, x: float, y: float, z: float, angular_z: float = 0.):
    msg = Twist()
    msg.linear.x = float(x)
    msg.linear.y = float(y)
    msg.linear.z = float(z)
    msg.angular.z = float(angular_z)
    self.publisher_vel.publish(msg)

  def set_destiny(self, x: float, y: float, z: float = 0.):
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    self.destiny = point

  def get_range_interval(self, start_rad: float, end_rad: float):
    start_index, end_index = self.calc_range_index_by_rad(
        start_rad), self.calc_range_index_by_rad(end_rad)
    return self.scan.ranges[int(start_index):int(end_index)]

  def calc_range_index_by_rad(self, rad: float):
    if self.scan.angle_increment == 0:
      return 0
    # ang = angle_min + angle_increment * index => index = (ang - angle_min) / angle_increment
    return (rad - self.scan.angle_min) / self.scan.angle_increment


def main(args=None):
  rclpy.init(args=args)

  minimal_publisher = Publisher()

  minimal_publisher.set_destiny(-5., 3.)

  rclpy.spin(minimal_publisher)

  minimal_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
