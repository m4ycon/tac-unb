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
    self.has_obstacle_foward = False
    self.angle_min = 0
    self.angle_max = 0
    self.angle_increment = 0

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
    self.angle_min, self.angle_max, self.angle_increment = scan.angle_min, scan.angle_max, scan.angle_increment

    regional_angle = 30
    dist_threshold = .5

    mid_index = self.calc_range_index_by_rad(-PI/2)
    front_region = scan.ranges[int(
        mid_index - regional_angle):int(mid_index + regional_angle)]

    closests = [x for x in front_region if x <= dist_threshold and x != 'inf']
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

  def calc_range_index_by_rad(self, rad: float):
    if self.angle_increment == 0:
      return 0
    # ang = angle_min + angle_increment * index
    # index = (ang - angle_min) / angle_increment
    return (rad - self.angle_min) / self.angle_increment


def main(args=None):
  rclpy.init(args=args)

  minimal_publisher = Publisher()

  rclpy.spin(minimal_publisher)

  minimal_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
