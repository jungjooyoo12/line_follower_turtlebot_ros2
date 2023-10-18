from functools import partial

import rclpy
from rclpy.node import Node

import std_msgs.msg
import sensor_msgs.msg 
import geometry_msgs.msg

d = None
def publish_twist(pub):
    if d is None:
        return

    twist = geometry_msgs.msg.Twist()

    print(d)

    if d == 'turn left':
        twist.linear.x = 0.1
        twist.angular.z = 0.15
    elif d == 'turn right':
        twist.linear.x = 0.1
        twist.angular.z = -0.15
    elif d == 'go straight':
        twist.linear.x = 0.15
        twist.angular.z = 0.0
    elif d == 'search':
        twist.linear.x = 0.0
        twist.angular.z = 0.25
    else:
        print("bad direction {}".format(d), file=sys.stderr)

    pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    node = Node('motion_node')

    def on_direction_received(msg):
        global d
        d = msg.data

    vel_pub = node.create_publisher(
            geometry_msgs.msg.Twist,
            '/cmd_vel',
            10)

    image_sub = node.create_subscription(
            std_msgs.msg.String,
            '/direction',
            on_direction_received,
            10)

    timer = node.create_timer(0.1, partial(publish_twist, vel_pub))
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
