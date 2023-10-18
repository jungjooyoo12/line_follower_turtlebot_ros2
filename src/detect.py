#!/usr/bin/env python3

from functools import partial
import rclpy
from rclpy.node import Node
import std_msgs.msg 
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2 as cv2
import linedetect

def detect_line_and_publish(pub, cvb, msg):
    img = cvb.imgmsg_to_cv2(msg, 'bgr8')

    res, frame, hsv, mask = linedetect.colorthresh(img)
    
    #cv.imshow("img", img)
    #cv.imwrite("img.png", img)
    #cv.imshow("hsv", hsv)
    #cv.imshow("mask", mask)
    #cv.waitKey(0)

    #print(res)

    msg = std_msgs.msg.String()
    msg.data = res

    pub.publish(msg)


image_msg = None
def image_received(msg):
    global image_msg
    image_msg = msg

def main(args=None):
    rclpy.init(args=args)

    node = Node('detect')

    direction_pub = node.create_publisher(
            std_msgs.msg.String,
            '/direction',
            10)
    
    image_sub = node.create_subscription(
            sensor_msgs.msg.Image,
            '/camera/image_raw',
            image_received,
            10)
    
    cvb = CvBridge()

    while rclpy.ok():
        if image_msg is not None:
            detect_line_and_publish(direction_pub, cvb, image_msg)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
