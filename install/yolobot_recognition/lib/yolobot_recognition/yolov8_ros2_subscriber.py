#!/usr/bin/env python3

import cv2
import threading #threading allows us to have several nodes run concurrently
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):
    #subscribing to the camera topic
    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_callback,
            10)
        self.subscription 

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, "bgr8") #converting it to the opencv format

class Yolo_subscriber(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)
        self.subscription 

        # a publisher which will publish images with boundary boxes drawn using opencv
        self.cnt = 0

        self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)

    def yolo_callback(self, data):
        #drawing a boundary box around each detected object using rectangle function 
        global img
        for r in data.yolov8_inference:
        
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            yolo_subscriber.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")
            cv2.rectangle(img, (top, left), (bottom, right), (255, 255, 0))
            self.cnt += 1

        self.cnt = 0
        img_msg = bridge.cv2_to_imgmsg(img)  
        self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_subscriber = Yolo_subscriber() #subscribe to a YOLO inference results topic
    camera_subscriber = Camera_subscriber() # subscribe to a camera image topic

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_subscriber)
    executor.add_node(camera_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True) # thread pool maintains multiple threads waiting for tasks to be allocated for concurrent execution by the supervising program then threads are created and started for multi-threaded executor.

    executor_thread.start()
    
    rate = yolo_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
