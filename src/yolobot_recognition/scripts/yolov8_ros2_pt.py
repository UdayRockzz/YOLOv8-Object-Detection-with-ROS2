#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy #RCL Pi provides the canonical python API for interacting with ros2
from rclpy.node import Node
from sensor_msgs.msg import Image #importing image message type which we will use to publish data
from cv_bridge import CvBridge #import CV Bridge Library which enables us to do conversions between ros image messages and opencv images

#importing previously defined custom messages

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolov8n.pt') #Define YOLO model here and using the smallest model YOLO v8n

        self.yolov8_inference = Yolov8Inference() #Defining the YOLOV8 inference msg and The YOLO Inference API allows you to access the YOLOv8 object detection capabilities via a RESTful API.
        
        #creating a subscriber to camera topic
        self.subscription = self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1) #publisher to publish inference results as an array
        self.img_pub = self.create_publisher(Image, "/inference_result", 1) #publisher which will publish inference results as an image

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8") #ros messages are converted to CV2 image format
        results = self.model(img)

        #adding frame it and timestamp to a header of YOLO V8 inference message
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        #an array inference results for each object
        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")
        
        # extract an annotated image from result and convert it to a ros message
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        #the image and inference results are published
        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear() #clear an array each time a new topic is received

if __name__ == '__main__':
    rclpy.init(args=None) # ros communication is initialized
    camera_subscriber = Camera_subscriber() # creating an instance of a node
    rclpy.spin(camera_subscriber) #node will be executed until shutdown
    rclpy.shutdown()
