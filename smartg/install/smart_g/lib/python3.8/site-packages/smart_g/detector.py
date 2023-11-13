import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String 
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
from ultralytics import YOLO
import math 
import sys
import tkinter as tk

class Detector(Node):

    def __init__(self):
        
    
        super().__init__("image_detector")

        self.publisher_ = self.create_publisher(Image, "detected_frames", 10)
        self.text_publisher = self.create_publisher(String, 'class_topic', 10)
        self.text_msg = String()
        self.bbox_publisher = self.create_publisher(String, 'bbox_topic', 10)
        self.bbox_msg = String()
        
        self.subscription = self.create_subscription(
            Image, "video_frames", self.listener_callbackB, 10)
        
        
        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.filtered_frame = None
        self.class_name = ""
        self.bbox_name = []

        self.encoding = 'rgb8'
        
        self.br = CvBridge()



    def listener_callbackB(self, data):
        
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        img_rts = self.br.imgmsg_to_cv2(data)
        img_rgb = cv2.cvtColor(img_rts, cv2.COLOR_BGR2RGB)

        # model
        model = YOLO("yolo-Weights/bestweights.pt")

        # object classes
        classNames = ["freshripe", "freshunripe", "overripe", "ripe", "rotten", "unripe"]

        # Initialize an empty list to store object information
        detected_objects = []      
        results = model(img_rgb, stream=True)
        # Clear the previous frame's detected objects
        detected_objects = []
        
        # coordinates
        for r in results:
            boxes = r.boxes

            for box in boxes:
                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                # put box in cam and print
                cv2.rectangle(img_rgb, (x1, y1), (x2, y2), (255, 0, 255), 3)
                
                self.bbox_name = [x1, y1, x2, y2]
                
                # confidence
                confidence = math.ceil((box.conf[0]*100))/100

                # class name
                cls = int(box.cls[0])
                self.class_name=classNames[cls]

                # Append the object information to the list
                detected_objects.append({
                    'coordinates': (x1, y1, x2, y2),
                    'class_name': self.class_name,
                    'confidence': confidence})
                
                # object details
                org = (x1, y1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(img_rgb, classNames[cls], org, font, fontScale, color, thickness)

        self.filtered_frame = img_rgb
        
        cv2.waitKey(1)
        
    def timer_callback(self):
        
        self.text_msg.data = self.class_name
        self.text_publisher.publish(self.text_msg)
        
        ############
        bbox_str = ' '.join(map(str, self.bbox_name))
        self.bbox_msg.data = bbox_str ###########
        self.bbox_publisher.publish(self.bbox_msg)
        
        # Check if a frame has been received from the subscriber
        if self.filtered_frame is not None:
        # Publish the filtered frame.
        # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(self.filtered_frame, encoding=self.encoding))

        # Display the message on the console
        self.get_logger().info("Publishing video frame")

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_detectorA= Detector()

    # Spin the node so the callback function is called.
    rclpy.spin(image_detectorA)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_detectorA.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
