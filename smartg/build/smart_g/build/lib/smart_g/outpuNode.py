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

class DetectorA(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_detector")

        # Create the subscriber. This subscriber will receive an Image
        # from the topic selected. The queue size is 10 messages.
        
        self.subscription = self.create_subscription(
            Image, "tracked_obj", self.listener_callbackB, 10)
        self.subscription = self.create_subscription(
            Image, "leaf_frames", self.listener_callbackB, 10)
        self.text_subscription = self.create_subscription(
            String, 'time_topic', self.text_callback, 10)
        #self.text_subscription = self.create_subscription(
         #   String, 'bbox_topic', self.text_callback, 10)
        
        self.subscription 
        
        # Create the publisher. This publisher will publish an Image
        # to the filterA_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, "final_frames", 10)
        self.text1_publisher = self.create_publisher(String, 'text1_topic', 10)
        self.text1_msg = String()
        self.text2_publisher = self.create_publisher(String, 'text2_topic', 10)
        self.text2_msg = String()
        
        
        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the bridge object  
      

        self.encoding = 'rgb8'  # Initialize encoding to 'rgb8'
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def video_output(self):
        root = tk.Tk()
        root.geometry("416x416")
        root.title("Select Video Output to show")

        def handle_button_click(selection):
            self.start_video_subscription(selection)

        label = tk.Label(root, text="Please select the video source:")
        label.pack()

        button_leaf = tk.Button(root, text="Hojas", command=lambda: handle_button_click("leaf_frames"))
        button_banana = tk.Button(root, text="Bananos", command=lambda: handle_button_click("tracked_obj"))

        button_leaf.pack()
        button_banana.pack()

        root.mainloop()

    def start_video_output(self, topic):
        self.publisher_.publish(topic)


    def listener_callbackB(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        img_rts = self.br.imgmsg_to_cv2(data)
        # Convert the image format to 'bgr8'
        img_bgr8 = cv2.cvtColor(img_rts, cv2.COLOR_RGB2BGR)
        img_rgb = cv2.cvtColor(img_bgr8, cv2.COLOR_BGR2RGB)

        # model
      
            

        self.filtered_frame = img_bgr8
        #cv2.imshow('Processed Frame', img_rts)
        cv2.waitKey(1)
        #self.publisher_.publish(self.br.cv2_to_imgmsg(img_rts))
        #self.get_logger().info("Publishing video frame")
        
    def timer_callback(self):
        
        self.text_msg.data = self.class_name
        self.text_publisher.publish(self.text_msg)
        
        self.bbox_msg.data = self.class_name
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
    image_detectorA= DetectorA()

    image_detectorA.get_video_source()

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
