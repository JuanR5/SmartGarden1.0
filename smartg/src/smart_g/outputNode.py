import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import random  # Use the random module to generate random colors
from random import randint  # Handles the creation of random integers

class OutputNode(Node):
   
    def __init__(self):
        
        super().__init__("outputNode")
        
        self.class_subscription = self.create_subscription(
            String, 'class_topic', self.class_callback, 10)

        self.subscription = self.create_subscription(
            Image, "detected_frames", self.listener_callbackB, 10)
        
        
        self.subscription

        self.publisher_ = self.create_publisher(Image, "tracked_obj", 10)
        self.time_publisher = self.create_publisher(String, 'time_topic', 10)
        self.time_msg = String()

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.class_name = None
        self.color = (0,0,0) 

        self.filtered_frame = np.empty(shape=(416, 416, 3), dtype=np.uint8)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
  
    def class_callback(self, msg):
        # Callback for class topic
        self.class_name = msg.data
            
    def process_frame(self, current_frame):
        
        optimal_harvest_time = harvest_times.get(self.class_name, "N/A")
        self.time_msg.data = f"Optimal Harvest Time: {optimal_harvest_time} weeks"
        org = (50, 50)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (0, 0, 255)  # Red color
        thickness = 2
        cv2.putText(current_frame, self.time_msg.data, org, font, fontScale, color, thickness)

    def listener_callbackB(self, data):
        
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        img_rts = self.br.imgmsg_to_cv2(data)
        current_frame = cv2.cvtColor(img_rts, cv2.COLOR_BGR2RGB)
        
        self.process_frame(current_frame)
        # Publish the processed frame
                    
        self.filtered_frame = current_frame
        # Store the filtered frame

        cv2.waitKey(1)
        
    def timer_callback(self):
              
        self.time_publisher.publish(self.time_msg)
        
        # Check if a frame has been received from the subscriber
        if self.filtered_frame is not None:
            # Publish the filtered frame.
            # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(self.filtered_frame, encoding="bgr8"))

        # Display the message on the console
        self.get_logger().info("Publishing tracked obj")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    outputNode = OutputNode()

    # Spin the node so the callback function is called.
    rclpy.spin(outputNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    outputNode.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()




#####################################################################################

import tkinter as tk
import cv2

class VideoApp(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Video Selection")

        # Create video frames
        self.video_frame1 = tk.Frame(self)
        self.video_frame1.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.video_frame2 = tk.Frame(self)
        self.video_frame2.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Create video labels
        self.video_label1 = tk.Label(self.video_frame1, width=640, height=480)
        self.video_label1.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.video_label2 = tk.Label(self.video_frame2, width=640, height=480)
        self.video_label2.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Create video capture objects
        self.cap1 = cv2.VideoCapture(0)
        self.cap2 = cv2.VideoCapture(1)

        # Create switch button
        self.switch_button = tk.Button(self, text="Switch Video", command=self.switch_video)
        self.switch_button.pack(side=tk.BOTTOM)

        # Create text message label
        self.text_label = tk.Label(self, text="Output Message", font=("Arial", 12))
        self.text_label.pack(side=tk.BOTTOM)

        # Start video streaming
        self.show_video1()

    def show_video1(self):
        while True:
            ret, frame = self.cap1.read()
            if ret:
                cv2.imshow("Video Frame 1", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.cap1.release()
        cv2.destroyAllWindows()

    def show_video2(self):
        while True:
            ret, frame = self.cap2.read()
            if ret:
                cv2.imshow("Video Frame 2", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.cap2.release()
        cv2.destroyAllWindows()

    def switch_video(self):
        if self.video_frame1.winfo_ismapped():
            self.video_frame1.pack_forget()
            self.video_frame2.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
            self.show_video2()
            self.switch_button.configure(text="Switch to Video 1")
        else:
            self.video_frame2.pack_forget()
            self.video_frame1.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
            self.show_video1()
            self.switch_button.configure(text="Switch to Video 2")

if __name__ == "__main__":
    app = VideoApp()
    app.mainloop()
