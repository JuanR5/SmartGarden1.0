import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
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
        ###
        # Create the publisher. This publisher will publish an Image
        # to the filterA_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, "detected_frames", 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the bridge object  
        self.filtered_frame = None

        self.encoding = 'bgr8'  # Initialize encoding to 'rgb8'
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def get_video_source(self):
        root = tk.Tk()
        root.geometry("416x416")
        root.title("Select Video Source")

        def handle_button_click(selection):
            self.start_video_subscription(selection)

            # Set encoding based on user selection
            if selection in ["filteredSubs_frames"]:
                self.encoding = 'mono8'
            else:
                self.encoding = 'bgr8'

            root.destroy()

        label = tk.Label(root, text="Please select the video source for detection:")
        label.pack()

        button_normal = tk.Button(root, text="Normal", command=lambda: handle_button_click("video_frames"))
        button_filterA = tk.Button(root, text="Filter A", command=lambda: handle_button_click("filteredA_frames"))
        button_filterB = tk.Button(root, text="Filter B", command=lambda: handle_button_click("filteredB_frames"))
        button_filterSubs = tk.Button(root, text="Filter BkgSubsT", command=lambda: handle_button_click("filteredSubs_frames"))

        button_normal.pack()
        button_filterA.pack()
        button_filterB.pack()
        button_filterSubs.pack()

        root.mainloop()

    def start_video_subscription(self, topic):
        self.subscription = self.create_subscription(
            Image, topic, self.listener_callbackB, 10
        )
        self.subscription


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
        model = YOLO("yolo-Weights/bestweights.pt")

        # object classes
        classNames = ["freshripe", "freshunripe", "overripe", "ripe", "rotten", "unripe"]

        # Initialize an empty list to store object information
        detected_objects = []      
        results = model(img_bgr8, stream=True)
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
                cv2.rectangle(img_bgr8, (x1, y1), (x2, y2), (255, 0, 255), 3)
                # print coordinates
                
                
                # confidence
                confidence = math.ceil((box.conf[0]*100))/100
                #print("Confidence --->",confidence)

                # class name
                cls = int(box.cls[0])
                class_name=classNames[cls]
                #print("Class name -->", classNames[cls])

                # Append the object information to the list
                detected_objects.append({
                    'coordinates': (x1, y1, x2, y2),
                    'class_name': class_name,
                    'confidence': confidence
                })
                # object details
                org = (x1, y1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(img_bgr8, classNames[cls], org, font, fontScale, color, thickness)
        
            

        self.filtered_frame = img_bgr8
        #cv2.imshow('Processed Frame', img_rts)
        cv2.waitKey(1)
        #self.publisher_.publish(self.br.cv2_to_imgmsg(img_rts))
        #self.get_logger().info("Publishing video frame")
    def timer_callback(self):
        
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
