import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
import os

class Leaf_define(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("leaf_define")

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, "video_frames", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

###        # Create the publisher. This publisher will publish an Image
        # to the filterA_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, "leaf_frames", 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the bridge object  
        self.filtered_frame = None  
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Load the pre-trained model
        leaf_disease_model = load_model('/home/sofia/Proyecto_final/modeloClasificacion/leaf-diseases-detect/leaf-diseases-detect-main/Leaf Deases(96,88).h5')

        # Define class labels
        label_names = ['Apple scab', 'Apple Black rot', 'Apple Cedar apple rust', 'Apple healthy', 'Cherry Powdery mildew',
                    'Cherry healthy', 'Corn Cercospora leaf spot Gray leaf spot', 'Corn Common rust', 'Corn Northern Leaf Blight',
                    'Corn healthy', 'Grape Black rot', 'Grape Esca', 'Grape Leaf blight', 'Grape healthy', 'Peach Bacterial spot',
                    'Peach healthy', 'Pepper bell Bacterial spot', 'Pepper bell healthy', 'Potato Early blight', 'Potato Late blight',
                    'Potato healthy', 'Strawberry Leaf scorch', 'Strawberry healthy', 'Tomato Bacterial spot', 'Tomato Early blight',
                    'Tomato Late blight', 'Tomato Leaf Mold', 'Tomato Septoria leaf spot', 'Tomato Spider mites', 'Tomato Target Spot',
                    'Tomato Yellow Leaf Curl Virus', 'Tomato mosaic virus', 'Tomato healthy']

        # Open a connection to the webcam (0 represents the default camera, you can change it if needed)
        cap = cv2.VideoCapture(0)

        # Set the window size (you can change these values to make the window larger)
        window_width = 500
        window_height = 500

        # Create the window
        cv2.namedWindow('Leaf Disease Classification', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Leaf Disease Classification', window_width, window_height)

        # Set the font properties for the text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.3  # Adjust the font size (you can change this value)
        font_color = (0, 0, 0)  # Green color
        font_thickness = 1  # Adjust the font thickness (you can change this value)

    def timer_callback(self):
            """
            Callback function.
            This function gets called every 0.1 seconds.
            """
            # Check if a frame has been received from the subscriber
            if self.filtered_frame is not None:
            # Publish the filtered frame.
            # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message
                self.publisher_.publish(self.br.cv2_to_imgmsg(self.filtered_frame, encoding='bgr8')) ## if gray scale encoding = mono8

            # Display the message on the console
            self.get_logger().info("Publishing filterB frame")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_filterB = ImageFilterB()

    # Spin the node so the callback function is called.
    rclpy.spin(image_filterB)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_filterB.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()