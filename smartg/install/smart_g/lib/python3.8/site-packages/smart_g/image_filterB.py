import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library


class ImageFilterB(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_filterB")

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, "video_frames", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

###        # Create the publisher. This publisher will publish an Image
        # to the filterA_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, "filteredB_frames", 10)

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

        #### INVERTED FILTER
        def invert(img):
            inv = cv2.bitwise_not(img)
            return inv

        # making the invert img
        img_rst = invert(current_frame)

        # Store the filtered frame
        self.filtered_frame = img_rst    
        ##########
        """

        # Convert the current frame to grayscale
        img_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # Store the grayscale frame
        self.filtered_frame = img_gray
        """

        # Display image
        #cv2.imshow("Inverted", img_rst)

        cv2.waitKey(1)

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
