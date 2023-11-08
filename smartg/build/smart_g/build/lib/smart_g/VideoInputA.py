import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import sys
import tkinter as tk

class VideoInputA(Node):
    """
    Create an VideoInput class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("video_publisher")

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, "video_frames", 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()

        # Create a VideoCapture object
    def get_video_source(self):
        root = tk.Tk()
        root.title("Select Video Source")

        # Function to handle button click
        def handle_button_click(selection):
            if selection == "webcam":
                self.cap = cv2.VideoCapture(0)  # Webcam
            elif selection == "recorded video":
                video_path = input('Enter the path to the pre-recorded video file (e.g., "your_video.avi"): ')
                self.cap = cv2.VideoCapture(video_path)
            else:
                self.get_logger().error('Invalid input. Please write again.')
                sys.exit(0)  # You may want to handle this case accordingly

            if not self.cap.isOpened():
                self.get_logger().error("Failed to open video source.")
            else:
                root.destroy()

        label = tk.Label(root, text="Please select the video source for publishing:", font=("Arial", 18))
        label.pack(pady=10)  # Add padding on the top and bottom

        # Create a frame to hold the buttons
        button_frame = tk.Frame(root)
        button_frame.pack()

        button_webcam = tk.Button(button_frame, text="Webcam", command=lambda: handle_button_click("webcam"), width=20, height=2, font=("Arial", 18, "bold"))
        button_recorded_video = tk.Button(button_frame, text="Recorded Video", command=lambda: handle_button_click("recorded video"), width=20, height=2, font=("Arial", 18, "bold"))

        # Add color to buttons
        button_webcam.configure(bg='green')
        button_recorded_video.configure(bg='orange')

        button_webcam.grid(row=0, column=0, padx=10)  # Use grid for better layout control
        button_recorded_video.grid(row=0, column=1, padx=10)

        # Adjust the window size based on content
        root.update_idletasks()
        root.geometry('{}x{}'.format(button_frame.winfo_width() + 20, button_frame.winfo_height() + label.winfo_height() + 40))
        
        root.mainloop()
        # Used to convert between ROS and OpenCV images

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.

        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))

        # Display the message on the console
        self.get_logger().info("Publishing video frame")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    video_publisherA = VideoInputA()

    video_publisherA.get_video_source()

    # Spin the node so the callback function is called.
    rclpy.spin(video_publisherA)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_publisherA.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
