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

        self.encoding = 'rgb8'  # Initialize encoding to 'rgb8'
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
        frame = self.br.imgmsg_to_cv2(data)

        # Load the pre-trained model
        leaf_disease_model = load_model('leaf-model/Leaf Deases(96,88).h5')

        # Define class labels
        label_names = ['Apple scab', 'Apple Black rot', 'Apple Cedar apple rust', 'Apple healthy', 'Cherry Powdery mildew',
                    'Cherry healthy', 'Corn Cercospora leaf spot Gray leaf spot', 'Corn Common rust', 'Corn Northern Leaf Blight',
                    'Corn healthy', 'Grape Black rot', 'Grape Esca', 'Grape Leaf blight', 'Grape healthy', 'Peach Bacterial spot',
                    'Peach healthy', 'Pepper bell Bacterial spot', 'Pepper bell healthy', 'Potato Early blight', 'Potato Late blight',
                    'Potato healthy', 'Strawberry Leaf scorch', 'Strawberry healthy', 'Tomato Bacterial spot', 'Tomato Early blight',
                    'Tomato Late blight', 'Tomato Leaf Mold', 'Tomato Septoria leaf spot', 'Tomato Spider mites', 'Tomato Target Spot',
                    'Tomato Yellow Leaf Curl Virus', 'Tomato mosaic virus', 'Tomato healthy']

        # Set the window size (you can change these values to make the window larger)
        #window_width = 500
        #window_height = 500

        # Create the window
        #cv2.namedWindow('Leaf Disease Classification', cv2.WINDOW_NORMAL)
        #cv2.resizeWindow('Leaf Disease Classification', window_width, window_height)

        # Set the font properties for the text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4  # Adjust the font size (you can change this value)
        font_color = (0, 0, 0)  # Green color
        font_thickness = 1 # Adjust the font thickness (you can change this value)
        # Preprocess the frame (resize to 150x150 and convert to array)
        frame = cv2.resize(frame, (150, 150))
        frame = img_to_array(frame)
        frame = preprocess_input(frame)

        # Make a prediction using the model
        prediction = leaf_disease_model.predict(np.expand_dims(frame, axis=0))

        # Get the predicted class label and confidence
        predicted_class = label_names[np.argmax(prediction)]
        confidence = prediction[0][np.argmax(prediction)] * 100

        # Display the predicted label and confidence on the frame
        label_text = f"{predicted_class} ({confidence:.2f}%)"
        cv2.putText(frame, label_text, (10, 30), font, font_scale, font_color, font_thickness)

        # Display the frame with the label
        self.filtered_frame = frame
        
        #cv2.imshow('Leaf Disease Classification', frame)
        cv2.waitKey(1)

        # Exit the loop if the 'q' key is pressed
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #cv2.destroyAllWindows()     

    def timer_callback(self):
            """
            Callback function.
            This function gets called every 0.1 seconds.
            """
            # Check if a frame has been received from the subscriber
            if self.filtered_frame is not None:
            # Publish the filtered frame.
            # Convert to 8-bit RGB (rgb8) format
                filtered_frame_rgb8 = (self.filtered_frame * 255).astype(np.uint8)
            # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message
                #self.publisher_.publish(self.br.cv2_to_imgmsg(self.filtered_frame, encoding='rgb8')) ## if gray scale encoding = mono8
                self.publisher_.publish(self.br.cv2_to_imgmsg(filtered_frame_rgb8, encoding='rgb8'))
            # Display the message on the console
            self.get_logger().info("Publishing leaf frame")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    leaf_define = Leaf_define()

    # Spin the node so the callback function is called.
    rclpy.spin(leaf_define)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leaf_define.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()