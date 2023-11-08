import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
from random import randint  # Handles the creation of random integers


# Define your list of trackers
type_of_trackers = [
    "BOOSTING",
    "MIL",
    "KCF",
    "TLD",
    "MEDIANFLOW",
    "GOTURN",
    "MOSSE",
    "CSRT",
]
desired_tracker = "MOSSE"

harvest_times = {
    "freshripe": 2,
    "freshunripe": 3,
    "overripe": 1,
    "ripe": 2,
    "rotten": 0,
    "unripe": 4,
}


class Tracker(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        
        super().__init__("obj_tracker")

        self.subscription = self.create_subscription(
            Image, "detected_frames", self.listener_callbackB, 10
        )
        
        self.class_subscription = self.create_subscription(
            String, 'class_topic', self.class_callback, 10
        )
        
        self.bbox_subscription = self.create_subscription(
            String, 'bbox_topic', self.bbox_callback, 10
        )
        
        self.subscription

        self.publisher_ = self.create_publisher(Image, "tracked_obj", 10)
        self.text_publisher = self.create_publisher(String, 'time_topic', 10)
        self.text_msg = String()

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.multi_tracker = cv2.MultiTracker_create()
        self.bounding_box_list = None
        self.class_name = None

        self.filtered_frame = np.empty(shape=(416, 416, 3), dtype=np.uint8)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def generate_tracker(self, type_of_tracker):
        if type_of_tracker == type_of_trackers[0]:
            tracker = cv2.TrackerBoosting_create()
        elif type_of_tracker == type_of_trackers[1]:
            tracker = cv2.TrackerMIL_create()
        elif type_of_tracker == type_of_trackers[2]:
            tracker = cv2.TrackerKCF_create()
        elif type_of_tracker == type_of_trackers[3]:
            tracker = cv2.TrackerTLD_create()
        elif type_of_tracker == type_of_trackers[4]:
            tracker = cv2.TrackerMedianFlow_create()
        elif type_of_tracker == type_of_trackers[5]:
            tracker = cv2.TrackerGOTURN_create()
        elif type_of_tracker == type_of_trackers[6]:
            tracker = cv2.TrackerMOSSE_create()
        elif type_of_tracker == type_of_trackers[7]:
            tracker = cv2.TrackerCSRT_create()
        else:
            tracker = None
            print("The name of the tracker is incorrect")
            print("Here are the possible trackers:")
            for track_type in type_of_trackers:
                print(track_type)
        return tracker

    def listener_callbackB(self, data):
        
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        img_rts = self.br.imgmsg_to_cv2(data)
        current_frame = cv2.cvtColor(img_rts, cv2.COLOR_BGR2RGB)
            
        for bbox in self.bounding_box_list:
            self.multi_tracker.add(
                self.generate_tracker(desired_tracker), current_frame, bbox
            )

        success, bboxes = self.multi_tracker.update(current_frame)

        for i, bbox in enumerate(bboxes):
            point1 = (int(bbox[0]), int(bbox[1]))
            point2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(current_frame, point1, point2, self.color_list[i], 5)

            # Retrieve optimal harvest time in weeks based on the class
            optimal_harvest_time = harvest_times.get(self.class_name, "N/A")

            # Display optimal harvest time on the image
            self.text_publisher = f"Optimal Harvest Time: {optimal_harvest_time} weeks"
            org = (point1[0], point1[1] - 10)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0, 0, 255)  # Red color
            thickness = 2

            cv2.putText(current_frame, self.text_publisher, org, font, fontScale, color, thickness)

        self.filtered_frame = self.br.cv2_to_imgmsg(current_frame, encoding="rgb8")
        # Store the filtered frame

        cv2.waitKey(1)
        
    
    def class_callback(self, msg):
        # Callback for class topic
        self.class_name = msg.data

    def bbox_callback(self, msg):
        # Callback for bbox topic
        # Parse the string back to a list
        bbox_data = msg.data.split()
        if len(bbox_data) == 4:
            bbox = [int(value) for value in bbox_data]
            self.bounding_box_list.append(tuple(bbox))
    

    def timer_callback(self):
              
        # Check if a frame has been received from the subscriber
        if self.filtered_frame is not None:
            # Publish the filtered frame.
            # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message
            self.publisher_.publish(self.filtered_frame)

        # Display the message on the console
        self.get_logger().info("Publishing tracked obj")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    obj_tracker = Tracker()

    # Spin the node so the callback function is called.
    rclpy.spin(obj_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obj_tracker.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
