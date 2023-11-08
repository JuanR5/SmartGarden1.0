import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrackedObject:
    def __init__(self, tracker, bbox, color, class_name):
        self.tracker = tracker
        self.bbox = bbox
        self.color = color
        self.class_name = class_name
        self.tracking_id = None

class Tracker(Node):
    def __init__(self):
        super().__init__("image_filterSubs")
        self.class_subscription = self.create_subscription(String, 'class_topic', self.class_callback, 10)
        self.bbox_subscription = self.create_subscription(String, 'bbox_topic', self.bbox_callback, 10)
        self.publisher_ = self.create_publisher(Image, "tracked_obj", 10)
        self.time_publisher = self.create_publisher(String, 'time_topic', 10)
        self.time_msg = String()
        self.tracked_objects = []
        self.br = CvBridge()

    def generate_tracker(self, bbox):
        tracker = cv2.TrackerMOSSE_create()
        tracker.init(self.current_frame, bbox)
        return tracker

    def class_callback(self, msg):
        self.class_name = msg.data

    def bbox_callback(self, msg):
        bbox_data = msg.data.split()
        if len(bbox_data) == 4:
            bbox = [int(value) for value in bbox_data]
            self.update_or_create_object(bbox)

    def update_or_create_object(self, bbox):
        if not self.tracked_objects:
            self.create_object(bbox)
        else:
            found = False
            for obj in self.tracked_objects:
                overlap = self.calculate_overlap(obj.bbox, bbox)
                if overlap > 0.5:  # Adjust the overlap threshold as needed
                    self.update_object(obj, bbox)
                    found = True
                    break
            if not found:
                self.create_object(bbox)

    def calculate_overlap(self, bbox1, bbox2):
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        endx1, endy1 = x1 + w1, y1 + h1
        endx2, endy2 = x2 + w2, y2 + h2
        x, y = max(x1, x2), max(y1, y2)
        endx, endy = min(endx1, endx2), min(endy1, endy2)
        if endx < x or endy < y:
            return 0.0
        area1 = (endx1 - x1) * (endy1 - y1)
        area2 = (endx2 - x2) * (endy2 - y2)
        intersection = (endx - x) * (endy - y)
        return intersection / (area1 + area2 - intersection)

    def create_object(self, bbox):
        color = (np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255))
        tracker = self.generate_tracker(bbox)
        obj = TrackedObject(tracker, bbox, color, self.class_name)
        self.tracked_objects.append(obj)

    def update_object(self, obj, bbox):
        obj.tracker.update(self.current_frame, bbox)
        obj.bbox = bbox

    def process_and_publish_frame(self):
        for obj in self.tracked_objects:
            success, bbox = obj.tracker.update(self.current_frame)
            if success:
                obj.bbox = bbox
                self.draw_bbox(obj)
        self.publish_frame()

    def draw_bbox(self, obj):
        x, y, w, h = [int(coord) for coord in obj.bbox]
        cv2.rectangle(self.current_frame, (x, y), (x + w, y + h), obj.color, 2)
        text = f"{obj.class_name} ID: {obj.tracking_id}"
        cv2.putText(self.current_frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 2)
        self.increment_tracking_id(obj)

    def increment_tracking_id(self, obj):
        if obj.tracking_id is None:
            obj.tracking_id = len(self.tracked_objects)

    def publish_frame(self):
        self.publisher_.publish(self.br.cv2_to_imgmsg(self.current_frame, encoding="bgr8"))

    def listener_callbackB(self, data):
        self.get_logger().info("Receiving video frame")
        img_rts = self.br.imgmsg_to_cv2(data)
        self.current_frame = img_rts.copy()
        self.process_and_publish_frame()

def main(args=None):
    rclpy.init(args=args)
    obj_tracker = Tracker()
    rclpy.spin(obj_tracker)
    obj_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


########
"""                 Opciones para seguir intentando el tracker                  """
#######


""" Opcion 1A
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import random  # Use the random module to generate random colors
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
   
    def __init__(self):
        
        super().__init__("obj_tracker")
        self.class_subscription = self.create_subscription(
            String, 'class_topic', self.class_callback, 10)
        self.bbox_subscription = self.create_subscription(
            String, 'bbox_topic', self.bbox_callback, 10)
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

        self.multi_tracker = cv2.MultiTracker_create()
        self.bounding_box_list = []
        self.class_name = None
        self.color_list = [] 

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
            
    def process_frame(self, current_frame):
        for bbox in self.bounding_box_list:
            self.multi_tracker.add(self.generate_tracker(desired_tracker), current_frame, bbox)

            blue = random.randint(127, 255)
            green = random.randint(127, 255)
            red = random.randint(127, 255)
            self.color_list.append((blue, green, red))

        success, bboxes = self.multi_tracker.update(current_frame)

        for i, bbox in enumerate(bboxes):
            point1 = (int(bbox[0]), int(bbox[1]))
            point2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(current_frame, point1, point2, self.color_list[i], 5)

            # Retrieve optimal harvest time in weeks based on the class
            optimal_harvest_time = harvest_times.get(self.class_name, "N/A")

            # Display optimal harvest time on the image
            self.time_msg.data = f"Optimal Harvest Time: {optimal_harvest_time} weeks"
            org = (point1[0], point1[1] - 10)
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
            self.publisher_.publish(self.br.cv2_to_imgmsg(self.filtered_frame, encoding="rgb8"))

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


"""