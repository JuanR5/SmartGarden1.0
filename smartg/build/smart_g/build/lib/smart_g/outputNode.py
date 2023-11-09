import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import tkinter as tk
from PIL import Image as PILImage
from PIL import ImageTk
import numpy as np
import time

class OutputNode(Node):

    def __init__(self):

        super().__init__("outputNode")
        
        self.publisher_ = self.create_publisher(Image, "final_frames", 10)

        self.br = CvBridge()
        self.encoding = 'rgb8'
        self.filtered_frame = None
        self.selected_topic = "tracked_obj"  # Default topic

        self.subscription_dict = {}
        self.switching = False  # Flag to prevent rapid switching
        self.switch_delay = 2  # Delay in seconds

        self.init_gui()
        self.start_video_subscription("tracked_obj")
        self.start_video_subscription("leaf_frames")
        self.video_subscription = self.subscription_dict[self.selected_topic]
        self.text_subscription = self.create_subscription(String, 'time_topic', self.text_callback, 10)
        #self.start_label_subscription() #Label subscription 

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS Video Viewer")

        self.video_frame = tk.Label(self.root)
        self.video_frame.pack()

        self.text_frame = tk.Label(self.root, text="Result Message Here")
        self.text_frame.pack()
        self.text_frame.config(font=("Arial", 15))
        
        self.label2 = tk.Label(self.root, text="")
        self.label2.pack()
        self.label2.config(font=("Arial", 15))

        self.switch_button = tk.Button(self.root, text="Switch Video", command=self.switch_video)
        self.switch_button.pack()
        # Set the size and color of the button
        self.switch_button.config(height=10, width=10, background="#ff8c00", foreground="black")
        # Set the font of the button text
        self.switch_button.config(font=("Arial", 10, "bold"))
    
    def start_label_subscription(self):
        if "label_topic" not in self.subscription_dict:
            subscription = self.create_subscription(String, "label_topic", self.label_callback, 10)
            self.subscription_dict["label_topic"] = subscription
            
    def update_video_frame(self, frame):
        img_tk = self.cv2_to_image_tk(frame)
        self.video_frame.config(image=img_tk)
        self.video_frame.img = img_tk
        self.root.update()

    def text_callback(self, msg):
        self.text_frame.config(text=msg.data)
        self.root.update()  
        
    def label_callback(self, msg):
        if self.selected_topic == "leaf_frames":
            self.label2.config(text=msg.data)
            self.root.update()

    def start_video_subscription(self, topic):
        subscription = self.create_subscription(Image, topic, self.video_callback, 10)
        self.subscription_dict[topic] = subscription

    def video_callback(self, msg):
        img = self.br.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        self.update_video_frame(img)
        self.publisher_.publish(msg)

    def switch_video(self):
        if not self.switching:
            self.switching = True
            new_topic = "leaf_frames" if self.selected_topic == "tracked_obj" else "tracked_obj"

            # First subscribe to the new topic
            self.start_label_subscription()
            # Add a delay to prevent rapid switching
            time.sleep(self.switch_delay)          
            self.start_video_subscription(new_topic)

            # Then unsubscribe from the current topic
            self.destroy_subscription(self.video_subscription)
            # Finally, update the selected topic variable
            self.selected_topic = new_topic
            self.video_subscription = self.subscription_dict[self.selected_topic]
            self.switching = False
            
    def cv2_to_image_tk(self, cv_image):
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # Convert the ROS 2 Image message to a format suitable for Tkinter
        image_data = np.frombuffer(image_msg.data, dtype=np.uint8)
        image = np.reshape(image_data, (image_msg.height, image_msg.width, 3))

        # Convert the NumPy image to a PIL Image
        pil_image = PILImage.fromarray(image)

        # Convert the PIL Image to a Tkinter-compatible format
        image_tk = ImageTk.PhotoImage(pil_image)

        return image_tk
    
def main(args=None):
    rclpy.init(args=args)
    outputNode = OutputNode()

    try:
        rclpy.spin(outputNode)
    except KeyboardInterrupt:
        pass
    finally:
        outputNode.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
