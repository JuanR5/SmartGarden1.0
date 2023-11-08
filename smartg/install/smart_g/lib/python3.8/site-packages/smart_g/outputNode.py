import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from PIL import ImageTk
import tkinter as tk
from PIL import Image
from io import BytesIO

class OutputNode(Node):
    def __init__(self):
        super().__init__("outputNode")
        
        self.text_subscription = self.create_subscription(String, 'time_topic', self.text_callback, 10)

        self.br = CvBridge()
        self.encoding = 'rgb8'
        self.filtered_frame = None

        self.init_gui()
        self.start_video_subscription("tracked_obj")  # Start with "tracked_obj" topic

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS Video Viewer")

        self.video_frame = tk.Label(self.root)
        self.video_frame.pack()

        self.text_frame = tk.Label(self.root, text="Result Message Here")
        self.text_frame.pack()

    def update_video_frame(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(frame_rgb, (416, 416))  # Resize the frame if needed
        img = self.br.cv2_to_imgmsg(img, encoding=self.encoding)
        self.video_frame.img = img
        img_pil = Image.frombytes("RGB", (img.width, img.height), bytes(img.data), "raw", "RGB", 0, 1)
        img_tk = ImageTk.PhotoImage(img_pil)
        self.video_frame.config(image=img_tk)
        self.video_frame.img = img_tk
        self.root.update()

    def text_callback(self, msg):
        self.text_frame.config(text=msg.data)
        self.root.update()

    def start_video_subscription(self, topic):
        self.subscription = self.create_subscription(Image, topic, self.video_callback, 10)

    def video_callback(self, msg):
        
        img = self.br.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        self.update_video_frame(img)

def main(args=None):
    
    rclpy.init(args=args)
    outputNode = OutputNode()
    rclpy.spin(outputNode)
    outputNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
