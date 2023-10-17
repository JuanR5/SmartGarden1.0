import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class MedianFiltering(Node):
    def __init__(self):
        super().__init__("image_medianf")
        self.subscription = self.create_subscription(
            Image, "video_frames", self.listener_callback, 10
        )
        self.subscription
        self.br = CvBridge()

        # Initialize a list to store previous frames
        self.frame_buffer = []

    def listener_callback(self, data):
        self.get_logger().info("Receiving video frame")
        try:
            current_frame = self.br.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

        # Append the current frame to the buffer
        self.frame_buffer.append(current_frame)

        # Limit the buffer size to a certain number of frames
        buffer_size = 10
        # Create kernel for morphological operation
        kernel = np.ones((20, 20), np.uint8)
        max_index = None
        if len(self.frame_buffer) > buffer_size:
            self.frame_buffer.pop(0)  # Remove the oldest frame

        # Calculate the median frame from the buffer
        if len(self.frame_buffer) > 0:
            median_frame = np.median(self.frame_buffer, axis=0).astype(np.uint8)

            # Calculate the absolute difference between the current frame and the median frame
            diff_frame = cv2.absdiff(current_frame, median_frame)

            # Threshold the difference frame to create a binary foreground mask
            _, fg_mask = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)
            # Close dark gaps in foreground object using closing
            fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
            fg_mask = cv2.cvtColor(fg_mask, cv2.COLOR_BGR2GRAY)
            # Find the index of the largest contour and draw bounding box
            fg_mask_bb = fg_mask
            cv2.imshow("Filtro binario", fg_mask)

            contours, hierarchy = cv2.findContours(
                fg_mask_bb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )[-2:]
            areas = [cv2.contourArea(c) for c in contours]
            # Log the length of areas for debugging
            self.get_logger().info("Length of areas: %d" % len(areas))
            
            # If there are no countours
            if len(areas) < 1:
                # Display the resulting frame
                # cv2.imshow('frame sin BoundingBox',current_frame)
                cv2.waitKey(1)

            else:
                # Find the largest moving object in the image
                max_index = np.argmax(areas)
                # Log the value of max_index for debugging
                self.get_logger().info("max_index: %d" % max_index)
            
            # Draw the bounding box
            if max_index is not None and max_index != 0:
                cnt = contours[max_index]
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

                # Draw circle in the center of the bounding box
                x2 = x + int(w / 2)
                y2 = y + int(h / 2)
                cv2.circle(current_frame, (x2, y2), 4, (0, 255, 0), -1)

                # Print the centroid coordinates on the image
                text = "x: " + str(x2) + ", y: " + str(y2)
                cv2.putText(
                    current_frame,
                    text,
                    (x2 - 10, y2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

            # Display the original frame and the foreground mask
            cv2.imshow("Bounding Box", current_frame)
            # cv2.imshow('Foreground Mask', fg_mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            pass


def main(args=None):
    rclpy.init(args=args)
    image_medianf = MedianFiltering()
    rclpy.spin(image_medianf)
    cv2.destroyAllWindows()
    image_medianf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()