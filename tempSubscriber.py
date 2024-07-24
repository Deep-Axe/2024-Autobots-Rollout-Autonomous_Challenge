import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

QOS_PROFILE_DEFAULT = 10

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage

import os
import csv

def append_values_to_csv(row):
    with open(os.path.join(os.path.dirname(__file__),'data.csv'), 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(row)

class Comms(Node):

    def __init__(self):
        super().__init__('temp')
    
        self.subscription = self.create_subscription(
                Int16,
                '/temp',
                self.callback,
                QOS_PROFILE_DEFAULT)
        
        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

    def callback(self, Msg):
        print(Msg.data)

    def camera_image_callback(self, message):
		# Convert message to an n-dimensional numpy array representation of image.
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        append_values_to_csv(image)		

def main(args=None):
	rclpy.init(args=args)

	comms = Comms()

	rclpy.spin(comms)

	comms.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()