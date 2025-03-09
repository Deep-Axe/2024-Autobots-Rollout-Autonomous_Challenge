import rclpy
from rclpy.node import Node
from synapse_msgs.msg import TrafficStatus

import cv2
import numpy as np


from sensor_msgs.msg import CompressedImage
import os
QOS_PROFILE_DEFAULT = 10

#Custom
from ultralytics import YOLO
import logging

logging.getLogger('ultralytics').setLevel(logging.CRITICAL)

path = os.path.join(os.path.dirname(__file__),'Yolov8Model1_stop_sign.pt')
model = YOLO(path)

class ObjectRecognizer(Node):
	""" Initializes object recognizer node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('object_recognizer')

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for traffic status.
		self.publisher_traffic = self.create_publisher(
			TrafficStatus,
			'/traffic_status',
			QOS_PROFILE_DEFAULT)

	""" Analyzes the image received from /camera/image_raw/compressed to detect traffic signs.
		Publishes the existence of traffic signs in the image on the /traffic_status topic.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CompressedImage.html"

		Returns:
			None
	"""
	def camera_image_callback(self, message):
		# Convert message to an n-dimensional numpy array representation of image.
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		traffic_status = TrafficStatus()
		
		results = model.predict(source=image, imgsz=640, conf=0.25)
		for result in results:
			if result.boxes:  
				#p = 100*(result.boxes.xywh[0][2]*result.boxes.xywh[0][3])/(result.boxes.orig_shape[0]*result.boxes.orig_shape[1])
				boxes = result.boxes.xyxy.cpu().numpy()
				#confs = result.boxes.conf.cpu().numpy()
				#classes = result.boxes.cls.cpu().numpy()

				#for box, conf, cls in zip(boxes, confs, classes):
				areas = []
				for box in boxes:
					x1, y1, x2, y2 = map(int, box)
					width = x2 - x1
					height = y2 - y1
					areas.append(width*height)
				print(areas)
				if result.boxes.conf.tolist()[0] > 0.95 and max(areas) > 400:
					#print(result.boxes.conf.tolist()[0])
					traffic_status.stop_sign = True
		
		self.publisher_traffic.publish(traffic_status)

'''        # For visualization purposes (optional)
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, 'Stop Sign', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)'''


def main(args=None):
	rclpy.init(args=args)

	object_recognizer = ObjectRecognizer()

	rclpy.spin(object_recognizer)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	object_recognizer.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
