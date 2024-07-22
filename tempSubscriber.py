import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

class Comms(Node):

    def __init__(self):
        super().__init__('temp')
    
        self.subscription = self.create_subscription(
                Int16,
                '/temp',
                self.callback,
                10)
        
    def callback(self, Msg):
        print(Msg.data)

def main(args=None):
	rclpy.init(args=args)

	comms = Comms()

	rclpy.spin(comms)

	comms.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()