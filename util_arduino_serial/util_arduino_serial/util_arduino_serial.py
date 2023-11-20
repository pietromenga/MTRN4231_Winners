import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('util_arduino_serial')
        self.get_logger().info(f"Start")
        # create subscriber to the endeffector command
        self.subscription = self.create_subscription(String, 'arduino', self.command_callback, 10)
        # Open serial port of arduino
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600) 
        # Check seral port
        print(self.serial_port)
	
    def command_callback(self, msg):
        #if the the msg from the arudino topic is start then active the end-effector for launching
        if msg.data == "Start":
            # print msg to terminal
            self.get_logger().info(f"Initalise arduino")
            # send to serial of the arudino so it can read
            self.serial_port.write(b'1')


def main(args=None):
    rclpy.init(args=args)
    util_arduino_serial = ArduinoNode()
    rclpy.spin(util_arduino_serial)
    util_arduino_serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
