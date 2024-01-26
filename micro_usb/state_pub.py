import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from odrive.enums import *
import odrive

class ODrivePublisher(Node):
    def __init__(self):
        super().__init__('odrive_publisher')
        self.publisher_speed = self.create_publisher(Float32, 'motor_speed', 10)
        self.publisher_temp = self.create_publisher(Float32, 'motor_temperature', 10)
        self.publisher_voltage = self.create_publisher(Float32, 'bus_voltage', 10)
        self.odrive = odrive.find_any()
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # 读取电机数据
        speed = self.odrive.axis0.encoder.vel_estimate
        #temp = self.odrive.axis0.motor.fet_temperature
        voltage = self.odrive.vbus_voltage

        # 发布数据
        self.publisher_speed.publish(Float32(data=speed))
        #self.publisher_temp.publish(Float32(data=temp))
        self.publisher_voltage.publish(Float32(data=voltage))

def main(args=None):
    rclpy.init(args=args)
    odrive_publisher = ODrivePublisher()
    rclpy.spin(odrive_publisher)
    odrive_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

