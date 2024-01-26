import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class ODriveUARTNode(Node):
    def __init__(self):
        super().__init__('odrive_uart_node')
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        self.serial_port = serial.Serial(uart_port, baudrate=115200, timeout=0.1)

        self.pub_speed = self.create_publisher(Float32, 'motor_speed', 10)
        self.pub_temp = self.create_publisher(Float32, 'motor_temperature', 10)
        # 更多发布器可以在这里添加

        self.sub_speed = self.create_subscription(Float32, 'set_motor_speed', self.speed_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def speed_callback(self, msg):
        self.set_speed(msg.data)

    def set_speed(self, speed):
        command = f"v 0 {speed}\n"  # 设置轴 0 的速度
        self.serial_port.write(command.encode())

    def get_speed(self):
        self.serial_port.write(b"f 0\n")  # 查询轴 0 的速度
        response = self.serial_port.readline().decode().strip()
        return float(response)

    def get_temperature(self):
        # 这里填写获取电机温度的命令和解析逻辑
        return 0.0

    def timer_callback(self):
        #speed = self.get_speed()
        #temp = self.get_temperature()
        pass
        #self.pub_speed.publish(Float32(data=speed))
        #self.pub_temp.publish(Float32(data=temp))
        # 发布其他状态

def main(args=None):
    rclpy.init(args=args)
    node = ODriveUARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

