import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
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
        self.pub_voltage = self.create_publisher(Float32, 'motor_voltage', 10)
        self.pub_error = self.create_publisher(String, 'motor_error', 10)

        self.sub_speed = self.create_subscription(Float32, 'set_motor_speed', self.speed_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def speed_callback(self, msg):
        self.set_speed(msg.data)

    def set_speed(self, speed):
        command = f"v 0 {speed}\n"
        self.serial_port.write(command.encode())

    def get_speed(self):
        self.serial_port.write(b"f 0\n")
        response = self.serial_port.readline().decode().strip()
        try:
            return float(response)
        except ValueError:
            print("no data")
            return 0.0

    def get_temperature(self):
        # 获取电机温度的命令和解析逻辑
        return 0.0

    def get_voltage(self):
        command = 'r vbus_voltage\n'.encode()  # 将字符串转换为字节并添加换行符
        self.serial_port.write(command)
        response = self.serial_port.readline().decode().strip()  # 读取响应并解码为字符串
        try:
            print(response)
            return float(response)
        except ValueError:
            print(f"Invalid response for voltage: '{response}'")
            return 0.0

    def get_error(self):
        # 获取电机错误状态的命令和解析逻辑
        return "No Error"

    def timer_callback(self):
        speed = self.get_speed()
        temp = self.get_temperature()
        voltage = self.get_voltage()
        error = self.get_error()

        self.pub_speed.publish(Float32(data=speed))
        self.pub_temp.publish(Float32(data=temp))
        self.pub_voltage.publish(Float32(data=voltage))
        self.pub_error.publish(String(data=error))

def main(args=None):
    rclpy.init(args=args)
    node = ODriveUARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

