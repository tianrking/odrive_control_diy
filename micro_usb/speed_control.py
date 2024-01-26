import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import odrive
from odrive.enums import *

class ODriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')
        self.subscription = self.create_subscription(
            Float32,
            'set_velocity',
            self.velocity_callback,
            10)
        self.subscription  # 防止订阅被垃圾回收

        self.odrv0 = odrive.find_any()
        self.axis = self.odrv0.axis0  # 假设使用 axis0，根据实际情况调整

        # ODrive 初始化
        #self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    def velocity_callback(self, msg):
        self.get_logger().info('Setting velocity: "%s"' % msg.data)
        self.axis.controller.input_vel = msg.data

def main(args=None):
    rclpy.init(args=args)

    odrive_node = ODriveNode()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

