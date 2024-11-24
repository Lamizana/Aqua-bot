#!/usr/bin/env python3
# --------------------------------------------------------- #
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

# --------------------------------------------------------- #
class CameraPositioner(Node):
    """
    Permet le deplacement de la camera
    """
    def __init__(self):
        super().__init__('camera_positioner')
        self.publisher = self.create_publisher(
            Float64,
            '/aquabot/thrusters/main_camera_sensor/pos',
            10
        )
        self.set_camera_position()

    def set_camera_position(self):
        msg = Float64()
        msg.data = math.radians(-90)  # Convertir -90 degrés en radians
        self.publisher.publish(msg)
        self.get_logger().info(f'Camera set to -90 degrees ({msg.data} radians)')


# --------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = CameraPositioner()
    try:
        rclpy.spin_once(node, timeout_sec=1.0)  # Permet d'exécuter une seule fois
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------- #
if __name__ == '__main__':
    main()
