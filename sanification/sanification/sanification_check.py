import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from energy_map_msg.msg import EnergyMap
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class SanificationChecker(Node):
    def __init__(self):
        super().__init__('sanification_checker')

        self.map = None
        self.threshold_energy_map = None
        self.sanification_complete = Bool()
        self.sanification_complete.data = False

        self.generic_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.map_qos = self.generic_qos
        self.energy_qos = self.generic_qos
        self.sanification_qos = self.generic_qos

        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, self.map_qos)
        self.threshold_energy_publisher = self.create_subscription(OccupancyGrid, '/threshold_energy_map', self.thres_energy_map_callback, self.energy_qos)
        # Topic to communicate sanification has started
        self.sanification_publisher = self.create_publisher(Bool, '/is_sanitized', self.sanification_qos)

        self.check_period = 1.0
        self.energy_timer = self.create_timer(self.check_period, self.sanification_check)

    def map_callback(self, msg):
        self.map = np.transpose(np.array(msg.data).reshape(msg.info.height, msg.info.width))
        self.get_logger().info("Map received")
    
    def thres_energy_map_callback(self, msg):
        self.threshold_energy_map = np.transpose(np.array(msg.data).reshape(msg.info.height, msg.info.width))
        self.get_logger().info("Threshold energy map received")

    def publish_sanification(self):
        self.sanification_publisher.publish(self.sanification_complete)
        self.get_logger().info(f'Sanification State published: {self.sanification_complete.data}')

    def is_free(self, mx, my):
        return self.map[mx, my] == 0
    
    def is_sanitized(self, mx, my):
        return self.threshold_energy_map[mx, my] == 1
    
    def sanification_check(self):
        if self.map is not None and self.threshold_energy_map is not None:
            for x in range(self.map.shape[0]):
                for y in range(self.map.shape[1]):
                    if not self.is_free(x, y):
                        continue
                    if not self.is_sanitized(x, y):
                        self.sanification_complete.data = False
                        break
                    elif self.is_sanitized(x, y):
                        self.sanification_complete.data = True
            self.publish_sanification()

def main(args=None):
    rclpy.init(args=args)
    sanification_checker = SanificationChecker()

    # Spin the ROS2 node
    rclpy.spin(sanification_checker)

    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()
