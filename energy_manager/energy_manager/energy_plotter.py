import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from energy_map_msg.msg import EnergyMap
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


class EnergyPlotter(Node):
    
    def __init__(self):
        super().__init__("energy_plotter")

        self.generic_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
 
        self.energy_map = None
        self.threshold_energy_map = None
        # self.room_energy_map = None

        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, self.generic_qos)
        self.energy_subsciber = self.create_subscription(EnergyMap, "/energy_map", self.energy_callback, self.generic_qos)
        self.threshold_energy_subsciber = self.create_subscription(OccupancyGrid, '/threshold_energy_map', self.threshold_energy_callback, self.generic_qos)
        # self.room_energy_subsciber = self.create_subscription(OccupancyGrid, '/room_energy_map', self.room_energy_callback, self.generic_qos)
        
        self.plotting_period = 1.0
        self.energy_timer = self.create_timer(self.plotting_period, self.plot)

    def map_callback(self, msg: EnergyMap):
        self.get_logger().info("Map received")
        self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        
    def energy_callback(self, msg: EnergyMap):
        self.get_logger().info("Energy map received")
        self.energy_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def threshold_energy_callback(self, msg: OccupancyGrid):
        self.get_logger().info("Threshold energy map received")
        self.threshold_energy_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    # def room_energy_callback(self, msg: OccupancyGrid):
    #     self.get_logger().info("Room energy map received")
    #     self.room_energy_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def plot(self):
        if self.energy_map is not None and self.threshold_energy_map is not None:# and self.room_energy_map is not None:
            self.get_logger().info("Plotting energy maps")
            plt.figure(figsize=(9, 5))

            plt.subplot(1, 2, 1)
            plt.imshow(np.rot90(np.transpose(self.energy_map), 2))
            # plt.imshow(np.transpose(self.energy_map))
            plt.title('Energy Map')
            plt.colorbar()

            plt.subplot(1, 2, 2)
            plt.imshow(np.rot90(np.transpose(self.threshold_energy_map), 2))
            # plt.imshow(np.transpose(self.threshold_energy_map))
            plt.title('Energy Threshold Map\n0 -> Not sanitized  1 -> Sanitized')
            plt.colorbar()

            # plt.subplot(1, 3, 3)
            # plt.imshow(np.rot90(np.transpose(self.room_energy_map), 2))
            # plt.title('Energy Room Map')

            plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = EnergyPlotter()
    rclpy.spin(node)

if __name__ == "__main__":
    main()