import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from energy_map_msg.msg import EnergyMap
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import os
import itertools

class EnergyCalculator(Node):
    def __init__(self):
        super().__init__('energy_calculator')

        self.map = None
        self.robot_pose = None
        self.visibility_map = None
        self.energy_map = EnergyMap()
        self.energy_matrix = None
        self.threshold_energy_map = OccupancyGrid()
        self.downsampled_threshold_energy_matrix = None
        self.sanification_complete = Bool()
        self.sanification_complete.data = False
        self.room_complete = Bool()
        # self.power = 0.1 #[Wm^2]
        self.power = 0.3 #[Wm^2]
        self.rooms_boxes = []
        self.curr_room = None
        self.room_energy_map = None

        self.generic_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, self.generic_qos)
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, self.generic_qos)
        self.visibility_subscriber = self.create_subscription(OccupancyGrid, '/visibility_grid', self.visibility_callback, self.generic_qos)
        self.room_subscriber = self.create_subscription(Int8, '/curr_room', self.room_callback, self.generic_qos)
        self.energy_publisher = self.create_publisher(EnergyMap, '/energy_map', self.generic_qos)
        self.threshold_energy_publisher = self.create_publisher(OccupancyGrid, '/threshold_energy_map', self.generic_qos)
        self.sanification_publisher = self.create_publisher(Bool, '/is_sanitized', self.generic_qos)
        self.room_sanification_publisher = self.create_publisher(Bool, '/room_is_sanitized', self.generic_qos)

        self.energy_period = 0.5
        self.energy_timer = self.create_timer(self.energy_period, self.energy_calculator)
        self.publish_threshold_timer = self.create_timer(self.energy_period, self.threshold_energy_map_publish)
        self.publish_energy_timer = self.create_timer(self.energy_period, self.energy_map_publish)
        # self.plot_room_energy_timer = self.create_timer(self.energy_period, self.plot_room_energy)

        path_to_boxes_file = "/home/student/ros2/sanitizer_ws/src/sanification/boxes.txt"

        # Reading the rooms boxes text file
        if os.path.exists(path_to_boxes_file):
            with open(path_to_boxes_file, "r") as file:
                lines = file.readlines()
                for line in lines:
                    self.rooms_boxes.append(tuple(map(int, line.strip().split())))
        else:
            self.get_logger().info(f"The path {path_to_boxes_file} doesn't exist")
        self.get_logger().info(f'{self.rooms_boxes}')


    def map_callback(self, msg):
        self.map = msg
        self.energy_matrix = np.zeros((self.map.info.width, self.map.info.height))
        self.downsampled_threshold_energy_matrix = np.zeros_like(self.energy_matrix)
        for x in range(self.downsampled_threshold_energy_matrix.shape[0]):
            for y in range(self.downsampled_threshold_energy_matrix.shape[1]):
                if self.is_free(x, y):
                    self.downsampled_threshold_energy_matrix[x][y] = 2
        self.get_logger().info("Map received")

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.get_logger().info("Robot pose received")
    
    def visibility_callback(self, msg):
        self.visibility_map = np.transpose(np.array(msg.data).reshape(msg.info.height, msg.info.width))
        self.get_logger().info("Visibility map received")

    def room_callback(self, msg):
        self.get_logger().info(f'Current room: {self.curr_room}, received room: {msg.data}')
        if self.curr_room != msg.data:
            self.room_complete.data = False
            self.publish_room_sanification()
        self.curr_room = msg.data
        self.get_logger().info("Current room received")

    def energy_map_publish(self):
        if self.energy_matrix is not None:
            self.get_logger().info("Publishing energy map")
            self.energy_map.header = self.map.header
            self.energy_map.info = self.map.info
            self.energy_matrix = self.energy_matrix.astype('float32')
            self.energy_map.data = np.transpose(self.energy_matrix).flatten().tolist()
            self.energy_publisher.publish(self.energy_map)
    
    def threshold_energy_map_publish(self):
        if self.downsampled_threshold_energy_matrix is not None:
            self.get_logger().info("Publishing threshold energy map")
            self.threshold_energy_map.header = self.map.header
            self.threshold_energy_map.info = self.map.info
            self.downsampled_threshold_energy_matrix = self.downsampled_threshold_energy_matrix.astype(np.int8)
            self.threshold_energy_map.data = np.transpose(self.downsampled_threshold_energy_matrix).flatten().tolist()
            self.threshold_energy_publisher.publish(self.threshold_energy_map)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")
        return (mx, my)

    def distance(self, mx1, my1, mx2, my2):
        return (((mx1 - mx2)*self.map.info.resolution)**2 + ((my1 - my2)*self.map.info.resolution)**2)**0.5

    def is_visible(self, mx, my):
        return self.visibility_map[mx, my] == 0
    
    def is_free(self, mx, my):
        return np.transpose(np.array(self.map.data).reshape(self.map.info.height, self.map.info.width))[mx, my] == 0
    
    def is_sanitized(self, mx, my):
        return self.downsampled_threshold_energy_matrix[mx, my] == 1
    
    def publish_sanification(self):
        self.sanification_publisher.publish(self.sanification_complete)
        self.get_logger().info(f'Sanification State published: {self.sanification_complete.data}')

    def publish_room_sanification(self):
        self.room_sanification_publisher.publish(self.room_complete)
        self.get_logger().info(f'Room Sanification State published: {self.room_complete.data}')
        
    def energy_calculator(self):
        if self.map is not None and self.robot_pose is not None and self.visibility_map is not None and self.sanification_complete is not None:
            if not self.sanification_complete.data:
                self.get_logger().info("Calculating energy")
                mx, my = self.worldToMap(self.robot_pose.position.x,self.robot_pose.position.y)
                for x in range(self.energy_matrix.shape[0]):
                    for y in range(self.energy_matrix.shape[1]):
                        if self.is_visible(x, y) and self.is_free(x, y):
                            distance = self.distance(mx, my, x, y)
                            if distance >= 0.1 and self.energy_matrix[x][y] < 20:
                                self.energy_matrix[x][y] += self.power * self.energy_period / distance

                downsampling_factor = 6

                self.get_logger().info("Calculating downsampled energy map")
                # Calculate the downsampled result only for free cells
                for i in range(0, self.energy_matrix.shape[0]):
                    for j in range(0, self.energy_matrix.shape[1]):
                        row_slice = slice(i, min(i + downsampling_factor, self.energy_matrix.shape[0]))
                        col_slice = slice(j, min(j + downsampling_factor, self.energy_matrix.shape[1]))
                        cell_sanification = self.energy_matrix[row_slice, col_slice]
                        
                        # Keep only free cells
                        cell_sanification = cell_sanification[self.is_free(row_slice, col_slice)]
                        if len(cell_sanification) > 0:
                            # Compute average sanification of the downsampling_factor x downsampling_factor tile
                            average_sanitation = np.mean(cell_sanification)
                        else:
                            average_sanitation = 0
                                                
                        # Assign 1 to the tile if the average sanification level is at least 10
                        if average_sanitation >= 10:
                            self.downsampled_threshold_energy_matrix[row_slice, col_slice] = 1
                
                room_left = self.rooms_boxes[self.curr_room-1][1]
                room_right = self.rooms_boxes[self.curr_room-1][3]
                room_top = self.rooms_boxes[self.curr_room-1][2]
                room_bottom = self.rooms_boxes[self.curr_room-1][4]

                self.room_energy_map = self.downsampled_threshold_energy_matrix[room_left:room_right, room_top:room_bottom]
                self.get_logger().info('Checking if the room has been sanitized')
                for x, y in itertools.product(range(room_left, room_right+1), range(room_top, room_bottom+1)):
                        if not self.is_free(x, y):
                            self.get_logger().info('Occupied Cell')
                            continue
                        if not self.is_sanitized(x, y):
                            self.get_logger().info('One free cell in the room is not sanitized')
                            self.room_complete.data = False
                            break
                        elif self.is_sanitized(x, y):
                            self.room_complete.data = True
                if self.room_complete.data:
                    self.get_logger().info('The room has been sanitized')
                self.publish_room_sanification()

                for pos in np.ndindex(self.downsampled_threshold_energy_matrix.shape):
                    if not self.is_free(pos[0], pos[1]):
                        continue
                    if not self.is_sanitized(pos[0], pos[1]):
                        self.sanification_complete.data = False
                        break
                    elif self.is_sanitized(pos[0], pos[1]):
                        self.sanification_complete.data = True
                self.publish_sanification()

    # def plot_room_energy(self):
    #     if self.room_energy_map is not None:
    #         plt.imshow(plt.imshow(np.rot90(np.transpose(self.room_energy_map), 2)))
    #         plt.colorbar()
    #         plt.show()

def main(args=None):
    rclpy.init(args=args)
    energy_calculator = EnergyCalculator()

    # Spin the ROS2 node
    rclpy.spin(energy_calculator)

    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()