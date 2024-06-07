import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import cv2

class VisibilityGridPublisher(Node):
    def __init__(self):
        super().__init__('visibility_grid_publisher')

        self.map = None
        self.matrix_map = None
        # self.downsampled_matrix_map = None
        self.visibility_map = OccupancyGrid()
        # self.downsampled_visibility_map = OccupancyGrid()
        self.robot_pose = None
        # self.downsampling_factor = 4

        self.generic_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, self.generic_qos)
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, self.generic_qos)
        self.visibility_map_publisher = self.create_publisher(OccupancyGrid, '/visibility_grid', self.generic_qos)

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.get_logger().info(f"Robot pose received")
        self.visibility_grid_calculator()

    def map_callback(self, msg):
        self.map = msg
        # self.get_logger().info(f"{self.map.info.height}")
        # self.get_logger().info(f"{self.map.info.width}")
        self.get_logger().info("Map received")
        return

    def visibility_map_publish(self,):
        if self.matrix_map is not None:
            self.get_logger().info("Publishing the map")
            self.visibility_map.header = self.map.header
            self.visibility_map.info = self.map.info
            self.matrix_map = self.matrix_map.astype(np.int8)
            self.visibility_map.data = self.matrix_map.flatten().tolist()
            self.visibility_map_publisher.publish(self.visibility_map)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")
        return (mx, my)

    def visibility_within_cone(self, grid, u_direction, v_direction):
        u = np.asarray(u_direction, dtype=int)
        v = np.asarray(v_direction, dtype=int)
        origin = np.array([0,0], dtype=int)
        dims = np.asarray(grid.shape, dtype=int)
        m = 0
        k = 0
        position = np.array([0,0], dtype=int)
        while np.all(position < dims):
            while np.all(position < dims):
                if not np.all(position == origin):
                    pos = tuple(position)
                    pos_minus_u = tuple(np.maximum(origin, position - u))
                    pos_minus_v = tuple(np.maximum(origin, position - v))
                    grid[pos] *= (m * grid[pos_minus_u] + k * grid[pos_minus_v]) / (m + k)
                k += 1
                position += v
            m += 1
            k = 0
            position = m * u
        grid[:] = (grid >= 0.5)

    def visibility_grid_calculator(self):
        if self.map is not None and self.robot_pose is not None:
            self.get_logger().info("Map and pose are available")

            self.matrix_map = np.where(np.transpose(np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)) == 0, 1, 0)
            mx, my = self.worldToMap(self.robot_pose.position.x,self.robot_pose.position.y)

            # The instructions below are referred to the 4-neighborhood version of the algorithm
            # Discarded because of insufficient accuracy
            # self.visibility_within_cone(matrix_map[mx:, my:], [1,0], [0,1])
            # self.visibility_within_cone(matrix_map[mx::-1, my:], [1,0], [0,1])
            # self.visibility_within_cone(matrix_map[mx::-1, my::-1], [1,0], [0,1])
            # self.visibility_within_cone(matrix_map[mx:, my::-1], [1,0], [0,1])

            # The instructions below are referred to the 8-neighborhood version of the algorithm
            # Discarded since it is less precise than the 16-neighborhood version and it is not so much faster
            # self.visibility_within_cone(matrix_map[mx:, my:], [1,1], [1,0])
            # self.visibility_within_cone(matrix_map[mx:, my:], [1,1], [0,1])
            # self.visibility_within_cone(matrix_map[mx::-1, my:], [1,1], [1,0])
            # self.visibility_within_cone(matrix_map[mx::-1, my:], [1,1], [0,1])
            # self.visibility_within_cone(matrix_map[mx::-1, my::-1], [1,1], [1,0])
            # self.visibility_within_cone(matrix_map[mx::-1, my::-1], [1,1], [0,1])
            # self.visibility_within_cone(matrix_map[mx:, my::-1], [1,1], [1,0])
            # self.visibility_within_cone(matrix_map[mx:, my::-1], [1,1], [0,1])

            # The instructions below are referred to the 16-neighborhood version of the algorithm
            # Chosen since it is the most precide and it is not particolarlu slower than the previous two
            self.visibility_within_cone(self.matrix_map[mx:, my:], [2,1], [1,0])
            self.visibility_within_cone(self.matrix_map[mx:, my:], [2,1], [1,1])
            self.visibility_within_cone(self.matrix_map[mx:, my:], [1,2], [1,1])
            self.visibility_within_cone(self.matrix_map[mx:, my:], [1,2], [0,1])
            self.visibility_within_cone(self.matrix_map[mx::-1, my:], [2,1], [1,0])
            self.visibility_within_cone(self.matrix_map[mx::-1, my:], [2,1], [1,1])
            self.visibility_within_cone(self.matrix_map[mx::-1, my:], [1,2], [1,1])
            self.visibility_within_cone(self.matrix_map[mx::-1, my:], [1,2], [0,1])
            self.visibility_within_cone(self.matrix_map[mx::-1, my::-1], [2,1], [1,0])
            self.visibility_within_cone(self.matrix_map[mx::-1, my::-1], [2,1], [1,1])
            self.visibility_within_cone(self.matrix_map[mx::-1, my::-1], [1,2], [1,1])
            self.visibility_within_cone(self.matrix_map[mx::-1, my::-1], [1,2], [0,1])
            self.visibility_within_cone(self.matrix_map[mx:, my::-1], [2,1], [1,0])
            self.visibility_within_cone(self.matrix_map[mx:, my::-1], [2,1], [1,1])
            self.visibility_within_cone(self.matrix_map[mx:, my::-1], [1,2], [1,1])
            self.visibility_within_cone(self.matrix_map[mx:, my::-1], [1,2], [0,1])

        
            self.matrix_map=np.where(np.transpose(self.matrix_map) == 0, 100, 0)

            self.get_logger().info("Visibility map calculated")
            self.visibility_map_publish()        

def main(args=None):
    rclpy.init(args=args)
    visibility_grid_publisher = VisibilityGridPublisher()

    # Spin the ROS2 node
    rclpy.spin(visibility_grid_publisher)

    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()
