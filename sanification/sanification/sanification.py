import rclpy
import time
from rclpy.node import Node
import os
import numpy as np
from scipy.spatial.distance import cdist

from geometry_msgs.msg import PoseStamped
from tf2_ros import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int8

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class Sanitizer(Node):

    def __init__(self):
        super().__init__('action_server_control')

        self.map = None
        self.gcostmap = None
        self.robot_pose = None
        self.is_sanitized = Bool()
        self.is_sanitized.data = False
        self.room_is_sanitized = Bool()
        self.room_is_sanitized.data = False

        self.centers = []
        self.points_in_rooms = []

        self.generic_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Map subscriber for the mapToWorld and worldToMap conversions
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, self.generic_qos)

        # Global costmap subscriber for points adjustments
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.gcostmap_callback, self.generic_qos)

        # Amcl subscriber for the rooms reordering
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, self.generic_qos)

        self.room_publisher = self.create_publisher(Int8, '/curr_room', self.generic_qos)

        # Topic to communicate sanification has been completed
        self.sanification_subscriber = self.create_subscription(Bool, '/is_sanitized', self.sanification_callback, self.generic_qos)

        # Topic to communicate sanification has been completed
        self.room_sanification_subscriber = self.create_subscription(Bool, '/room_is_sanitized', self.room_sanification_callback, self.generic_qos)
        
        # Action client for the navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    ##### MAP RELATED FUNCTIONS #####
    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info("Map received")

    def gcostmap_callback(self, msg):
        self.gcostmap = msg
        self.get_logger().info("Global costmap received")

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return (wx, wy)
    
    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")
        return (mx, my)
    
    ##### RECEIVE ROBOT POSE #####
    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.info("Robot pose received")

    #### RECEIVE SANIFICATION STATES (ROOM AND ENTIRE ENVIRONMENT) #####
    def sanification_callback(self, msg):
        self.is_sanitized = msg
        self.get_logger().info(f"Sanification state received -> {self.is_sanitized.data}")

    def room_sanification_callback(self, msg):
        self.room_is_sanitized = msg
        self.get_logger().info(f"Room Sanification state received -> {self.room_is_sanitized.data}")

    
    ##### DEFINE ROOMS ORDER FOR SANIFICATION #####
    def distance(self, x1, y1, x2, y2):
        return ((x1 - x2)**2 + (y1 - y2)**2)**0.5
    
    def reorder_centers(self):
        # Sort centers based on their distance from the initiali robot pose
        sorted_points = [(-1, self.robot_pose.position.x, self.robot_pose.position.y)]
        
        for _ in range(len(self.centers)):
            current_point = sorted_points[-1]
            remaining_points = [point for point in self.centers if point[0] not in [p[0] for p in sorted_points]]
            # self.info(f"remaining points {remaining_points}, current point {current_point}")
            next_point = min(remaining_points, key=lambda point: self.distance(point[1], point[2], current_point[1], current_point[2]))
            sorted_points.append(next_point)
        
        self.centers = sorted_points[1:]  # Exclude the initial position from the result

    ##### MOVE COVERAGE PATH POINTS TO COST FREE CELLS #####
    def verify_and_replace_points(self):
        if self.gcostmap is not None:
            # Reshape the 1D array into a 2D array
            costmap_data = np.transpose(np.array(self.gcostmap.data).reshape((self.gcostmap.info.height, self.gcostmap.info.width)))

            for i, point in enumerate(self.points_in_rooms):
                mx, my = self.worldToMap(point[1], point[2])

                if not self.is_cost_free(mx, my, costmap_data):
                    # If the point has non-zero cost, find the nearest valid point with zero cost
                    nearest_valid_point = self.find_nearest_cost_free_point(mx, my, costmap_data)
                    self.points_in_rooms[i][1], self.points_in_rooms[i][2] = self.mapToWorld(nearest_valid_point[0], nearest_valid_point[1])

    def is_cost_free(self, x, y, costmap_data):
        return costmap_data[x, y] == 0

    def find_nearest_cost_free_point(self, x, y, costmap_data):
        valid_points = np.argwhere(costmap_data == 0)
        distances =cdist([(x, y)], valid_points)
        min_distance_index = np.argmin(distances)
        return valid_points[min_distance_index]
    
    ##### SEND ROOM UNDER SANFIFICATION #####
    def publish_room(self, room):
        msg = Int8()
        msg.data = room
        self.room_publisher.publish(msg)

    ##### NAVIGATION #####
    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                    self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        return True
    
    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status
    
    def waitUntilNav2Active(self):
        self.info('Wait for Nav2 to be ready...')
        self._waitForNodeToActivate('amcl')
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return
    
    def navigation(self):
        i = 0
        while not self.isNavComplete():
        # Print the time to arrival and, in case, abort the navigation if it's taking too long
            i = i + 1
            feedback = self.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')            
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelNav()

        # Print the navigation outcome
        result = self.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            self.info('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    
    ##### LOGGING #####
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
            
def main(args=None):

    rclpy.init()
    sanitizer = Sanitizer()

    path_to_points_file = "/home/student/ros2/sanitizer_ws/src/sanification/points_to_reach.txt"
    # path_to_world_points_file = "/home/student/ros2/sanitizer_ws/src/sanification/world_points_to_reach.txt"
    # path_to_verified_points_file = "/home/student/ros2/sanitizer_ws/src/sanification/verified_points_to_reach.txt"
    path_to_centers_file = "/home/student/ros2/sanitizer_ws/src/sanification/centers.txt"
    # path_to_ordered_centers_file = "/home/student/ros2/sanitizer_ws/src/sanification/ordered_centers.txt"

    # Wait for the map topic
    while sanitizer.map is None:
        sanitizer.info("Waiting for the map")
        rclpy.spin_once(sanitizer)

    # Wait for the robot position topic
    while sanitizer.robot_pose is None:
        sanitizer.info("Waiting for the robot position")
        rclpy.spin_once(sanitizer)

    # Reading the rooms centers text file
    if os.path.exists(path_to_centers_file):
        with open(path_to_centers_file, "r") as file:
            lines = file.readlines()
            for line in lines:
                point = line.split()
                sanitizer.centers.append((int(point[0]),) + sanitizer.mapToWorld(float(point[1]), float(point[2])))
    else:
        sanitizer.info(f"The path {path_to_centers_file} doesn't exist")
    
    # Reorder the centers list from the closest to the furthest wrt the robot pose
    sanitizer.reorder_centers()
    # with open(path_to_ordered_centers_file, 'w') as f:
    #     for point in sanitizer.centers:
    #         f.write(' '.join(map(str, point)) + '\n')

    # Reading the rooms point text file
    if os.path.exists(path_to_points_file):
        with open(path_to_points_file, "r") as file:
            lines = file.readlines()
            for line in lines:
                point = line.split()
                sanitizer.points_in_rooms.append([int(point[0])] + list(sanitizer.mapToWorld(float(point[1]), float(point[2]))) + [float(point[3])])
    else:
        sanitizer.info(f"The path {path_to_points_file} doesn't exist")
    
    # with open(path_to_world_points_file, 'w') as f:
    #     for point in sanitizer.points_in_rooms:
    #         f.write(' '.join(map(str, point)) + '\n')
    
    # Check if some points fall in areas occupied or really close to obstacles and replace them with the closest free point 
    sanitizer.verify_and_replace_points()
    # with open(path_to_verified_points_file, 'w') as f:
    #     for point in sanitizer.points_in_rooms:
    #         f.write(' '.join(map(str, point)) + '\n')

    # Start sanification
    print(f'Current sanification state -> {sanitizer.is_sanitized.data}')
    if not sanitizer.is_sanitized.data:
        # Order points following the reordered centers
        ordered_points = [point for center in sanitizer.centers for point in sanitizer.points_in_rooms if point[0] == center[0]]
        with open('ordered_points.txt', 'w') as f:
            for item in ordered_points:
                f.write(' '.join(map(str, item)) + '\n')
        i = 0
        first_room_point = 0
        curr_room = ordered_points[0][0]
        while not sanitizer.is_sanitized.data:
            point = ordered_points[i]
            # If the new point is in a new room and the current one has been completely santized, change room 
            if point[0] != curr_room and sanitizer.room_is_sanitized.data:
                sanitizer.info(f'Room {curr_room} has been sanitized, going to room {point[0]}')
                curr_room = point[0]
                first_room_point = i
                sanitizer.publish_room(curr_room)
                while sanitizer.room_is_sanitized.data:
                    rclpy.spin_once(sanitizer)
            # If the new point is in a new room and the current one has not been completely santized, restart the room
            elif point[0] != curr_room and not sanitizer.room_is_sanitized.data:
                sanitizer.info(f'Room {curr_room} has not been completely sanitized yet, restarting sanification')
                i = first_room_point
                continue
            # Otherwise, keep sanitizing the current room
            elif point[0] == curr_room and sanitizer.room_is_sanitized.data:
                if i < len(ordered_points) - 1:
                    i += 1
                continue
            else:
                sanitizer.publish_room(curr_room)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = sanitizer.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(point[1])
            goal_pose.pose.position.y = float(point[2])
            goal_pose.pose.orientation.w = float(point[3])
            sanitizer.goToPose(goal_pose)
            sanitizer.navigation()
            time.sleep(4.0) # wait a little bit before proceeding with the next goal
            if i < len(ordered_points) - 1:
                i += 1
            elif i == len(ordered_points)-1:
                i = first_room_point

        if sanitizer.is_sanitized.data:
            print('The sanification has been completed')

if __name__ == '__main__':
    main()