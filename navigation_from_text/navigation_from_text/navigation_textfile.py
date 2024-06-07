import rclpy
import time
from rclpy.node import Node
import os

from geometry_msgs.msg import PoseStamped
from tf2_ros import Duration

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class NavigatorFromTextfile(Node):

    def __init__(self):
        super().__init__('action_server_control')
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait for the action server to be ready
        #self.action_client.wait_for_server()

        print('Init completed')
    
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
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    
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

     # Do something depending on the return code
        result = self.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            self.info('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

def main(args=None):

    rclpy.init()
    navigator = NavigatorFromTextfile()

    print("*-*-*-*-*-*-*-*-* NAVIGATION TO GOALS FROM TEXT FILE ALGORITHM *-*-*-*-*-*-*-*-*")
    
    path_to_textfile = "/home/student/ros2/sanitizer_ws/src/navigation_from_text/goals.txt"

    # read the goals from the text file
    if os.path.exists(path_to_textfile):
        with open("/home/student/ros2/sanitizer_ws/src/navigation_from_text/goals.txt", "r") as file:
            lines = file.readlines()
            for line in lines:
                goal = line.split()
                goal_x = float(goal[0])
                goal_y = float(goal[1])
                goal_w = float(goal[2])
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = float(goal_x)
                goal_pose.pose.position.y = float(goal_y)
                goal_pose.pose.orientation.w = float(goal_w)
                navigator.goToPose(goal_pose)
                navigator.navigation()
                time.sleep(2) # wait a little bit before proceeding with the next goal
            navigator.info("All the", str(len(lines)) ,"goals contained into the text file were achieved by the sanitizer robot")

    else:
        navigator.info("The specified path doesn't exist")


if __name__ == '__main__':
    main()