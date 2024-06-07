import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import time
import numpy as np
from nav_msgs.msg import Odometry

TWO_PI = 6.28
ROTATION_VELOCITY = - 0.6


class InitialPositionNode(Node):

    def __init__(self):
        super().__init__('initial_position_node')

        # Subscribe to the amcl topic to get the initial position
        self.odom_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',  # Adjust the topic based on your robot configuration
            self.amcl_callback,
            10
        )
        
        # Publish the initial position to the initialpose topic
        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )
        
        # Subscribe to the odom topic to get the estimate on the robot current position
        self.odometry_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.covariance_treshold = 0.07 # vary this value to refine the estimated position

        self.covariance_msg = PoseWithCovarianceStamped()

        self.covariance_values = PoseWithCovarianceStamped()

        self.tb3_pose = [0, 0, 0]

        self.tb3_orientation = [0, 0, 0, 0]

        # Wait for the initial position to be obtained
        self.get_logger().info('Waiting for the initial position...')
        rclpy.spin_once(self, timeout_sec=1)  # Spin once to handle callbacks

    def amcl_callback(self, msg):
        # Check if the covariance of the position is below a certain treshold

        self.amcl_position = msg.pose.pose.position
        self.amcl_orientation = msg.pose.pose.orientation
        self.covariance_msg.pose.covariance = msg.pose.covariance

    def odom_callback(self,odom_msg):
        
        self.odom_data = odom_msg

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z

        x_o = odom_msg.pose.pose.orientation.x
        y_o = odom_msg.pose.pose.orientation.y
        z_o = odom_msg.pose.pose.orientation.z
        w_o = odom_msg.pose.pose.orientation.w

        self.covariance_values = odom_msg.pose.covariance

        self.tb3_pose = [x,y,z]

        self.tb3_orientation = [x_o, y_o, z_o, w_o]
    
    def publish_initial_pose(self):
        #publish an initial guess position on the topic initialpose

        rclpy.spin_once(self, timeout_sec = 1)
        initial_pose_msg = PoseWithCovarianceStamped()

        initial_pose_msg.pose.pose.position.x = float(self.tb3_pose[0])
        initial_pose_msg.pose.pose.position.y = float(self.tb3_pose[1])
        initial_pose_msg.pose.pose.position.z = float(self.tb3_pose[2])

        initial_pose_msg.pose.pose.orientation.x = float(self.tb3_orientation[0])
        initial_pose_msg.pose.pose.orientation.y = float(self.tb3_orientation[1])
        initial_pose_msg.pose.pose.orientation.z = float(self.tb3_orientation[2])
        initial_pose_msg.pose.pose.orientation.w = float(self.tb3_orientation[3])
        
        initial_pose_msg.pose.covariance = self.covariance_values

        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()


        self.amcl_pose_publisher.publish(initial_pose_msg)
        position_str = f'Position: x={initial_pose_msg.pose.pose.position.x}, y={initial_pose_msg.pose.pose.position.y}, z={initial_pose_msg.pose.pose.position.z}'
        self.get_logger().info(f'Publishing initial guess position on the topic initialpose. {position_str}')
        return

    def localization(self):

        i = False
        while True: 

            print('*-*-*-*-*-*-*-*-* LOCALIZATION ALGORITHM *-*-*-*-*-*-*-*-*')
            self.rotate()
            rclpy.spin_once(self, timeout_sec=1) # update the covariance matrix from the topic amcl
            i = self.check_covariance()
            if i:
                break

    def check_covariance(self):
        #check if the covariance is below a certain treshold
        covariance_values = self.covariance_msg.pose.covariance

        print('This is the actual covariance matrix')

        print(covariance_values)

        if np.max(covariance_values) < self.covariance_treshold:
            self.get_logger().info("All covariance values are below the threshold.")
            self.get_logger().info("Now the robot is localized into the map. You can perform the navigation")
            return True
        else:
            self.get_logger().warn("At least one covariance value is above the threshold.")
            return False

    def rotate(self):
        vel_msg = Twist() #create an angular velocity message
        vel_msg.angular.z = ROTATION_VELOCITY
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)
        time.sleep(- TWO_PI / vel_msg.angular.z) # rotate for some time
        self.stop() # stop the robot to check the covariance

    def stop(self):
      #stops the robot's velocity

        vel_msg = Twist() #create a velocity message, linear and angular velocity
        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0.0
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)
    
def main():
    rclpy.init()
    initial_position_node = InitialPositionNode()
    time.sleep(5)
    initial_position_node.publish_initial_pose()
    initial_position_node.localization()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
