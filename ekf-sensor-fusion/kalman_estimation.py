import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np


from filters.kalman_filter import KalmanFilter
import sensor_utils as s_utils

class KalmanEstimator(Node):
    def __init__(self):
        super.__init__("Kalman_estimator")

        # msg assigned to collect odometry measurements from Wheel encoder : observation
        self.odom_wheelenc_subscription = self.create_subscription(
            Odometry,
            "odom_wheel_enc",
            self.odom_wheel_enc_callback,
            10
        )

        # msg assigned for collecting odometry estimated by Kalman filter : Belief
        self.odom_estimated_subscription = self.create_subscription(
            Odometry,
            "odom_estimate",
            self.odom_estimate_callback,
            10
        )

        self.initial_state = np.zeros(3)  # mu_0
        self.initial_covariance = np.eye(3) # Sigma_0
        self.u = np.zeros(2)  # initial action [ v , omega]
        
        # action model noise v(t)
        motion_noise_std = [2.0, 2.0, 2.0] # v(t)
        observation_noise_std = [2.0, 2.0, 2.0] # w(t)

        self.kf = KalmanFilter(self.initial_state, self.initial_covariance, motion_noise_std, observation_noise_std)
        self.prediction_done = False # signals the update step

    
    def odom_estimate_callback(self,msg):
        self.u = np.asarray([msg.twist.twist.linear.x, msg.twist.twist.angular.z])  # control inputs

        # compute dt
        curr_time = self.get_clock().now().nanoseconds
        if self.prev_time:
            dt = (curr_time - self.prev_time) / 1e9 #seconds
        else:
            dt = 0.0

        #Prediction
        mu,Sigma = self.kf.predict(self.u,dt)
        self.prediction_done = True
        self.prev_time = curr_time
        print("predicted pos",mu)

        #visualize



    # Update Prior to get Kalman estimate
    def odom_wheel_enc_callback(self,msg):

        # Set the initial pose
        if not self.initial_pose:
            self.initial_pose = s_utils.odom_pose(msg)

        # normalize the pose
        current_pose = s_utils.odom_pose(msg)
        self.normalized_pose = np.array(s_utils.get_normalized_pose2D(self.initial_pose, current_pose))

        if self.prediction_done:

            z = self.normalized_pose
            mu, Sigma = self.kf.update(z) # obtain posterior belief 
            print(f'updated pose with new observation {self.normalized_pose} : {mu}')

            # Visualize


def main(args=None):
    rclpy.init(args=args)
    kalman_filter_node = KalmanEstimator()
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()


