import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Twist
from sam_msgs.msg import ThrusterAngles
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
from functools import partial
from math import atan2

class PositionFilter(Node):

    def __init__(self):
        super().__init__('position_filter')

        # Initialize class variables
        self.prev_heading, self.current_heading = None, None
        self.rpm = 0
        self.horizontal_thrust_vector = 0.
        self.dt = 0.1

        # Subscribers containing the range of the follower from the leaders
        self.leader1_distance_sub = self.create_subscription(Float32, "/follower/distance_to_leader1", self.leader1_distance_cb, 1)
        self.leader2_distance_sub = self.create_subscription(Float32, "/follower/distance_to_leader2", self.leader2_distance_cb, 1)

        # Subscribers containing other important information for the follower: heading, rpm, thrust vector
        # self.heading_sub = self.create_subscription(Float32, "/follower/heading", 
        #     lambda msg: setattr(self, 'current_heading', msg.data), 10)
        # self.rpm_sub = self.create_subscription(Float32, "/follower/rpm",
        #     lambda msg: setattr(self, 'rpm', msg.rpm), 10)
        self.thrust_vector_sub = self.create_subscription(ThrusterAngles, "/follower/core/thrust_vector_cmd", 
            lambda msg: setattr(self, 'horizontal_thrust_vector', msg.thruster_horizontal_radians), 1)

        # Publisher to publish the filtered position of the follower
        self.filtered_position_pub = self.create_publisher(Point, '/follower/filtered_position', 10)
        self.filtered_velocity_pub = self.create_publisher(Twist, '/follower/filtered_velocity', 1)

        # Timer for predict step of the filter
        self.timer = self.create_timer(self.dt, self.filter_predict)

        # Initialize the filter
        # State: [pos_x, pos_y, v, theta]
        # Measurement: [distance_to_leader1], or [distance_to_leader2], or [heading_change]
        points = MerweScaledSigmaPoints(n=4, alpha=0.01, beta=2., kappa=-1.0)
        self.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=1, dt=self.dt, points=points, 
            fx=self.state_transition_function, hx=None)      
        self.ukf.x = np.array([-7.5, 3., -0.5, 0.])  # Initial state assuming that the leaders move ahead first
        self.ukf.P = np.diag([10.0**2, 10.0**2, 1.0, 1.0])  # Initial covariance
        q_noise = Q_discrete_white_noise(dim=2, dt=self.dt, var=0.3**2)  # Noise for x and v; std should be how much v can change between timesteps
        Q = np.diag([0., 0., 0., 0.2**2])
        Q[0,0], Q[0,2], Q[2,0], Q[2,2] = q_noise[0,0], q_noise[0,1], q_noise[1,0], q_noise[1,1]
        self.ukf.Q = Q  # Process noise
        self.ukf.R = np.array([0.3**2]) # Measurement noise

    def filter_predict(self):   
        self.ukf.predict()
        # print(self.ukf.x)

    def state_transition_function(self, x: np.ndarray, dt: float):
        pos_x, pos_y, vel, theta = x
        pos_x += vel * np.cos(theta) * dt
        pos_y += vel * np.sin(theta) * dt
        theta += self.horizontal_thrust_vector * dt
        return np.array([pos_x, pos_y, vel, theta])

    def state_mean(self, sigmas, Wm):
        # Documentation: https://filterpy.readthedocs.io/en/latest/kalman/UnscentedKalmanFilter.html
        x = np.zeros(4)
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 3]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 3]), Wm))
        x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
        x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
        x[2] = np.sum(np.dot(sigmas[:, 2], Wm))
        x[3] = atan2(sum_sin, sum_cos)
        return x
    
    def leader1_distance_cb(self, msg: Float32):
        distance_to_leader1 = msg.data
        self.ukf.hx = partial(self.measurement_function, offset=np.array([0., 0.]))
        self.ukf.update(z=np.array([distance_to_leader1]))
        self.publish_point()

    def leader2_distance_cb(self, msg: Float32):
        distance_to_leader2 = msg.data
        self.ukf.hx = partial(self.measurement_function, offset=np.array([0., 10.]))
        self.ukf.update(z=np.array([distance_to_leader2]))
        self.publish_point()

    def measurement_function(self, x: np.ndarray, offset: np.ndarray):
        pos_x, pos_y, _, _ = x
        return np.sqrt([ (pos_x-offset[0]) ** 2 + (pos_y-offset[1]) ** 2 ])

    def publish_point(self):
        pos_x, pos_y, vel, theta = self.ukf.x
        point = Point()
        point.x = pos_x
        point.y = pos_y
        point.z = 0.0  # Assuming z is 0 for this example
        self.filtered_position_pub.publish(point)
        # self.get_logger().info(f'Publishing Point: x={point.x}, y={point.y}, z={point.z}')

        twist = Twist()
        twist.linear.x = vel
        twist.linear.y = theta
        self.filtered_velocity_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    position_filter = PositionFilter()
    rclpy.spin(position_filter)
    position_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()