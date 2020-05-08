import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

import numpy as np
import geometry_msgs.msg as gm
import gazebo_msgs.msg as gz
import sensor_msgs.msg as sm
from cv_bridge import CvBridge


class SpeedPublisher(Node):

    def __init__(self, robot_namespace='', vel=np.zeros(shape=(6, 1)), cam_coord=False):
        super().__init__('speed_publisher', namespace=robot_namespace)
        self.publisher_ = self.create_publisher(gm.Twist, 'cmd_vel', 10)
        self.vel = vel
        self.cam_coord = cam_coord

    def pub_x(self, x):
        __msg = gm.ModelState()
        __msg.twist.linear.x = x
        self.publisher_.publish(__msg)

    def pub_vel_cam_coord(self):
        #  cam   robot
        #   z      x
        #  -x      y
        #  -y      z
        __msg = gm.Twist()
        if len(self.vel) == 6:
            __msg.linear.x = self.vel[2][0]
            __msg.linear.y = -self.vel[0][0]
            __msg.linear.z = -self.vel[1][0]
            __msg.angular.x = self.vel[5][0]
            __msg.angular.y = -self.vel[3][0]
            __msg.angular.z = -self.vel[4][0]
        if len(self.vel) == 2:
            __msg.linear.x = -self.vel[0][0]
            __msg.linear.y = 0.
            __msg.linear.z = 0.
            __msg.angular.x = 0.
            __msg.angular.y = 0.
            __msg.angular.z = 10 * self.vel[1][0]
        self.publisher_.publish(__msg)

    def pub_vel_robot_coord(self):
        __msg = gm.Twist()
        if len(self.vel) == 6:
            __msg.linear.x = self.vel[0][0]
            __msg.linear.y = self.vel[1][0]
            __msg.linear.z = self.vel[2][0]
            __msg.angular.x = self.vel[3][0]
            __msg.angular.y = self.vel[4][0]
            __msg.angular.z = self.vel[5][0]
        if len(self.vel) == 2:
            __msg.linear.x = self.vel[0][0]
            __msg.linear.y = 0.
            __msg.linear.z = 0.
            __msg.angular.x = 0.
            __msg.angular.y = 0.
            __msg.angular.z = -self.vel[1][0]
        self.publisher_.publish(__msg)

    def pub_vel(self):
        if self.cam_coord is True:
            self.pub_vel_cam_coord()
        else:
            self.pub_vel_robot_coord()


class ClockSubscriber(Node):

    def __init__(self):
        super().__init__('clock_subscriber')
        self.subscription = self.create_subscription(
            Clock,
            'clock',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.clock = 0.0
        self.prev_clock = 0.0

    def listener_callback(self, msg):
        self.clock = msg.clock.sec + (float(msg.clock.nanosec) / (pow(10, 9)))
       # print(self.clock)


class GazeboSpeedSubscriber(Node):

    def __init__(self, robot_namespace=''):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            gz.ModelStates, '/gazebo/model_states',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.linear_x = 0.
        self.angular_w = 0.

    def listener_callback(self, msg):
        self.linear_x = msg.twist[1].linear.x
        self.angular_w = msg.twist[1].angular.z


class CameraSubscriber(Node):

    def __init__(self, robot_namespace='', cam_ns='/camera1'):
        super().__init__('cam_subscriber')
        self.subscription = self.create_subscription(
            sm.Image,
            robot_namespace + cam_ns + '/image_raw',
            self.listener_callback, 10)
        self.img = np.zeros((480, 640, 1), np.uint8)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        cvb = CvBridge()
        self.img = cvb.imgmsg_to_cv2(msg)


class RGBCameraSubscriber(Node):

    def __init__(self, robot_namespace='', cam_ns='/camera'):
        super().__init__('cam_subscriber')
        self.subscription = self.create_subscription(
            sm.Image,
            robot_namespace + cam_ns + '/rgb/image_raw',
            self.listener_callback, 10)
        self.img = np.zeros((480, 640, 1), np.uint8)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        cvb = CvBridge()
        self.img = cvb.imgmsg_to_cv2(msg)


class TestPub(Node):

    def __init__(self):
        super().__init__('test_publisher', namespace='/trolol')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_msg)

    def pub_msg(self):
        msg = String()
        msg.data = "OMGOMG"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


class EffortPublisher(Node):

    def __init__(self, robot_namespace=''):
        super().__init__('effort_publisher', namespace=robot_namespace)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'effort_group_controller/command', 10)
        self.pub(0, 0)

    def pub(self, left, right, max_left=0.05, max_right=0.05):
        __msg = Float64MultiArray()
        #array = np.array([0.005, 0.005], dtype=np.float64)
        if abs(left) > max_left:
            left = max_left * np.sign(left)
        if abs(right) > max_right:
            right = max_right * np.sign(right)
        __msg.data = [np.float64(left), np.float64(right)]
        #__msg.data = array
        self.publisher_.publish(__msg)


class SpeedSubscriber(Node):

    def __init__(self, robot_namespace=''):
        super().__init__('speed_subscriber', namespace=robot_namespace)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.twist = Odometry().twist.twist
        self.vel2 = np.zeros(2)

    def listener_callback(self, msg):
        self.twist = msg.twist.twist
        self.vel_x = np.sqrt(np.power(self.twist.linear.x, 2) + np.power(self.twist.linear.y, 2))
        self.vel_w = self.twist.angular.z
        self.vel2 = np.array([self.vel_x, self.vel_w])


class PoseSubscriber(Node):

    def __init__(self, robot_namespace=''):
        super().__init__('pose_subscriber', namespace=robot_namespace)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.pose = Odometry().pose.pose
        self.euler = np.zeros(3)
        self.eta = np.zeros(3)

    def listener_callback(self, msg):
        self.pose = msg.pose.pose
        buff = R.from_quat([self.pose.orientation.x, self.pose.orientation.y,
                            self.pose.orientation.z, self.pose.orientation.w])
        self.euler = buff.as_euler('zyx')
        self.eta = np.asarray([self.pose.position.x, self.pose.position.y, self.euler[0]])
        #print(self.nu)


class JointSubscriber(Node):

    def __init__(self, robot_namespace=''):
        super().__init__('joint_subscriber', namespace=robot_namespace)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.joint_state = JointState()

    def listener_callback(self, msg):
        self.joint_state = msg


class WhSpeedPublisher(Node):

    def __init__(self, robot_namespace=''):
        super().__init__('velocity_publisher', namespace=robot_namespace)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'velocity_group_controller/command', 10)
        self.pub(0, 0)

    def pub(self, left, right, max_left=10, max_right=10):
        __msg = Float64MultiArray()
        #array = np.array([0.005, 0.005], dtype=np.float64)
        if abs(left) > max_left:
            left = max_left * np.sign(left)
        if abs(right) > max_right:
            right = max_right * np.sign(right)
        __msg.data = [np.float64(left), np.float64(right)]
        #__msg.data = array
        self.publisher_.publish(__msg)


class Ros2ClockSubscriber(Node):

    def __init__(self):
        super().__init__('clock_subscriber')
        self.subscription = self.create_subscription(
            Clock,
            'clock',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.clock = 0.0
        self.prev_clock = 0.0

    def listener_callback(self, msg):
        self.clock = msg.clock.sec + (float(msg.clock.nanosec) / (pow(10, 9)))
       # print(self.clock)


class SpeedPublisherEff(Node):
    def __init__(self, robot_namespace='', vel=np.zeros(shape=(6, 1)), cam_coord=False):
        super().__init__('speed_publisher', namespace=robot_namespace)
        self.publisher_ = self.create_publisher(gm.Twist, 'cmd_vel', 10)
        self.vel = vel
        self.cam_coord = cam_coord

    def pub_x(self, x):
        __msg = gm.ModelState()
        __msg.twist.linear.x = x
        self.publisher_.publish(__msg)

    def pub_vel_cam_coord(self):
        #  cam   robot
        #   z      x
        #  -x      y
        #  -y      z
        __msg = gm.Twist()
        if len(self.vel) == 6:
            __msg.linear.x = self.vel[2][0]
            __msg.linear.y = -self.vel[0][0]
            __msg.linear.z = -self.vel[1][0]
            __msg.angular.x = self.vel[5][0]
            __msg.angular.y = -self.vel[3][0]
            __msg.angular.z = -self.vel[4][0]
        if len(self.vel) == 2:
            __msg.linear.x = -self.vel[0]
            __msg.linear.y = 0.
            __msg.linear.z = 0.
            __msg.angular.x = 0.
            __msg.angular.y = 0.
            __msg.angular.z = self.vel[1]
        self.publisher_.publish(__msg)

    def pub_vel_robot_coord(self):
        __msg = gm.Twist()
        if len(self.vel) == 6:
            __msg.linear.x = self.vel[0][0]
            __msg.linear.y = self.vel[1][0]
            __msg.linear.z = self.vel[2][0]
            __msg.angular.x = self.vel[3][0]
            __msg.angular.y = self.vel[4][0]
            __msg.angular.z = self.vel[5][0]
        if len(self.vel) == 2:
            __msg.linear.x = self.vel[0]
            __msg.linear.y = 0.
            __msg.linear.z = 0.
            __msg.angular.x = 0.
            __msg.angular.y = 0.
            __msg.angular.z = -self.vel[1]
        self.publisher_.publish(__msg)

    def pub_vel(self):
            if self.cam_coord is True:
                self.pub_vel_cam_coord()
            else:
                self.pub_vel_robot_coord()

