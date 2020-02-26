import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
import geometry_msgs.msg as gm
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



class SpeedSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            gm.ModelState,
            '/twist',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg.twist.linear.x))


class CameraSubscriber(Node):

    def __init__(self, robot_namespace='', cam_ns='/camera1'):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            sm.Image,
            robot_namespace + cam_ns + '/image_raw',
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
