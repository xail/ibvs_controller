import numpy as np
import ros2connect.gazebo_connect as gc
import rclpy


def test(args=None):
    rclpy.init(args=args)
    sub = gc.SpeedSubscriber()
    rclpy.spin_once(sub)
    rclpy.spin_once(sub)
    print('v_x = ', sub.linear_x)
    print('w_z =', sub.angular_w)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__test__':
    test()
