import numpy as np
import ros2connect.gazebo_connect as gc
import rclpy
import matplotlib.pyplot as plt
import time
from os.path import expanduser

home = expanduser("~")

folder = home + '/resources/'


def test(args=None):
    robot_namespace = '/turtlebot'
    rclpy.init(args=args)
    sub_clock = gc.ClockSubscriber()
    sub_vel = gc.SpeedSubscriber(robot_namespace)
    eff_pub = gc.EffortPublisher(robot_namespace=robot_namespace)
    vel_clock_exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    vel_clock_exec.add_node(sub_clock)
    vel_clock_exec.add_node(sub_vel)
    vel_clock_exec.spin_once()
    left = 0.0
    right = 0.0
    step = 0.0001
    time.sleep(1)
    vel_clock_exec.spin_once()
    eff_pub.pub(left, right)
    print(sub_vel.vel.angular)
    while sub_vel.vel.linear.x < 2:
        if sub_vel.vel.angular.z > 0:
            right += step
        else:
            if sub_vel.vel.angular.z < 0:
                left += step
            else:
                right += step
                left += step
        eff_pub.pub(left, right)
        time.sleep(0.5)
        vel_clock_exec.spin_once()
    print('left =', left)
    print('right =', right)
    eff_pub.pub(0.0, 0.0)
    sub_clock.destroy_node()
    sub_vel.destroy_node()
    vel_clock_exec.shutdown()
    eff_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__test__':
    test()


def plotVel_2DOF_v(v):
    if len(v) > 0:
        t = range(0, len(v))
        # print(self.y[0])
        plt.plot(t, v, color='black')
        plt.gca().set_xlabel('t, с')
        plt.gca().set_ylabel('v, м/с')
        plt.gca().set_xlim(left=0, right=len(v))
        plt.gca().set_ylim(bottom=min(v), top=max(v))
        # plt.gca().lines[0].set_xdata(t)
        # plt.gca().lines[0].set_ydata(self.y[0])
        # plt.gca().relim()
        # plt.gca().autoscale_view()
        plt.savefig(folder + 'test.eps')
        #plt.savefig(folder + 'v.pdf')
        plt.close()


def plotVel_2DOF_omega(self):
    if self.velGraph is True:
        if len(self.y[1]) > 0:
            t = range(0, len(self.y[1]))
            plt.plot(t, self.y[1], color='black')
            plt.gca().set_xlabel('t, с')
            plt.gca().set_ylabel('omega, рад/с')
            plt.gca().set_xlim(left=0, right=len(self.y[1]))
            plt.gca().set_ylim(bottom=min(self.y[0]), top=max(self.y[1]))
            plt.savefig(folder + 'omega.pdf')
            plt.close()