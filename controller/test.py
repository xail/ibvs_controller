import numpy as np
import ros2connect.gazebo_connect as gc
import rclpy
import matplotlib.pyplot as plt
from os.path import expanduser

home = expanduser("~")

folder = home + '/resources/'


def test(args=None):
    rclpy.init(args=args)
    sub = gc.ClockSubscriber()
    rclpy.spin_once(sub)
    # v = []
    # for i in range(0, 10):
    #     rclpy.spin_once(sub)
    #     v.append(sub.linear_x)
    # print('v_x = ', sub.linear_x)
    # print('w_z =', sub.angular_w)
    # print(v)
    # plotVel_2DOF_v(v)
    sub.destroy_node()
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