import controller.CL_class as Cc
import controller.features_extr as fe
import numpy as np
import ros2connect.gazebo_connect as r2c
import cv2 as cv
import rclpy
import keyboard
from os.path import expanduser

home = expanduser("~")
img_filename = 'Obj.png'
folder = home + '/resources/'


def main(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    subscriber = r2c.CameraSubscriber()
    publisher = r2c.SpeedPublisher(robot_namespace, cam_coord=True)

    #img = cv.imread(folder + img_filename)
    #if img is None:
    #    rclpy.spin_once(subscriber)
    #    img = subscriber.img
    lmbda = 3
    accuracy = 0.01

    #cl = Cc.ControlLaw(lmbda)
    cl = Cc.ControlLaw(lmbda, detector=cv.AKAZE_create())
    if len(cl.kp_des) > 0:
        rclpy.spin_once(subscriber)
        img_new = subscriber.img
        publisher.vel = cl.vel(img_new)
        publisher.pub_vel()
        #rclpy.spin_once(publisher)
        #while exit_cond(publisher.vel, accuracy):
        while cl.stop is False:
            rclpy.spin_once(subscriber)
            img_new = subscriber.img
            if img_new is None:
                print('No image')
                break
            publisher.vel = cl.vel(img_new)
            publisher.pub_vel()
            #rclpy.spin_once(publisher)
        #print(publisher.vel)
        if cl.error is False:
            print('End point, no errors')
    else:
        print("Bad init image. Desired key points vector is empty")
    stop(publisher)
    subscriber.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()


def effort_control(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    cam_sub = r2c.CameraSubscriber(robot_namespace)
    sub_clock = r2c.ClockSubscriber()
    odom = r2c.PoseSubscriber(robot_namespace)
    sub_joint = r2c.JointSubscriber(robot_namespace)
    eff_pub = r2c.EffortPublisher(robot_namespace=robot_namespace)
    pose_clock_exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    pose_clock_exec.add_node(sub_clock)
    pose_clock_exec.add_node(odom)
    #img = cv.imread(folder + img_filename)
    #if img is None:
    #    rclpy.spin_once(subscriber)
    #    img = subscriber.img
    fr = 0.001  # coefficient of friction
    t_prev = 0
    z_e_prev = None
    z_mu_prev = np.zeros(2)
    z_nu_prev = np.zeros(3)
    accuracy = 0.01

    #cl = Cc.ControlLaw(lmbda)
    cl = Cc.ControlLaw(detector=cv.ORB_create())
    if len(cl.kp_des) > 0:
        rclpy.spin_once(cam_sub)
        pose_clock_exec.spin_once()
        img_new = cam_sub.img
        eff_buff, z_nu_prev, z_e_prev, z_mu_prev = cl.eff(img_new, sub_clock.clock - t_prev, odom.nu, z_nu_prev,
                                                          z_e_prev,
                                                          z_mu_prev)
        t_prev = sub_clock.clock
        rclpy.spin_once(sub_joint)
        eff_pub.pub(eff_buff[0] - fr * sub_joint.joint_state.velocity[0],
                    eff_buff[1] - fr * sub_joint.joint_state.velocity[0])

        while cl.stop is False:
            rclpy.spin_once(cam_sub)
            img_new = cam_sub.img
            if img_new is None:
                print('No image')
                break
            pose_clock_exec.spin_once()
            eff_buff, z_mu_prev, z_e_prev, z_mu_prev = cl.eff(img_new, sub_clock.clock - t_prev, odom.nu, z_nu_prev,
                                                              z_e_prev,
                                                              z_mu_prev)
            t_prev = sub_clock.clock
            rclpy.spin_once(sub_joint)
            eff_pub.pub(eff_buff[0] - fr * sub_joint.joint_state.velocity[0],
                        eff_buff[1] - fr * sub_joint.joint_state.velocity[0])
        if cl.error is False:
            print('End point, no errors')
        else:
            print('ERRORS')
    else:
        print("Bad init image. Desired key points vector is empty")
    eff_pub.pub(0.0, 0.0)
    cam_sub.destroy_node()
    sub_clock.destroy_node()
    odom.destroy_node()
    pose_clock_exec.shutdown()
    eff_pub.destroy_node()
    sub_joint.destroy_node()
    rclpy.shutdown()


def capture(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    subscriber = r2c.CameraSubscriber(robot_namespace)
    rclpy.spin_once(subscriber)
    img = subscriber.img
    while img is None:
        rclpy.spin_once(subscriber)
        img = subscriber.img
    cv.imwrite(folder + img_filename, img)

    subscriber.destroy_node()
    rclpy.shutdown()


def exit_cond(vel, accuracy):
    if len(vel) == 6:
        return (abs(vel[0][0]) > accuracy) or (abs(vel[1][0]) > accuracy)\
                or (abs(vel[2][0]) > accuracy) or (abs(vel[3][0]) > accuracy)\
                or (abs(vel[4][0]) > accuracy) or (abs(vel[5][0]) > accuracy)
    if len(vel) == 2:
        return (abs(vel[0][0]) > accuracy) or (abs(vel[1][0]) > accuracy)
    else:
        print('Unsupported number DOF')
        return False


def move_target(args=None):
    rclpy.init(args=args)
    robot_namespace = '/target'
    publisher = r2c.SpeedPublisher(robot_namespace)
    vel = [[1.],
           [0.],
           [0.],
           [0.],
           [0.],
           [0.]]
    publisher.vel = vel
    rclpy.spin_once(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


def stop(publisher):
    vel = np.zeros(shape=(len(publisher.vel), 1))
    publisher.vel = vel
    publisher.pub_vel()


def move_predator(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator2'
    vel = [[0.],
           [0.],
           [0.],
           [0.],
           [0.],
           [1.]]
    publisher = r2c.SpeedPublisher(robot_namespace, vel)
    publisher.pub_vel()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


def chase_key(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    subscriber = r2c.CameraSubscriber()
    publisher = r2c.SpeedPublisher(robot_namespace, cam_coord=True)

    #img = cv.imread(folder + img_filename)
    #if img is None:
    #    rclpy.spin_once(subscriber)
    #    img = subscriber.img

    lmbda = 1
    accuracy = 0.001

    #cl = Cc.ControlLaw(lmbda)
    cl = Cc.ControlLaw(lmbda, detector=cv.ORB_create())
    if len(cl.kp_des) > 0:
        rclpy.spin_once(subscriber)
        img_new = subscriber.img
        publisher.vel = cl.vel(img_new)
        publisher.pub_vel()
        #rclpy.spin_once(publisher)
        while True:
            rclpy.spin_once(subscriber)
            img_new = subscriber.img
            if img_new is None:
                print('No image')
                break
            publisher.vel = cl.vel(img_new)
            publisher.pub_vel()
            try:
                if keyboard.is_pressed('q'):
                    break
            except:
                None
            #rclpy.spin_once(publisher)
        print(publisher.vel)
        print('End point, no errors')
    else:
        print("Bad init image. Desired key points vector is empty")
    stop(publisher)
    subscriber.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

if __name__ == '__capture__':
    capture()

if __name__ == '__move_predator__':
    move_predator()

if __name__ == '__chase_key__':
    chase_key()

if __name__ == '__effort_control__':
    effort_control()


