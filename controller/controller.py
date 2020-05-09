import controller.CL_class as Cc
import controller.features_extr as fe
import numpy as np
import ros2connect.gazebo_connect as r2c
import cv2 as cv
import rclpy
import keyboard
import controller.motor as mr
import time
import controller.graph as gr
from os.path import expanduser

import controller.model as md


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
    control_type = 2
    #cl = Cc.ControlLaw(lmbda)
    cl = Cc.ControlLaw(lmbda, detector=cv.AKAZE_create())
    if len(cl.kp_des) > 0:
        rclpy.spin_once(subscriber)
        img_new = subscriber.img
        publisher.vel = cl.vel(img_new, control_type)
        publisher.pub_vel()
        #rclpy.spin_once(publisher)
        #while exit_cond(publisher.vel, accuracy):
        while cl.stop is False:
            rclpy.spin_once(subscriber)
            img_new = subscriber.img
            if img_new is None:
                print('No image')
                break
            publisher.vel = cl.vel(img_new, control_type)
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


def capture(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    subscriber = r2c.CameraSubscriber()
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
    #vel = np.zeros(shape=(len(publisher.vel), 1))
    vel = np.zeros(len(publisher.vel))
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


def effort_control(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    cam_sub = r2c.CameraSubscriber()
    clock_sub = r2c.ClockSubscriber()
    pos_sub = r2c.PoseSubscriber(robot_namespace)
    vel_sub = r2c.SpeedSubscriber(robot_namespace)
    vel_pub = r2c.SpeedPublisherEff(robot_namespace)

    rclpy.spin_once(clock_sub)
    clock_sub.prev_clock = clock_sub.clock
    z_e_prev = None
    z_v_prev = np.zeros(2)
    rclpy.spin_once(pos_sub)
    z_eta_prev = pos_sub.eta
    tau_prev = np.zeros(2)
    p_prev = np.zeros(2)
    control_type = 2
    #cl = Cc.ControlLaw(lmbda)
    cl = Cc.ControlLaw(detector=cv.AKAZE_create())
    if len(cl.kp_des) > 0:
        rclpy.spin_once(cam_sub)
        img_new = cam_sub.img
        rclpy.spin_once(pos_sub)

        v, tau_prev, z_eta_prev, z_e_prev, z_v_prev, p_prev = cl.eff(img_new, z_eta_prev, z_e_prev, z_v_prev, tau_prev,
                                                           clock_sub, vel_sub, pos_sub, p_prev, l_type=control_type)
        vel_pub.vel = v
        vel_pub.pub_vel()
        # time.sleep(0.1)
        while cl.stop is False:
            rclpy.spin_once(cam_sub)
            img_new = cam_sub.img
            if img_new is None:
                print('No image')
                break
            rclpy.spin_once(pos_sub)
            v, tau_prev, z_eta_prev, z_e_prev, z_v_prev, p_prev = cl.eff(img_new, z_eta_prev, z_e_prev, z_v_prev, tau_prev,
                                                                clock_sub, vel_sub, pos_sub, p_prev, l_type=control_type)
            vel_pub.vel = v
            vel_pub.pub_vel()
            # time.sleep(0.)
        if cl.error is False:
            print('End point, no errors')
        else:
            print('ERRORS')
    else:
        print("Bad init image. Desired key points vector is empty")
    stop(vel_pub)

    gr.save_matlab_n(['vel', 'm', 'e', 'z_e', 'z_v', 'eta', 'z_eta', 'p', 'K_e_z_e', 'L', 'time'],
                     [cl.vel_logger, cl.m_logger, cl.e_logger, cl.z_e_logger,
                      cl.z_v_logger, cl.eta_logger, cl.z_eta_logger, cl.p_logger,
                      cl.K_e_z_e_logger, cl.L_logger, cl.time_logger],
                     'akaze_eff_controller')
    cam_sub.destroy_node()
    clock_sub.destroy_node()
    pos_sub.destroy_node()
    vel_pub.destroy_node()
    vel_sub.destroy_node()
    rclpy.shutdown()


def simple_eff_con(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    cam_sub = r2c.CameraSubscriber()
    clock_sub = r2c.ClockSubscriber()
    odom = r2c.PoseSubscriber(robot_namespace)
    vel_pub = r2c.SpeedPublisherEff(robot_namespace)
    vel_sub = r2c.SpeedSubscriber(robot_namespace)
    # img = cv.imread(folder + img_filename)
    # if img is None:
    #    rclpy.spin_once(subscriber)
    #    img = subscriber.img
    fr = 0.0001  # coefficient of friction
    k_m = 0.001
    rclpy.spin_once(clock_sub)
    clock_sub.prev_clock = clock_sub.clock
    control_type = 2
    cl = Cc.ControlLaw(detector=cv.AKAZE_create())
    if len(cl.kp_des) > 0:
        rclpy.spin_once(cam_sub)
        rclpy.spin_once(vel_sub)
        img_new = cam_sub.img
        vel_pub.vel = cl.simp_eff(img_new, clock_sub, vel_sub, l_type=control_type)
        vel_pub.pub_vel()
        time.sleep(0.1)
        while cl.stop is False:
            rclpy.spin_once(cam_sub)
            rclpy.spin_once(vel_sub)
            img_new = cam_sub.img
            if img_new is None:
                print('No image')
                break
            vel_pub.vel = cl.simp_eff(img_new, clock_sub, vel_sub, l_type=control_type)
            print('vel_pub')
            print(vel_pub.vel[0])
            vel_pub.pub_vel()
            time.sleep(0.1)
        if cl.error is False:
            print('End point, no errors')
        else:
            print('ERRORS')
    else:
        print("Bad init image. Desired key points vector is empty")
    stop(vel_pub)
    gr.save_matlab_n(['vel', 'm', 'e'],
                     [cl.vel_logger, cl.m_logger, cl.e_logger],
                     'akaze_simple_eff_controller')
    cam_sub.destroy_node()
    clock_sub.destroy_node()
    odom.destroy_node()
    vel_sub.destroy_node()
    rclpy.shutdown()


def fixed_eff(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    clock_sub = r2c.ClockSubscriber()
    odom = r2c.PoseSubscriber(robot_namespace)
    vel_pub = r2c.SpeedPublisherEff(robot_namespace)
    vel_sub = r2c.SpeedSubscriber(robot_namespace)
    rclpy.spin_once(clock_sub)
    clock_sub.prev_clock = clock_sub.clock
    vel_logger = []
    time_logger = []
    pose_logger = []
    vel_input_logger = []
    prev_vel = np.zeros(2)
    print(clock_sub.clock)
    #time.sleep(0.1)
    i = 0
    # while clock_sub.clock < 30:
    while i < 100:
        rclpy.spin_once(vel_sub)
        rclpy.spin_once(odom)
        # vel_pub.vel = mr.motor([0.1 + 0.1 * int(i/10), 0.0], clock_sub, vel_sub)
        # vel_pub.vel = [0.1 + 0.1 * int(i/10), 0.0]
        vel_pub.vel = mr.motor_lsim(md.sys, [5.0, 0.0] , clock_sub, vel_sub, prev_vel)
        prev_vel = vel_pub.vel
        vel_input_logger.append(vel_pub.vel)
        vel_logger.append(vel_sub.vel2)
        time_logger.append(clock_sub.clock)
        pose_logger.append(odom.eta)
        print(vel_pub.vel)
        vel_pub.pub_vel()
        i += 1
    stop(vel_pub)
    gr.save_matlab_n(['vel_gazebo', 'pose', 'time', 'vel_input'],
                     [vel_logger, pose_logger, time_logger, vel_input_logger],
                     'fixed_eff_controller')
    clock_sub.destroy_node()
    odom.destroy_node()
    vel_sub.destroy_node()
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

if __name__ == '__simple_eff_con__':
    simple_eff_con()

if __name__ == '__fixed_eff__':
    fixed_eff()

# def effort_control(args=None):
#     rclpy.init(args=args)
#     robot_namespace = '/predator'
#     cam_sub = r2c.CameraSubscriber(robot_namespace)
#     sub_clock = r2c.ClockSubscriber()
#     #odom = r2c.PoseSubscriber(robot_namespace)
#     odom = r2c.PoseSubscriber(robot_namespace)
#     speed = r2c.SpeedSubscriber(robot_namespace)
#     sub_joint = r2c.JointSubscriber(robot_namespace)
#     eff_pub = r2c.EffortPublisher(robot_namespace=robot_namespace)
#     pose_clock_exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
#     pose_clock_exec.add_node(sub_clock)
#     pose_clock_exec.add_node(odom)
#     #img = cv.imread(folder + img_filename)
#     #if img is None:
#     #    rclpy.spin_once(subscriber)
#     #    img = subscriber.img
#     fr = 0.0001  # coefficient of friction
#     k_m = 0.001
#     pose_clock_exec.spin_once()
#     t_prev = sub_clock.clock
#     z_e_prev = None
#     z_mu_prev = np.zeros(2)
#     z_nu_prev = odom.nu
#     contr_type = 2
#     #cl = Cc.ControlLaw(lmbda)
#     cl = Cc.ControlLaw(detector=cv.ORB_create())
#     if len(cl.kp_des) > 0:
#         rclpy.spin_once(cam_sub)
#         pose_clock_exec.spin_once()
#         img_new = cam_sub.img
#         #print("clock=", sub_clock.clock)
#         #print("prev_clock=", t_prev)
#         dt = float(sub_clock.clock) - float(t_prev)
#         # eff_buff, z_nu_prev, z_e_prev, z_mu_prev = cl.eff(img_new, dt, odom.nu, z_nu_prev,
#         #                                                   z_e_prev,
#         #                                                   z_mu_prev, l_type=contr_type)
#         eff_buff = cl.simp_eff(img_new, odom.vel2)
#         t_prev = sub_clock.clock
#         rclpy.spin_once(sub_joint)
#         eff_pub.pub(eff_buff[0] - fr * sub_joint.joint_state.velocity[0],
#                     eff_buff[1] - fr * sub_joint.joint_state.velocity[1])
#         time.sleep(0.1)
#         while cl.stop is False:
#             rclpy.spin_once(cam_sub)
#             img_new = cam_sub.img
#             if img_new is None:
#                 print('No image')
#                 break
#             pose_clock_exec.spin_once()
#             #print("clock=", sub_clock.clock)
#             #print("prev_clock=", t_prev)
#             dt = float(sub_clock.clock) - float(t_prev)
#             #eff_buff = cl.simp_eff(img_new, odom.vel2)
#             if dt > 0:
#                 eff_buff, z_mu_prev, z_e_prev, z_mu_prev = cl.eff(img_new, dt, odom.nu, z_nu_prev,
#                                                                   z_e_prev,
#                                                                   z_mu_prev, l_type=contr_type)
#                 t_prev = sub_clock.clock
#             rclpy.spin_once(sub_joint)
#             eff_pub.pub(eff_buff[0] - fr * sub_joint.joint_state.velocity[0],
#                         eff_buff[1] - fr * sub_joint.joint_state.velocity[1])
#             time.sleep(0.1)
#         else:
#             rclpy.spin_once(speed)
#             while speed.vel.linear.x > 0.01 and speed.vel.angular.z > 0.01:
#                 rclpy.spin_once(speed)
#                 m_stop = -k_m * speed.vel.linear.x
#                 eff_pub.pub(m_stop, m_stop)
#         if cl.error is False:
#             print('End point, no errors')
#         else:
#             print('ERRORS')
#     else:
#         print("Bad init image. Desired key points vector is empty")
#     eff_pub.pub(0.0, 0.0)
#     cam_sub.destroy_node()
#     sub_clock.destroy_node()
#     odom.destroy_node()
#     pose_clock_exec.shutdown()
#     eff_pub.destroy_node()
#     sub_joint.destroy_node()
#     speed.destroy_node()
#     rclpy.shutdown()

# def simple_eff_con(args=None):
#     rclpy.init(args=args)
#     robot_namespace = '/predator'
#     cam_sub = r2c.CameraSubscriber(robot_namespace)
#     speed = r2c.SpeedSubscriber(robot_namespace)
#     sub_joint = r2c.JointSubscriber(robot_namespace)
#     eff_pub = r2c.EffortPublisher(robot_namespace=robot_namespace)
#     #img = cv.imread(folder + img_filename)
#     #if img is None:
#     #    rclpy.spin_once(subscriber)
#     #    img = subscriber.img
#     fr = 0.0001  # coefficient of friction
#     k_m = 0.5
#     contr_type = 2
#     #cl = Cc.ControlLaw(lmbda)
#     cl = Cc.ControlLaw(detector=cv.ORB_create())
#     if len(cl.kp_des) > 0:
#         rclpy.spin_once(cam_sub)
#         img_new = cam_sub.img
#         eff_buff = cl.simp_eff(img_new, speed.vel2, l_type=contr_type)
#         rclpy.spin_once(sub_joint)
#         eff_pub.pub(eff_buff[0] - fr * sub_joint.joint_state.velocity[0],
#                     eff_buff[1] - fr * sub_joint.joint_state.velocity[1])
#         time.sleep(0.1)
#         while cl.stop is False:
#             rclpy.spin_once(cam_sub)
#             img_new = cam_sub.img
#             if img_new is None:
#                 print('No image')
#                 break
#             rclpy.spin_once(speed)
#             eff_buff = cl.simp_eff(img_new, speed.vel2, l_type=contr_type)
#             rclpy.spin_once(sub_joint)
#             eff_pub.pub(eff_buff[0] - fr * sub_joint.joint_state.velocity[0],
#                         eff_buff[1] - fr * sub_joint.joint_state.velocity[1])
#             #time.sleep(0.1)
#         else:
#             rclpy.spin_once(sub_joint)
#             eff_pub.pub(- fr * sub_joint.joint_state.velocity[0],
#                         - fr * sub_joint.joint_state.velocity[1])
#             while abs(speed.vel2[0]) > 0.01 or abs(speed.vel2[1]) > 0.01:
#                 rclpy.spin_once(speed)
#                 rclpy.spin_once(sub_joint)
#                 m_stop = -k_m * speed.vel.linear.x
#                 #print(-sub_joint.joint_state.effort[0], -sub_joint.joint_state.effort[1])
#                 eff_pub.pub(m_stop- fr * sub_joint.joint_state.velocity[0], m_stop- fr * sub_joint.joint_state.velocity[0])
#         if cl.error is False:
#             print('End point, no errors')
#         else:
#             print('ERRORS')
#     else:
#         print("Bad init image. Desired key points vector is empty")
#     eff_pub.pub(0.0, 0.0)
#     cam_sub.destroy_node()
#     eff_pub.destroy_node()
#     sub_joint.destroy_node()
#     speed.destroy_node()
#     rclpy.shutdown()