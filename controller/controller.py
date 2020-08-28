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

# main function that make robot chaise target
def main(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    subscriber = r2c.CameraSubscriber()
    publisher = r2c.SpeedPublisher(robot_namespace, cam_coord=True)
    
    Plot = False
    lmbda = 3
    accuracy = 0.01

    cl = Cc.ControlLaw(lmbda, detector=cv.AKAZE_create(), velGraph=Plot)
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
        if cl.error is False:
            print('End point, no errors')
    else:
        print("Bad init image. Desired key points vector is empty")
    stop(publisher)
    if Plot is True:
        cl.plotVel_2DOF_v()
        cl.plotVel_2DOF_omega()
    subscriber.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()

# taking picture of target and save it
def capture(args=None):
    rclpy.init(args=args)

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

# start target movement on x axis with speed 1
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

# predator's stop function
def stop(publisher):
    vel = np.zeros(shape=(len(publisher.vel), 1))
    publisher.vel = vel
    publisher.pub_vel()

# start torsion of the predator around the z axis with speed 1 
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

# chase target untill press q
def chase_key(args=None):
    rclpy.init(args=args)
    robot_namespace = '/predator'
    subscriber = r2c.CameraSubscriber()
    publisher = r2c.SpeedPublisher(robot_namespace, cam_coord=True)

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
