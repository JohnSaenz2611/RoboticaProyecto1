#!/usr/bin/env python3

import os
from math import degrees
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from file_reader import File_reader
from A_star_class import A_star
from A_star_relocate import A_star_relocate
from tf.transformations import euler_from_quaternion

NUMERO_ESCENA = 1
PATH = os.path.dirname(os.path.abspath(__file__))
path_file = open(PATH + f'/paths/Path_list_{NUMERO_ESCENA}.txt')
PATH_LIST = path_file.read().split('\n')
PATH_LIST.pop()
path_file.close()
X = 1
Y = -1
path_list = A_star(4)
scene = File_reader(PATH + f'/scenes/Escena-Problema{NUMERO_ESCENA}.txt')

# Posicion Final
posFinal_x_2 = scene.qLoc_x
posFinal_y_2 = scene.qLoc_y
# Posicion Inicial
posInicial_x_2 = 0 if scene.qf_x == 0 else (scene.qf_x - 0.25) * 2
posInicial_y_2 = 0 if scene.qf_y == 0 else (scene.qf_y - 0.25) * 2

path_file_2 = open(PATH + f'/paths/Path_list_{NUMERO_ESCENA}_ReLoc.txt')
PATH_LIST_2 = path_file_2.read().split('\n')
PATH_LIST_2.pop()
path_file_2.close()
TRANSFORMED_PATH_LIST_2 = []
for posicion in PATH_LIST_2:
    pos_x, pos_y = map(int, posicion.split(','))
    pos_x -= posInicial_x_2
    pos_y -= posInicial_y_2
    pos = f'{pos_x},{pos_y}'
    TRANSFORMED_PATH_LIST_2.append(pos)

MAX_SPEED = 0.5
# Posicion Final
posFinal_x, posFinal_y = [scene.qf_x, scene.qf_y]
# Posicion Inicial
posInicial_x = 0 if scene.q0_x == 0 else (scene.q0_x - 0.25) * 2
posInicial_y = 0 if scene.q0_y == 0 else (scene.q0_y - 0.25) * 2

TRANSFORMED_PATH_LIST = []
for posicion in PATH_LIST:
    pos_x, pos_y = map(int, posicion.split(','))
    pos_x -= posInicial_x
    pos_y -= posInicial_y
    pos = f'{pos_x},{pos_y}'
    TRANSFORMED_PATH_LIST.append(pos)

posInicial_x = 0
posInicial_y = 0
aux = TRANSFORMED_PATH_LIST[-1]
aux = aux.split(',')
posFinal_x = 0 if float(aux[0]) == 0 else float(aux[0]) / 2 + 0.25
posFinal_y = 0 if float(aux[1]) == 0 else float(aux[1]) / 2 + 0.25

class Main(object):
    def __init__(self, path_list):
        self.path_list = path_list
        self.max_speed_multiplier = 1
        self.position_x = 0.75  # Posicion actual del movil / de los sensores
        self.position_y = 0.75
        self.orientation = 0  # Orientacion actual del movil / de los sensores
        self.cmd_vel = Twist()
        self.real_pose = Pose2D()
        self.rate = rospy.Rate(1/0.01)
        self.R = 0.0195  # m
        self.L = 0.0381  # m
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.distance_publisher = rospy.Publisher(
            '/goal_distance', Float32, queue_size=1)
        self.right_motor_speed_publisher = rospy.Publisher(
            '/rightMotorSpeed', Float32, queue_size=1)
        self.left_motor_speed_publisher = rospy.Publisher(
            '/leftMotorSpeed', Float32, queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher(
            '/P3DX_RosAria/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/P3DX_RosAria/2D_pose', Pose2D,
                          self.pose_cb, queue_size=1)
        rospy.Subscriber('/p3dxPosition', Float32MultiArray,
                         callback=self.p3dxPosition_cb, queue_size=1)
        rospy.Subscriber('/PioneerOrientation', Float32,
                         callback=self.PioneerOrientation_cb, queue_size=1)
        self.angular_speed_publisher = rospy.Publisher(
            '/angularSpeed', Float32, queue_size=1)
        rospy.on_shutdown(self.on_shutdown_cb)

        rospy.Subscriber("/RosAria/pose", Odometry, callback=self.p3dxAngular_cb, queue_size=1)

        self.follow_path()

    def p3dxAngular_cb(self, msg: Odometry):
        # self.position_x = msg.pose.pose.position.x
        # self.position_y = msg.pose.pose.position.y
        angle_x = msg.pose.pose.orientation.x
        angle_y = msg.pose.pose.orientation.y
        angle_z = msg.pose.pose.orientation.z
        angle_w = msg.pose.pose.orientation.w
        new_angle = euler_from_quaternion([angle_x, angle_y, angle_z, angle_w])
        new_angle = degrees(new_angle[2])
        self.orientation = new_angle
        self.angular_speed_publisher.publish(new_angle)

    def pose_cb(self, msg: Pose2D):
        self.position_x = msg.x
        self.position_y = msg.y
    #     self.orientation = degrees(msg.theta)
    #     self.angular_speed_publisher.publish(self.orientation)

    def p3dxPosition_cb(self, msg: Float32MultiArray):
        self.position_x = msg.data[0]
        self.position_y = msg.data[1]

    def PioneerOrientation_cb(self, msg: Float32):
        self.orientation = msg.data  # grados

    def follow_path(self):
        old_goal_x = posInicial_x
        old_goal_y = posInicial_y
        for pair in self.path_list:
            rospy.loginfo(f'Paso X, siguiente punto: {pair}')
            goal_x, goal_y = map(float, pair.split(','))
            goal_x = goal_x / 2
            goal_y = goal_y / 2

            # Esto si se tiene que mover hacia la derecha
            #print(f'error: {old_goal_x}')
            if goal_x > old_goal_x:
                sentido = self.get_rotation(self.orientation, 0)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(X, 0.5)

            # Esto si se tiene que mover hacia la izquierda
            # TODO: Tener en cuenta el cambio de signo en el angulo cuando es 180
            elif goal_x < old_goal_x:
                sentido = self.get_rotation(self.orientation, 180)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(X, -0.5)
                self.publish_velocities(0, 0)

            # Esto si se tiene que mover hacia arriba
            elif goal_y > old_goal_y:
                sentido = self.get_rotation(self.orientation, 90)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(Y, 0.5)
                self.publish_velocities(0, 0)

            # Esto si se tiene que mover hacia abajo
            elif goal_y < old_goal_y:
                sentido = self.get_rotation(self.orientation, -90)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(Y, -0.5)

            old_goal_x = goal_x
            old_goal_y = goal_y

        # Mirar hacia el segundo punto
        sentido = self.get_rotation(self.orientation, 90)
        while sentido != 0 and not rospy.is_shutdown():
            self.rotate_90(sentido)
            sentido = self.get_rotation(self.orientation, 90)

        print(f'''
        Posicion actual x: {self.position_x}
        Posicion actual y: {self.position_y}
        Orientacion actual: {self.orientation}
        ''')

    def get_rotation(self, start_angle: float, final_angle: float):
        if start_angle - 5 > final_angle:
            return 1  # Derecha
        elif start_angle + 5 < final_angle:
            return -1  # Izquierda
        else:
            return 0  # Nada

    def move_forward(self, direction, distancia):
        self.max_speed_multiplier = 1
        rospy.loginfo('Moving forward')
        old_position_value_x = self.position_x
        old_position_value_y = self.position_y
        keep_moving_forward = True
        while keep_moving_forward and not rospy.is_shutdown():
            if direction == Y:
                #print(f"Posicion en Y: {self.position_y}")
                #print(f"Posicion inicial en Y: {old_position_value_y + distancia}")
                
                # Cambiar el 0.1 si el robot no lee tan rapido
                if self.position_y >= old_position_value_y + distancia - 0.1:
                    self.max_speed_multiplier = 0.1

                if (self.position_y >= old_position_value_y + distancia and (self.orientation > 88 and self.orientation < 92)) or (self.position_y <= old_position_value_y + distancia and (self.orientation < -88 and self.orientation > -92)):
                    keep_moving_forward = False
                    old_position_value_y = self.position_y

                self.publish_velocities(MAX_SPEED, MAX_SPEED)
                rospy.Rate.sleep(self.rate)

            elif direction == X:
                #print(f"Posicion en X: {self.position_x}")
                #print(f"Posicion inicial en X: {old_position_value_x + distancia}")

                if self.position_x >= old_position_value_x + distancia - 0.1:
                        self.max_speed_multiplier = 0.1

                if ((self.position_x >= old_position_value_x + distancia) and (self.orientation > - 5 and self.orientation < 5)) or ((self.position_x <= old_position_value_x + distancia) and (((self.orientation < -175 and self.orientation > -185)) or (self.orientation > 175 and self.orientation < 185))):
                    keep_moving_forward = False
                    old_position_value_x = self.position_x

                self.publish_velocities(MAX_SPEED, MAX_SPEED)
                rospy.Rate.sleep(self.rate)

        for i in range(10):
            self.left_motor_speed_publisher.publish(0)
            self.right_motor_speed_publisher.publish(0)
            continue

    def rotate_90(self, side):
        rospy.loginfo('Rotating 90 degrees')
        old_orientation_value = self.orientation
        keep_rotating = True
        self.max_speed_multiplier = 1
        while keep_rotating and not rospy.is_shutdown():
            
            if abs(old_orientation_value - self.orientation) > 90 - 5:
                # Cambiar el 0.01 si gira muy lento
                self.max_speed_multiplier = 0.01
            keep_rotating = abs(old_orientation_value - self.orientation) < 90
            rospy.logdebug(keep_rotating)
            rospy.logdebug(
                f'rotating {abs(old_orientation_value - self.orientation)} keep rotating: {keep_rotating}')
            self.publish_velocities(side * -0.1, side * 0.1)

        for i in range(10):
            self.left_motor_speed_publisher.publish(0)
            self.right_motor_speed_publisher.publish(0)

    def publish_velocities(self, rigth_speed, left_speed):
        self.cmd_vel.linear.x = ((rigth_speed + left_speed) / 2) * self.max_speed_multiplier
        self.cmd_vel.angular.z = ((rigth_speed - left_speed) / self.L) * self.max_speed_multiplier

        self.right_motor_speed_publisher.publish(rigth_speed * self.max_speed_multiplier)
        self.left_motor_speed_publisher.publish(left_speed * self.max_speed_multiplier)
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def on_shutdown_cb(self):
        self.left_motor_speed_publisher.publish(0)
        self.right_motor_speed_publisher.publish(0)
        rospy.loginfo('Closing node')


if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous=True, log_level=rospy.INFO)
    Main(TRANSFORMED_PATH_LIST)
    Main(TRANSFORMED_PATH_LIST_2)
