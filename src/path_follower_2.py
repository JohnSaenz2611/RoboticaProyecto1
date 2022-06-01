#!/usr/bin/env python3

import os
from math import degrees
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist, Pose2D
from file_reader import File_reader

NUMERO_ESCENA = 1
PATH = os.path.dirname(os.path.abspath(__file__))
path_file = open(PATH + f'/paths/Path_list_{NUMERO_ESCENA}.txt')
PATH_LIST = path_file.read().split('\n')
PATH_LIST.pop()
path_file.close()
X = 1
Y = -1

# if NUMERO_ESCENA == 4:
#     obstacle_file_2 = open(PATH + f'/obstacles/Obstacles_42.txt')
#     obstacle_list_2 = obstacle_file_2.read().split()
#     #Posicion Final
#     posFinal_2 = obstacle_list_2.pop()
#     posFinal_x_2, posFinal_y = map(float, posFinal_2.split(','))
#     #Posicion Inicial
#     posInicial_2 = obstacle_list_2.pop()
#     posInicial_x_2, posInicial_y_2 = map(float, posInicial_2.split(','))

#     path_file_2 = open(PATH + f'/paths/Path_list_42.txt')
#     PATH_LIST_2 = path_file_2.read().split('\n')
#     PATH_LIST_2.pop()
#     path_file_2.close()
#     TRANSFORMED_PATH_LIST_2 = []
#     for posicion in PATH_LIST_2:
#         pos_x, pos_y = map(int, posicion.split(','))
#         pos_x -= posInicial_x_2
#         pos_y -= posInicial_y_2
#         pos = f'{pos_x},{pos_y}'
#         TRANSFORMED_PATH_LIST_2.append(pos)

MAX_SPEED = 0.5

scene = File_reader(PATH + f'/scenes/Escena-Problema{NUMERO_ESCENA}.txt')

#Posicion Final
posFinal_x, posFinal_y = [scene.qf_x, scene.qf_y]
#Posicion Inicial
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
        self.position_x = 0.75 # Posicion actual del movil / de los sensores
        self.position_y = 0.75
        self.orientation = 0 # Orientacion actual del movil / de los sensores
        self.cmd_vel = Twist()
        self.real_pose = Pose2D()
        self.rate = rospy.Rate(1/0.01)
        self.R = 0.0195 # m
        self.L = 0.0381 # m
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.distance_publisher = rospy.Publisher('/goal_distance', Float32, queue_size=10)
        self.right_motor_speed_publisher = rospy.Publisher('/rightMotorSpeed', Float32, queue_size=10)
        self.left_motor_speed_publisher = rospy.Publisher('/leftMotorSpeed', Float32, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/P3DX/pose', Pose2D, self.pose_cb)
        rospy.Subscriber('/p3dxPosition', Float32MultiArray, callback=self.p3dxPosition_cb)
        rospy.Subscriber('/PioneerOrientation', Float32, callback=self.PioneerOrientation_cb)
        rospy.on_shutdown(self.on_shutdown_cb)

        self.follow_path()

    def pose_cb(self, msg: Pose2D):
        self.position_x = msg.x
        self.position_y = msg.y
        self.orientation = degrees(msg.theta)

    def p3dxPosition_cb(self, msg: Float32MultiArray):
        self.position_x = msg.data[0]
        self.position_y = msg.data[1]

    def PioneerOrientation_cb(self, msg: Float32):
        self.orientation = msg.data # grados

    def follow_path(self):
        old_goal_x = posInicial_x
        old_goal_y = posInicial_y
        for pair in self.path_list:
            rospy.loginfo(f'Paso X, siguiente punto: {pair}')
            goal_x, goal_y = map(float, pair.split(','))
            goal_x = goal_x / 2
            goal_y = goal_y / 2

            #Esto si se tiene que mover hacia la derecha
            #print(f'error: {old_goal_x}')
            if goal_x > old_goal_x:
                sentido = self.get_rotation(self.orientation, 0)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(X, 0.5)

            #Esto si se tiene que mover hacia la izquierda
            #TODO: Tener en cuenta el cambio de signo en el angulo cuando es 180
            elif goal_x < old_goal_x:
                sentido = self.get_rotation(self.orientation, 180)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(X, -0.5)
                self.publish_velocities(0, 0)

            #Esto si se tiene que mover hacia arriba
            elif goal_y > old_goal_y:
                sentido = self.get_rotation(self.orientation, 90)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(Y, 0.5)
                self.publish_velocities(0, 0)

            #Esto si se tiene que mover hacia abajo
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
            return 1 # Derecha
        elif start_angle + 5 < final_angle:
            return -1 # Izquierda
        else:
            return 0 # Nada

    def move_forward(self, direction, distancia):
        rospy.loginfo('Moving forward')
        old_position_value_x = self.position_x
        old_position_value_y = self.position_y
        keep_moving_forward = True
        while keep_moving_forward and not rospy.is_shutdown():
            if direction == Y:
                #print(f"Posicion en Y: {self.position_y}")
                #print(f"Posicion inicial en Y: {old_position_value_y + distancia}")
                if (self.position_y >= old_position_value_y + distancia and (self.orientation > 85 and self.orientation < 95)) or (self.position_y <= old_position_value_y + distancia and (self.orientation < -85 and self.orientation > -95)):
                    keep_moving_forward = False
                    old_position_value_y = self.position_y
                self.publish_velocities(MAX_SPEED, MAX_SPEED)
                rospy.Rate.sleep(self.rate)

            elif direction == X:
                #print(f"Posicion en X: {self.position_x}")
                #print(f"Posicion inicial en X: {old_position_value_x + distancia}")
                if ((self.position_x >= old_position_value_x + distancia) and (self.orientation > - 5 and self.orientation < 5)) or ((self.position_x <= old_position_value_x + distancia) and (((self.orientation < -175 and self.orientation > -185)) or (self.orientation > 175 and self.orientation < 185))):
                    keep_moving_forward = False
                    old_position_value_x = self.position_x
                self.publish_velocities(MAX_SPEED, MAX_SPEED)
                rospy.Rate.sleep(self.rate)

        for i in range(10):
            self.left_motor_speed_publisher.publish(0)
            self.right_motor_speed_publisher.publish(0)

    def rotate_90(self, side):
        rospy.loginfo('Rotating 90 degrees')
        old_orientation_value = self.orientation
        keep_rotating = True
        while keep_rotating and not rospy.is_shutdown():
            keep_rotating = abs(old_orientation_value - self.orientation) < 90
            rospy.logdebug(keep_rotating)
            rospy.logdebug(f'rotating {abs(old_orientation_value - self.orientation)} keep rotating: {keep_rotating}')
            self.publish_velocities(side * -0.1, side * 0.1)

        for i in range(10):
            self.left_motor_speed_publisher.publish(0)
            self.right_motor_speed_publisher.publish(0)

    def publish_velocities(self, rigth_speed, left_speed):
        self.cmd_vel.linear.x = (rigth_speed + left_speed) / 2
        self.cmd_vel.angular.z = (rigth_speed - left_speed) / self.L

        self.right_motor_speed_publisher.publish(rigth_speed)
        self.left_motor_speed_publisher.publish(left_speed)
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def on_shutdown_cb(self):
        self.left_motor_speed_publisher.publish(0)
        self.right_motor_speed_publisher.publish(0)
        rospy.loginfo('Closing node')

if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous=True, log_level=rospy.INFO)
    main = Main(TRANSFORMED_PATH_LIST)
    if NUMERO_ESCENA == 4:
        Main(TRANSFORMED_PATH_LIST_2)
    else:
        main.move_forward(Y, 1)
