#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist

NUMERO_ESCENA = 5
PATH = os.path.dirname(os.path.abspath(__file__))
path_file = open(PATH + f'/paths/Path_list_{NUMERO_ESCENA}.txt')
PATH_LIST = path_file.read().split('\n')
PATH_LIST.pop()
path_file.close()
X = 1
Y = -1

MAX_SPEED = 0.5

obstacle_file = open(PATH + f'/obstacles/Obstacles_{NUMERO_ESCENA}.txt')
obstacle_list = obstacle_file.read().split()
#Posicion Final
posFinal = obstacle_list.pop()
posFinal_x, posFinal_y = map(float, posFinal.split(','))
posFinal_x = posFinal_x / 2 + 0.25
posFinal_y = posFinal_y / 2 + 0.25
#Posicion Inicial
posInicial = obstacle_list.pop()
posInicial_x, posInicial_y = map(float, posInicial.split(','))
posInicial_x = posInicial_x / 2 + 0.25
posInicial_y = posInicial_y / 2 + 0.25

class Main(object):
    def __init__(self, path_list):
        self.path_list = path_list
        self.position_x = 0.75 # Posicion actual del movil / de los sensores
        self.position_y = 0.75
        self.orientation = 0 # Orientacion actual del movil / de los sensores
        self.move_cmd = Twist()
        self.rate = rospy.Rate(1/0.01)
        self.R = 0.0195 # m
        self.L = 0.0381 # m
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.distance_publisher = rospy.Publisher('/goal_distance', Float32, queue_size=10)
        self.right_motor_speed_publisher = rospy.Publisher('/rightMotorSpeed', Float32, queue_size=10)
        self.left_motor_speed_publisher = rospy.Publisher('/leftMotorSpeed', Float32, queue_size=10)
        rospy.Subscriber('/p3dxPosition', Float32MultiArray, callback=self.p3dxPosition_cb)
        rospy.Subscriber('/PioneerOrientation', Float32, callback=self.PioneerOrientation_cb)
        rospy.on_shutdown(self.on_shutdown_cb)

        self.follow_path()

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
            goal_x = goal_x / 2 + 0.25
            goal_y = goal_y / 2 + 0.25

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

            #Esto si se tiene que mover hacia arriba
            elif goal_y > old_goal_y:
                sentido = self.get_rotation(self.orientation, 90)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(Y, 0.5)
                
            #Esto si se tiene que mover hacia abajo
            elif goal_y < old_goal_y:
                sentido = self.get_rotation(self.orientation, -90)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(Y, -0.5)

            old_goal_x = float(pair[0]) / 2 + 0.25
            old_goal_y = float(pair[2]) / 2 + 0.25

        sentido = self.get_rotation(self.orientation, 90)
        while sentido != 0 and not rospy.is_shutdown():
            self.rotate_90(sentido)
            sentido = self.get_rotation(self.orientation, 90)

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
                self.left_motor_speed_publisher.publish(MAX_SPEED)
                self.right_motor_speed_publisher.publish(MAX_SPEED)
                rospy.Rate.sleep(self.rate)

            elif direction == X:
                #print(f"Posicion en X: {self.position_x}")
                #print(f"Posicion inicial en X: {old_position_value_x + distancia}")
                if ((self.position_x >= old_position_value_x + distancia) and (self.orientation > - 5 and self.orientation < 5)) or ((self.position_x <= old_position_value_x + distancia) and (((self.orientation < -175 and self.orientation > -185)) or (self.orientation > 175 and self.orientation < 185))):
                    keep_moving_forward = False
                    old_position_value_x = self.position_x
                self.left_motor_speed_publisher.publish(MAX_SPEED)
                self.right_motor_speed_publisher.publish(MAX_SPEED)
                rospy.Rate.sleep(self.rate)

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
            self.left_motor_speed_publisher.publish(side * 0.1)
            self.right_motor_speed_publisher.publish(-side * 0.1)

        self.left_motor_speed_publisher.publish(0)
        self.right_motor_speed_publisher.publish(0)

    # def publish_velocities(self):
    #     self.left_wheel_velocity = self.move_cmd.angular.z * (self.R - self.L / 2) + self.move_cmd.linear.x
    #     self.right_wheel_velocity = self.move_cmd.angular.z * (self.R + self.L / 2) + self.move_cmd.linear.x
    #     rospy.logdebug(f'Publishing Velocities: left: {self.left_wheel_velocity}, right: {self.right_wheel_velocity}')
    #     self.right_motor_speed_publisher.publish(self.right_wheel_velocity)
    #     self.left_motor_speed_publisher.publish(self.left_wheel_velocity)

    def on_shutdown_cb(self):
        rospy.loginfo('Closing node')

if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous=True, log_level=rospy.INFO)
    Main(PATH_LIST)
