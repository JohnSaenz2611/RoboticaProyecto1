#!/usr/bin/env python3

import os
import sys
from math import degrees
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from file_reader import File_reader
from A_star_class import A_star
from tf.transformations import euler_from_quaternion


MAX_SPEED = 0.5
try:
    MAX_SPEED_LINEAL = float(sys.argv[1])
    MAX_SPEED_ANGULAR = float(sys.argv[2])
except:
    MAX_SPEED_LINEAL = 1
    MAX_SPEED_ANGULAR = 1

NUMERO_ESCENA = 1
PATH = os.path.dirname(os.path.abspath(__file__))
scene = File_reader(PATH + f'/scenes/Escena-Problema{NUMERO_ESCENA}.txt')

PATH_LIST = A_star(
    numero_escena=NUMERO_ESCENA,
    q0_x=scene.q0_x,
    q0_y=scene.q0_y,
    qf_x=scene.qf_x,
    qf_y=scene.qf_y,
    obstacle_list=scene.obstacle_list
).path

X = 1
Y = -1

# Posicion Final
posFinal_x_2 = scene.qLoc_x
posFinal_y_2 = scene.qLoc_y
# Posicion Inicial
posInicial_x_2 = 0 if scene.qf_x == 0 else (scene.qf_x - 0.25) * 2
posInicial_y_2 = 0 if scene.qf_y == 0 else (scene.qf_y - 0.25) * 2

PATH_LIST_2 = A_star(
    numero_escena=f'{NUMERO_ESCENA}_ReLoc',
    q0_x=scene.qf_x,
    q0_y=scene.qf_y,
    qf_x=scene.qLoc_x,
    qf_y=scene.qLoc_y,
    obstacle_list=scene.obstacle_list
).path

TRANSFORMED_PATH_LIST_2 = []
for posicion in PATH_LIST_2:
    pos_x, pos_y = posicion
    pos_x -= posInicial_x_2
    pos_y -= posInicial_y_2
    pos = f'{pos_x},{pos_y}'
    TRANSFORMED_PATH_LIST_2.append(pos)

# Posicion Final
posFinal_x, posFinal_y = [scene.qf_x, scene.qf_y]
# Posicion Inicial
posInicial_x = 0 if scene.q0_x == 0 else (scene.q0_x - 0.25) * 2
posInicial_y = 0 if scene.q0_y == 0 else (scene.q0_y - 0.25) * 2

def transform_path_list(path_list, posInicial_x, posInicial_y):
    TRANSFORMED_PATH_LIST = []
    for posicion in path_list:
        pos_x, pos_y = posicion
        pos_x -= posInicial_x
        pos_y -= posInicial_y
        pos = f'{pos_x},{pos_y}'
        TRANSFORMED_PATH_LIST.append(pos)
    return TRANSFORMED_PATH_LIST

TRANSFORMED_PATH_LIST = transform_path_list(PATH_LIST, posInicial_x, posInicial_y)

posInicial_x = 0
posInicial_y = 0
aux = TRANSFORMED_PATH_LIST[-1]
aux = aux.split(',')
posFinal_x = 0 if float(aux[0]) == 0 else float(aux[0]) / 2 + 0.25
posFinal_y = 0 if float(aux[1]) == 0 else float(aux[1]) / 2 + 0.25

class Main(object):
    def __init__(self, path_list):
        self.path_list = path_list
        self.is_finish = False
        self.max_speed_multiplier = MAX_SPEED_LINEAL
        self.front_sensor = 0
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
        self.faced_rotation = 0
        self.distance_publisher = rospy.Publisher('/goal_distance', Float32, queue_size=1)
        self.right_motor_speed_publisher = rospy.Publisher('/rightMotorSpeed', Float32, queue_size=1)
        self.left_motor_speed_publisher = rospy.Publisher('/leftMotorSpeed', Float32, queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher('/P3DX_RosAria/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/P3DX_RosAria/2D_pose', Pose2D, self.pose_cb, queue_size=1)
        rospy.Subscriber('/p3dxPosition', Float32MultiArray, callback=self.p3dxPosition_cb, queue_size=1)
        rospy.Subscriber('/PioneerOrientation', Float32, callback=self.PioneerOrientation_cb, queue_size=1)
        rospy.Subscriber('/p3dxSensors', Float32MultiArray, callback=self.pioneer_copelia_sensors_cb, queue_size=1)
        self.angular_speed_publisher = rospy.Publisher('/angularSpeed', Float32, queue_size=1)
        rospy.on_shutdown(self.on_shutdown_cb)

        rospy.Subscriber("/RosAria/pose", Odometry, callback=self.p3dxAngular_cb, queue_size=1)

        while not self.is_finish:
            self.follow_path()
        
    def pioneer_copelia_sensors_cb(self, msg: Float32MultiArray):
        self.front_sensor = msg.data[4]
        rospy.logdebug(self.front_sensor)

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
            if pair == self.path_list[-1]:
                self.is_finish = True
            rospy.loginfo(f'Paso X, siguiente punto: {pair}')
            goal_x, goal_y = map(float, pair.split(','))
            goal_x = goal_x / 2
            goal_y = goal_y / 2

            # Esto si se tiene que mover hacia la derecha
            #print(f'error: {old_goal_x}')
            if goal_x > old_goal_x:
                self.faced_rotation = 1 # Derecha
                sentido = self.get_rotation(self.orientation, 0)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(X, 0.5)
                if self.look_for_obstacle(0.2):
                    break

            # Esto si se tiene que mover hacia la izquierda
            # TODO: Tener en cuenta el cambio de signo en el angulo cuando es 180
            elif goal_x < old_goal_x:
                self.faced_rotation = -1 # Izquierda
                sentido = self.get_rotation(self.orientation, 180)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(X, -0.5)
                self.publish_velocities(0, 0)
                if self.look_for_obstacle(0.2):
                    break

            # Esto si se tiene que mover hacia arriba
            elif goal_y > old_goal_y:
                self.faced_rotation = 2 # Arriba
                sentido = self.get_rotation(self.orientation, 90)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(Y, 0.5)
                self.publish_velocities(0, 0)
                if self.look_for_obstacle(0.2):
                    break

            # Esto si se tiene que mover hacia abajo
            elif goal_y < old_goal_y:
                self.faced_rotation = -2 # Abajo
                sentido = self.get_rotation(self.orientation, -90)
                if sentido != 0:
                    self.rotate_90(sentido)
                self.move_forward(Y, -0.5)
                if self.look_for_obstacle(0.2):
                    break

            old_goal_x = goal_x
            old_goal_y = goal_y

        if self.is_finish:
            # Mirar hacia el segundo punto
            sentido = self.get_rotation(self.orientation, scene.qf_theta)
            while sentido != 0 and not rospy.is_shutdown():
                self.rotate_90(sentido)
                sentido = self.get_rotation(self.orientation, scene.qf_theta)

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

    def look_for_obstacle(self, detection_distance):
        if self.front_sensor != 0 and self.front_sensor < detection_distance:
            x = round((self.position_x - 0.25) * 2, 0) 
            y = 9 - round((self.position_y - 0.25) * 2, 0)

            if self.faced_rotation == 1: # Derecha
                x += 1
            if self.faced_rotation == -1: # Izquierda
                x -= 1
            if self.faced_rotation == 2: # Arriba
                y -= 1
            if self.faced_rotation == -2: # Abajo
                y += 1

            if [x, y] in scene.obstacle_list:
                return False
            else:
                scene.obstacle_list.append([x, y])
                PATH_LIST = A_star(
                    numero_escena=NUMERO_ESCENA,
                    q0_x=(round((self.position_x - 0.25) * 2, 0)) / 2 + 0.25,
                    q0_y=(round((self.position_y - 0.25) * 2, 0)) / 2 + 0.25,
                    qf_x=scene.qf_x,
                    qf_y=scene.qf_y,
                    obstacle_list=scene.obstacle_list
                ).path

                self.path_list = transform_path_list(PATH_LIST, posInicial_x, posInicial_y)

                return True
        return False

    def move_forward(self, direction, distancia):
        self.max_speed_multiplier = MAX_SPEED_LINEAL
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

        pass

    def rotate_90(self, side):
        rospy.loginfo('Rotating 90 degrees')
        old_orientation_value = self.orientation
        keep_rotating = True
        self.max_speed_multiplier = MAX_SPEED_ANGULAR
        while keep_rotating and not rospy.is_shutdown():
            if abs(old_orientation_value - self.orientation) > 90 - 5:
                # Cambiar el 0.01 si gira muy lento
                # TODO !!!!!!!!!!!!!!!
                self.max_speed_multiplier = 2
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
