import cv2
import numpy as np
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

#Importar librerias

import skfuzzy as fuzz

from skfuzzy import control as crtl

import numpy as np

import matplotlib.pyplot as plt

import time



#Definir entradas (antecedentes y consecuentes)

distancia   = crtl.Antecedent(np.arange(0.2 , 1 , 0.01), "distancia")

orientacion = crtl.Antecedent(np.arange(0.0 , 90, 0.01), "orientacion")

 #Definir  salidas (antecedentes y consecuentes)

velocidad_lineal  = crtl.Consequent(np.arange( 0.0 , 0.2 ,0.01), "velocidad_lineal")

velocidad_angular = crtl.Consequent(np.arange(-1   ,  1  , 0.01), "velocidad_angular")

 

#Definir funciones de pertinencia




class difusa(Node):
    def __init__(self):
        super().__init__('Nodo_navegacion')

        #inicializar variables
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_yaw = 0.0
        self.goal_pose_x = -1.76
        self.goal_pose_y = -0.16
        self.goal_pose_yaw = 1.57
        self.step = 1
        self.ruta=[]
        self.rangos=[]
        self.initOdom = False

        #Inicializar publicadores

        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel", 10)

        #Inicializar el suscriptor
        self.odomsub = self.create_subscription(Odometry, "/odom", self.odomcallback,10)

        self.suscriptor = self.create_subscription(LaserScan, "/scan", self.sensor,10)

        #Crear timer

        self.timer = self.create_timer(0.01, self.updateOdom)

        self.get_logger().info("Actividad 3 ..... start .....")

        def sensor(self, msg):
            self.rangos = msg.ranges
            self.estado_inicial_sensor = True

        
        def odomcallback(self, msg):
            pose = msg.pose.pose

            self.postn = pose.position
            self.orient = pose.orientation
            self.last_pose_x = self.postn.x
            self.last_pose_y = self.postn.y

            _,_,self.last_pose_yaw = self.eulerFromQuaternion(self.orient.x, self.orient.y, self.orient.z, self.orient.w)

            self.initOdom = True
            #MUESTRA  LA UBICACION ACTUAL EN LA TERMINAL
            self.get_logger().info(f'Ubicacion actual: x={self.last_pose_x}, y={self.last_pose_y}, yaw={self.last_pose_yaw}')

        def updateOdom(self):
            if self.initOdom == True:
                self.generate_path()

        def generate_path(self):
            vel = Twist()

            rango1 = np.flip(np.array(self.rangos[1:45]))
            rango2 = np.flip(np.array(self.rangos[315:]))
            rango = np.concatenate((rango1,rango2),axis=0)

            rango_ord=np.argsort(rango)

            if len(rango) >0:
                rango_ord = np.argsort(rango)
                indice = rango_ord[0]
                lidar = rango[indice]

                if type(rango[indice]) != np.float32:
                    lidar = 1
            #paso 1) DEclara variables de entrada y saldia
            distancia = crtl.Antecedent(np.arange(0.2,1,0.01),"distancia")
            orientacion = crtl.Antecedent(np.arange(0,80,1),"orientacion")
            velocidad_angular = crtl.Antecedent(np.arange(-1,1,0.01),"velocidad_angular")


            #paso 2)#DEfiniendo funciones de pertenencia

            #Variable entrada de distancia


            distancia['cerca'] = fuzz.trimf(distancia.universe, [0.19,0.3,0.6])
            distancia['lejos'] = fuzz.trimf(distancia.universe, [0.04,0.7,1.1])

            #Variable entrada de distancia

            orientacion['izq'] = fuzz.trimf(distancia.universe, [-1,45,45])
            orientacion['der'] = fuzz.trimf(distancia.universe, [45,45,91])

            #Variable entrada de distancia

            velocidad_lineal['baja'] = fuzz.trimf(velocidad_lineal.universe, [0.0, 0.07, 0.15])
            velocidad_lineal['alta'] = fuzz.trimf(velocidad_lineal.universe, [0.07, 0.015, 0.2])

            velocidad_angular['izq'] = fuzz.trimf(velocidad_angular.universe, [-1,-0.5,-0.01])
            velocidad_angular['der'] = fuzz.trimf(velocidad_angular.universe, [0.01,0.5,1])
            velocidad_angular['centro'] = fuzz.trimf(velocidad_angular.universe, [-0.01,0,0.01])

            


            regla1 = crtl.Rule(
                distancia['lejos'] & orientacion["der"],
                (velocidad_lineal["alta"], velocidad_angular["centro"])
                )

            regla2 = crtl.Rule(
                distancia["lejos"] & orientacion["izq"], 
                (velocidad_lineal["alta"],velocidad_angular["centro"])
                )

            regla3 = crtl.Rule(
                distancia["cerca"] & orientacion["der"], 
                (velocidad_lineal["baja"],velocidad_angular["izq"])
                )

            regla4 = crtl.Rule(
                distancia["cerca"] & orientacion["izq"], 
                (velocidad_lineal["baja"],velocidad_angular["der"])
                )
            
            controlador = crtl.ControlSystem([regla1,regla2,regla3,regla4])
            simulador = crtl.ControlSystemSimulation(controlador)
            simulador.input["orientacion"] = indice
            simulador.input["distancia"] = lidar

            simulador.compute()

            print("VElocidad del ventilador MIRAR JULIAN:" , simulador.output[ "velocidad_ventilador" ], "km/s")


def main(args=None):
    rclpy.init(args = args)
    nodo = difusa()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#GRaficar las funciones de pertenencia
distancia.view()
orientacion.view()
plt.show()


















































#Variable de salida
velocidad_angular["izq"] = fuzz.trimf(velocidad_angular.universe,[-1,-0.5,-0.01])
velocidad_angular["centro"] = fuzz.trimf(velocidad_angular.universe,[-0.01,0,0.01])
velocidad_angular["der"] = fuzz.trimf(velocidad_angular.universe,[-0.01,0.5,1])



#Reglas
regla1 = crtl.Rule(
    distancia["cerca"] & orientacion["izq"], 
    (velocidad_lineal["baja"],velocidad_angular["der"])
    )

regla2 = crtl.Rule(
    distancia["cerca"] & orientacion["der"], 
    (velocidad_lineal["baja"],velocidad_angular["izq"])
    )

regla3 = crtl.Rule(
    distancia["lejos"] & orientacion["izq"], 
    (velocidad_lineal["alta"],velocidad_angular["centro"])
    )

regla4 = crtl.Rule(
    distancia["lejos"] & orientacion["der"], 
    (velocidad_lineal["alta"],velocidad_angular["centro"])
    )



import time

def main(args = None):
    rclpy.init(args = args)
    nodo = Odom()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()