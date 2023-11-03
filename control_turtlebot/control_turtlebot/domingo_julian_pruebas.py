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
import skfuzzy as fuzz
from skfuzzy import control as crtl
import numpy as np
import matplotlib.pyplot as plt
import time
import warnings
import sys
sys.path.append('/home/jaime/ros2_ws/src/control_turtlebot/control_turtlebot/funciones.py')

#import funciones.py


warnings.filterwarnings("ignore", category=UserWarning)

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
        self.estado_inicial_sensor = True####

    

    def odomcallback(self, msg):
        pose = msg.pose.pose

        self.postn = pose.position
        self.orient = pose.orientation
        self.last_pose_x = self.postn.x
        self.last_pose_y = self.postn.y
        _,_,self.last_pose_yaw = self.eulerFromQuaternion(self.orient.x, self.orient.y, self.orient.z, self.orient.w)
        self.initOdom = True
        


    def updateOdom(self):
        if self.initOdom == True:
            self.generate_path()  

    
    def rotacion(self):
        #self.acel_actual = 0.05 # antes la velocidad de incremento era 0.5
        self.vel_lineal = 0.5
        #self.giro=1 

    

    def generate_path(self):
        
        vel = Twist()
        
        #CAlcula punto objetivo se recalcula constantemente
        self.distance = math.sqrt((self.goal_pose_x - self.last_pose_x)**2 + (self.goal_pose_y -self.last_pose_y)**2)
        #CAlcula los valores para dar el giro-
        self.path_theta = math.atan2(self.goal_pose_x - self.last_pose_x,self.goal_pose_y - self.last_pose_y)
       #Todo para sacar el àngulo
        self.angle = self.path_theta - self.last_pose_yaw
        self.angular_velocity = 0.06
        vel, self.step = self.Turn(self.angle, self.angular_velocity, self.step)
        #Paso 2, desplazar al robot

        rango1 = np.flip(np.array(self.rangos[1:45]))
        #print(rango1)
        rango2 = np.flip(np.array(self.rangos[315:]))
        #print(rango2)
        rango = np.concatenate((rango1,rango2),axis=0)
        #print(rango)
        rango_ord=np.argsort(rango)
        #print(rango_ord)

        if len(rango) >0:

            #CAlcula punto objetivo se recalcula constantemente
            self.distance = math.sqrt((self.goal_pose_x - self.last_pose_x)**2 + (self.goal_pose_y -self.last_pose_y)**2)
            #CAlcula los valores para dar el giro-
            self.path_theta = math.atan2(self.goal_pose_x - self.last_pose_x,self.goal_pose_y - self.last_pose_y)
            #Todo para sacar el àngulo
            self.angle = self.path_theta - self.last_pose_yaw
            self.angular_velocity = 0.06
            vel, self.step = self.Turn(self.angle, self.angular_velocity, self.step)

            rango_ord = np.argsort(rango)
            indice = rango_ord[0]
            lidar = rango[indice]
            print("JAime mira el lidar:" + str(lidar))

            if type(rango[indice]) != np.float32:
                lidar = 1

            distancia = crtl.Antecedent(np.arange(0.2, 1, 0.01), "distancia")
            orientacion = crtl.Antecedent(np.arange(0, 80, 1), "orientacion")
            velocidad_angular = crtl.Consequent(np.arange(-1, 1, 0.01), "velocidad_angular")
            velocidad_lineal = crtl.Consequent(np.arange(0.0, 0.2, 0.01), "velocidad_lineal")

            # Definir funciones de pertenencia
            distancia['cerca'] = fuzz.trimf(distancia.universe, [0.19, 0.3, 0.6])
            distancia['lejos'] = fuzz.trimf(distancia.universe, [0.04, 0.7, 1.1])
            orientacion['izq'] = fuzz.trimf(distancia.universe, [-1, 45, 45])
            orientacion['der'] = fuzz.trimf(distancia.universe, [45, 45, 91])

            velocidad_angular['izq'] = fuzz.trimf(velocidad_angular.universe, [-1, -0.575, -0.25])
            velocidad_angular['der'] = fuzz.trimf(velocidad_angular.universe, [-0.25, 0, 0.25])
            velocidad_angular['centro'] = fuzz.trimf(velocidad_angular.universe, [0.25, 0.575, 1])

            velocidad_lineal['baja'] = fuzz.trimf(velocidad_lineal.universe, [0.0, 0.07, 0.15])
            velocidad_lineal['alta'] = fuzz.trimf(velocidad_lineal.universe, [0.07, 0.15, 0.2])

            # Definir reglas de lógica difusa
            regla1 = crtl.Rule(
                distancia['lejos'] & orientacion["der"],
                (velocidad_lineal["alta"], velocidad_angular["centro"])
            )
            regla2 = crtl.Rule(
                distancia["lejos"] & orientacion["izq"],
                (velocidad_lineal["alta"], velocidad_angular["centro"])
            )
            regla3 = crtl.Rule(
                distancia["cerca"] & orientacion["der"],
                (velocidad_lineal["baja"], velocidad_angular["izq"])
            )
            regla4 = crtl.Rule(
                distancia["cerca"] & orientacion["izq"],
                (velocidad_lineal["baja"], velocidad_angular["der"])
            )

            controlador = crtl.ControlSystem([regla1,regla2,regla3,regla4])
            simulador = crtl.ControlSystemSimulation(controlador)

      
            #print("Input antes de *************************************** :" + str( indice ))
            simulador.input["orientacion"] = indice
            simulador.input["distancia"] = lidar

            simulador.compute()

            vel.linear.x = simulador.output[ 'velocidad_lineal']  
            vel.angular.z = simulador.output[ 'velocidad_angular']  
        

            if lidar >0.5 and self.rangos[ 90] > 0.19 and self.rangos [ 270] > 0.19:

                #CAlcula punto objetivo se recalcula constantemente
                self.distance = math.sqrt((self.goal_pose_x - self.last_pose_x)**2 + (self.goal_pose_y -self.last_pose_y)**2)

                #calcular los valores y dar el giro
                self.path_theta = math.atan2(self.goal_pose_y - self.last_pose_y , self.goal_pose_x - self.last_pose_x)
                
                self.angle = self.path_theta - self.last_pose_yaw
                
                self.angular_velocity = 0.5
                
                vel, self.step = self.Turn(self.angle, self.angular_velocity, self.step)

                vel.linear.x = 0.06


 
            #if type(rango[indice]) != np.float32:
            self.ditance = math.sqrt((self.goal_pose_x - self.last_pose_x)**2 + (self.goal_pose_y - self.last_pose_y)**2 )
            #if lidar < 0.8:
            print("JAIme mira la distancia" + str(self.ditance ))

            if self.distance < 0.2:

                self.ditance = math.sqrt((self.goal_pose_x - self.last_pose_x)**2 + (self.goal_pose_y - self.last_pose_y)**2 )
                
                self.angle = self.goal_pose_yaw -self.last_pose_yaw

                self.angular_velocity = 0.05

                vel, self.step = self.Turn(self.angle, self.angular_velocity , self.step)

                vel.linear.x = 0.0
            #print("Llamado a la lògica difusa*****************************************************")
            #vel.linear.x, vel.angular.z = setup_fuzzy_logic(indice,lidar)
           

            print("Observar el V_W :" + str(vel.linear.z ))
            print("Observar el v_lineal :" + str( vel.linear.x  ))


            self.cmd_vel_pub.publish(vel)  


        #self.cmd_vel_pub.publish(vel)

    def eulerFromQuaternion(self, x, y, z, w):
        roll = math.atan2(2*(w*x + y*z),1 - 2*(x*x + y*y)) 
        pitch = math.asin(2*(w*y - x*z))
        yaw = math.atan2(2*(w*z + x*y),1 - 2*(y*y + z*z)) 

        return roll, pitch, yaw
    

    def Turn(self, angle, angular_velocity, step):
        vel = Twist()
        print("Miremos el angulo"+ str(angle))
        if math.fabs(angle) > 0.01:
            if angle >= 0:
                vel.angular.z = angular_velocity
            elif angle < 0:
                vel.angular.z = -angular_velocity
        else:
            print("llegamos casi a el paso #2")
            step = step + 1
        
        return vel, step
    
    
    def Straight(self, distance , linear_velocity, step):
    
        vel = Twist()
        if distance > 0.16:#0.08:
            vel.linear.x = linear_velocity
        else:
            step = step + 1

        return vel, step
    
    def esquivar(self,vel):
        print("Esta esquivando.....***")
        vel.linear.x = 0.0  
        vel.angular.z = 0.5
        if self.giro==0:
            self.rotacion()
        #self.Turn()
    
    def setup_fuzzy_logic(indice, lidar):
        # Definir variables de entrada y salida
        distancia = crtl.Antecedent(np.arange(0.2, 1, 0.01), "distancia")
        orientacion = crtl.Antecedent(np.arange(0, 80, 1), "orientacion")
        velocidad_angular = crtl.Consequent(np.arange(-1, 1, 0.01), "velocidad_angular")
        velocidad_lineal = crtl.Consequent(np.arange(0.0, 0.2, 0.01), "velocidad_lineal")

        # Definir funciones de pertenencia
        distancia['cerca'] = fuzz.trimf(distancia.universe, [0.19, 0.3, 0.6])
        distancia['lejos'] = fuzz.trimf(distancia.universe, [0.04, 0.7, 1.1])
        orientacion['izq'] = fuzz.trimf(distancia.universe, [-1, 45, 45])
        orientacion['der'] = fuzz.trimf(distancia.universe, [45, 45, 91])

        velocidad_angular['izq'] = fuzz.trimf(velocidad_angular.universe, [-1, -0.575, -0.25])
        velocidad_angular['der'] = fuzz.trimf(velocidad_angular.universe, [-0.25, 0, 0.25])
        velocidad_angular['centro'] = fuzz.trimf(velocidad_angular.universe, [0.25, 0.575, 1])

        velocidad_lineal['baja'] = fuzz.trimf(velocidad_lineal.universe, [0.0, 0.07, 0.15])
        velocidad_lineal['alta'] = fuzz.trimf(velocidad_lineal.universe, [0.07, 0.15, 0.2])

        # Definir reglas de lógica difusa
        regla1 = crtl.Rule(
            distancia['lejos'] & orientacion["der"],
            (velocidad_lineal["alta"], velocidad_angular["centro"])
        )
        regla2 = crtl.Rule(
            distancia["lejos"] & orientacion["izq"],
            (velocidad_lineal["alta"], velocidad_angular["centro"])
        )
        regla3 = crtl.Rule(
            distancia["cerca"] & orientacion["der"],
            (velocidad_lineal["baja"], velocidad_angular["izq"])
        )
        regla4 = crtl.Rule(
            distancia["cerca"] & orientacion["izq"],
            (velocidad_lineal["baja"], velocidad_angular["der"])
        )

        controlador = crtl.ControlSystem([regla1,regla2,regla3,regla4])
        simulador = crtl.ControlSystemSimulation(controlador)

        #indice = 25

        #print("Input antes de *************************************** :" + str( indice ))
        simulador.input["orientacion"] = indice
        simulador.input["distancia"] = lidar

        # print("Observar el input :" + str( indice ))
        # print("Observar el lidar :" + str( lidar ))

        simulador.compute()

        lineal = simulador.output[ 'velocidad_lineal']  
        angular = simulador.output[ 'velocidad_angular']  
        
        return lineal, angular


def main(args=None):
    rclpy.init(args = args)
    nodo = difusa()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
