#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SensorLidar(Node):
    def __init__(self):
        super().__init__('robot')
        #Inicializar variables
        self.rangos = []
        self.vel_lineal = 0.05 
        self.vel_angular = 0.0      
        self.acel_actual = 0.0   
        self.desaceleracion = 0
        self.estado_inicial_sensor = False  
        self.aceleracion = -0.15 
        self.vel_actual = self.vel_lineal
        self.giro=0
        self.Rmax=1 # distancia detectada por los objetos
        self.parar=0

        #Crear publicadores
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #Crear suscriptores
        self.suscriptor = self.create_subscription(LaserScan, '/scan', self.sensor, 10)
        #Crear Timers
        self.updateTimer = self.create_timer(0.5, self.actualizacion_estado)
        self.get_logger().info("Uso del sensor lidar para detectar objetos")
    #Funciones callback y funciones importantes
    def sensor(self, msg):
        self.rangos = msg.ranges
        self.estado_inicial_sensor = True

    def actualizacion_estado(self):
        if self.estado_inicial_sensor is True:
            self.controlVel()
            self.condicion()
            self.mostrar_datos()

       
    def controlVel(self):
        vel = Twist()
        self.acel_actual=0.0
        if self.parar==0:   # Si el robot esta en stop 
            #print("PARAR IGUAL A CERO")
            #print("RANGOS",self.rangos[0], self.Rmax)

            if self.rangos[0] < self.Rmax:  # si el rango de medida es menor al establecido se setea la velocidad lineal y angular
                vel.linear.x = 0.0  
                vel.angular.z = 0.5
                if self.giro==0:
                    self.rotacion()
                    
            
            else:                                        # si el robot no esta en el rango establecido 
                print("DESACELERACION",self.desaceleracion)
                vel.linear.x = self.vel_lineal  
                vel.angular.z = self.vel_angular
                
                if self.desaceleracion == 1:              # decremento 
                    self.acel_actual= -0.15
                    self.vel_lineal += self.acel_actual  # self.vel_lineal= self.vel_lineal + self.acel_actual
                    print("INGRESO a desacelerar", vel.linear.x)
                    if vel.linear.x <= 0.0:  # Limitar velocidad lineal a 0
                        vel.linear.x = 0.0
                        print("STOP")
                        self.stop()
                          

                else:
                    print("INGRESO",self.vel_lineal)
                    # desacelerar con una razón de cambio de -0.15 
                    if self.vel_lineal >= 0.70:
                        self.desaceleracion=1
                self.giro=0

        self.publisher.publish(vel)

   
    def condicion(self):
        if self.vel_lineal ==0.7:
            self.desaceleracion == 1

    def stop(self):
        self.giro=2
        self.desaceleracion=2
        self.parar=1

    def rotacion(self):
        self.acel_actual = 0.05 # antes la velocidad de incremento era 0.5
        self.vel_lineal += self.acel_actual
        self.giro=1 


    def mostrar_datos(self):
        distancia_objeto_cercano = self.rangos[0]
        vel=f"Velocidad: {self.vel_lineal:.2f} m/s"
        acel=(f"Aceleración: {self.acel_actual:.2f} m/s^2")
        dis_obj=(f"Distancia de objeto cercano: {distancia_objeto_cercano:.2f} m")

        print(f"{vel} | {acel} | {dis_obj}")



def main(args=None):
    rclpy.init(args = args)
    nodo = SensorLidar()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()