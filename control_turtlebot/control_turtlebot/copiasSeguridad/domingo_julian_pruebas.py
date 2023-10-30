# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math


class odomNode(Node):
    def __init__(self):
        super().__init__('odomContorl')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.image = []
        #self.bridge = CvBridge()
        self.linear_velocity = 0.0  # unit: m/s
        self.angular_velocity = 0.0  # unit: m/s
        self.initScanState = False  # To get the initial scan data at the beginning        
        self.bandera= False #varianle bandera
        self.bandera1=0 #variable bandera para el inicio para que el primer momento las velocidades sean 0 
        self.laneFitBef = []
        self.ctrlStartP = False
        self.pid=PID(0.01,0.0,0.0)
        self.starto = 0
        self.startp = 0
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmdVelPub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.odomSub = self.create_subscription(Odometry, "/odom", self.odomCallback, qos_profile=qos_profile_sensor_data)
        
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scanCallback, qos_profile=qos_profile_sensor_data)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.updateTimer = self.create_timer(0.01, self.updateCallback)

        self.get_logger().info("Turtlebot3 lane detection node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scanCallback(self,msg):
        self._scan = msg
        print('excelente')
        self.initScanState = True
        
    def eulerFromQuaternion(self, x, y, z, w):
        t0 = +2.0*(w*x+y*z)
        t1 = +1.0-2.0*(x*x+y*y)
        rollX = math.atan2(t0, t1)
        
        t2 = +2.0*(w*x-y*z)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = +1.0 if t2 < -1.0 else t2
        pitchY = math.asin(t2)
        
        t3 = +2.0*(w*z+x*y)
        t4 = +1.0 - 2.0*(y*y+z*z)
        yawZ = math.atan2(t3, t4)
        
        return rollX, pitchY, yawZ
    
    def yaw2Degrees(self, yaw):
          if yaw >= 0: 
                degree = yaw*180/math.pi
          else:
                tmp = yaw*180/math.pi
                degree = 360 + tmp
          return degree
    
    def odomCallback(self, msg):
          pose = msg.pose.pose
          self.position = pose.position
          self.orientation = pose.orientation
          orientationQ = self.orientation
          (roll, pitch, yaw) = self.eulerFromQuaternion(orientationQ.x, orientationQ.y, orientationQ.z, orientationQ.w)
          self.grd = self.yaw2Degrees(yaw)
        #print(postn.x, postn.y, postn.z)
        #print(oritn.x, oritn.y, oritn.z)
        

            
    def updateCallback(self):
          if self.initScanState is True:
            print('llegó')
            self.StartX = 0.0 #posicion en x objetivo del robot 3.58
            self.StartY = -2.0 #posicion en y objetivo del robot 0.198
            d=0.5 #umbral antes 1.0 domingo, condiciòn de la medida
            #self.scanRanges = LaserScan.ranges
            self.scanRanges = self._scan.ranges
            for i in range (len(self.scanRanges)):
                if self.scanRanges[i] == float('inf'):
                    self.scanRanges[i]=3.5
                    
            x=(self.StartX-self.position.x)
            y=(self.StartY-self.position.y)
            angulo_deseado=math.atan2(y,x)
            angulo_deseado= self.yaw2Degrees(angulo_deseado)
            distancia=math.sqrt(math.pow(x,2)+math.pow(y,2))        
            
            
            
            if (angulo_deseado<0):
                angulo_deseado=(angulo_deseado+360)%360
            #print(angulo_deseado)
            error=self.grd-angulo_deseado
            #print(error)
            #Calculo de error
            #sig=(1 / (1 +(np.exp(-error))))-0.5
            #velocidad=Twist()
            #velocidad.angular.z=sig
            #self.cmdVelPub.publish(velocidad)
            #velocidad=Twist()
            #kp=self.pid.update(angulo_deseado,self.grd)
            #velocidad.angular.z=kp*0.5
            #self.cmdVelPub.publish(velocidad)
            
            #acomodamos la pose del robot con la pose objetivo
            
            if error>0:
                if error<180 or error<=-180:
                    velocidad=Twist()
                    velocidad.angular.z=-0.5
                    self.cmdVelPub.publish(velocidad)
                else:
                    velocidad=Twist()
                    velocidad.angular.z=0.5
                    self.cmdVelPub.publish(velocidad)
            else:
                if error>-180:
                    velocidad=Twist()
                    velocidad.angular.z=0.5
                    self.cmdVelPub.publish(velocidad)
                else:
                    velocidad=Twist()
                    velocidad.angular.z=-0.5
                    self.cmdVelPub.publish(velocidad)
            '''
            self.pid.SetPoint(angulo_deseado)
            self.pid.update(self.grd)
            velocidad=Twist()
            velocidad.angular.z=self.pid.output
            self.cmdVelPub.publish(velocidad)
            '''        
            #CUANDO EL EERROR ENTRE LA POSE ACTUAL Y LA POSE DESEADA 
            #SE ENCUENTRE EN EUN RANGO ACEPTABLE PROCEDEMOS A
            #SE CALCULA LA FUERZA DE ATRACCION LA CUAL VA A AFECTAR A LA VELOCIDAD LINEAL
            
            ka=1.0
            
            fuerza_atractiva=ka*(distancia)
            
            if error>-4 and error<4:
                  
                if distancia>=0.2:
                    velocidad=Twist()
                    velocidad.angular.z=0.0
                    velocidad.linear.x=fuerza_atractiva
                    self.cmdVelPub.publish(velocidad)
                  
                      
                else:
                    velocidad=Twist()
                    velocidad.angular.z=0.0
                    velocidad.linear.x=0.0
                    self.cmdVelPub.publish(velocidad)
               
            #frente=np.append(self.scanRanges[315:360],self.scanRanges[0:45])
            #frente=np.mean(frente)
            izquierda=np.mean(self.scanRanges[15:25])
            derecha=np.mean(self.scanRanges[315:325])
            frente=self.scanRanges[0]
            if frente<=d or izquierda<=d or derecha<=d :
                  
                print('Esta cerca de un obstaculo')
                self.evasion_obstaculo()
                #creamos lafuerza de repulsion
  

    def evasion_obstaculo(self):
          izquierda=np.mean(self.scanRanges[15:25])
          derecha=np.mean(self.scanRanges[315:325])
          
          #frente=np.append(self.scanRanges[315:360],self.scanRanges[0:45])
          #frente=np.mean(frente)
          d0=1
          #frente=np.mean(self.scanRanges[0:1])
          frente=self.scanRanges[0]
          if self.scanRanges[0] == float('inf'):
                
                self.scanRanges[0]=3.5
          kobs=0.1
          a = (kobs*(((1/frente)-1/d0)*(1/(frente**2))))-0.5
          #a=0.5#Variable para generar velocidad y comparar
          #a=math.sqrt(frx**2,fry**2)
          velocidad=Twist()
          if izquierda>a and frente>a and derecha>a:
              velocidad.linear.x=0.15
              velocidad.angular.z=0.0
              self.cmdVelPub.publish(velocidad)

          #Obstaculo en el centro
          elif izquierda>a and frente<a and derecha>a:
              velocidad.linear.x=0.0
              velocidad.angular.z=1.0 
              self.cmdVelPub.publish(velocidad)

          #Obstaculo en la derecha
          elif izquierda>a and frente>a and derecha<a:
              velocidad.linear.x=0.0
              velocidad.angular.z=1.0
              self.cmdVelPub.publish(velocidad)

          #Obstaculo en la izquierda
          elif izquierda<a and frente>a and derecha>a:
              velocidad.linear.x=0.0
              velocidad.angular.z=-1.0
              self.cmdVelPub.publish(velocidad)

          #Obstaculo en la derecha y en el centro
          elif izquierda>a and frente<a and derecha<a:
              velocidad.linear.x=0.0
              velocidad.angular.z=1.0
              self.cmdVelPub.publish(velocidad)

          #Obstaculo en la izquierda y en el centro
          elif izquierda<a and frente<a and derecha>a:
              velocidad.linear.x=0.0
              velocidad.angular.z=-1.0
              self.cmdVelPub.publish(velocidad)

          #Obstaculo en la izquierda y en la derecha
          elif izquierda<a and frente>a and derecha<a:
              velocidad.linear.x=0.2
              velocidad.angular.z=0.0
              self.cmdVelPub.publish(velocidad)

          #Obstaculo en la izquierda, en la derecha y en el centro
          elif izquierda<a and frente<a and derecha<a:
              velocidad.linear.x=0.0
              velocidad.angular.z=1.0
              self.cmdVelPub.publish(velocidad)
            
import time
 
class PID:
  def __init__(self, P=0.2, I=0.0, D=0.0):
    self.Kp = P
    self.Ki = I
    self.Kd = D
    self.sample_time = 0.00
    self.current_time = time.time()
    self.last_time = self.current_time
    self.clear()
  def clear(self):
    self.SetPoint = 0.0
    self.PTerm = 0.0
    self.ITerm = 0.0
    self.DTerm = 0.0
    self.last_error = 0.0
    self.int_error = 0.0
    self.windup_guard = 20.0
    self.output = 0.0
  def update(self, feedback_value):
    error = self.SetPoint - feedback_value
    self.current_time = time.time()
    delta_time = self.current_time - self.last_time
    delta_error = error - self.last_error
    if (delta_time >= self.sample_time):
      self.PTerm = self.Kp * error#proporción
      self.ITerm += error * delta_time#integral
      if (self.ITerm < -self.windup_guard):
        self.ITerm = -self.windup_guard
      elif (self.ITerm > self.windup_guard):
        self.ITerm = self.windup_guard
      self.DTerm = 0.0
      if delta_time > 0:
        self.DTerm = delta_error / delta_time
      self.last_time = self.current_time
      self.last_error = error
      self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
      
       
  def setKp(self, proportional_gain):
    self.Kp = proportional_gain
  def setKi(self, integral_gain):
    self.Ki = integral_gain
  def setKd(self, derivative_gain):
    self.Kd = derivative_gain
  def setWindup(self, windup):
    self.windup_guard = windup
  def setSampleTime(self, sample_time):
    self.sample_time = sample_time
            
            

def main(args=None):
    print('start')
    rclpy.init(args=args)
    odomContorl = odomNode()
    rclpy.spin(odomContorl)

    odomContorl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()