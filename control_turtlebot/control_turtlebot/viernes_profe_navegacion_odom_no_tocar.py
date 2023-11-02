# import rclpy 
# import math
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from skfuzzy import control as crtl
# import skfuzzy as fuzz
# import numpy as np

# class ControlNavegacion(Node):
#     def __init__(self):
#         super().__init__("Nodo_navegacion")
#         #Inicializar variables
#         self.last_pose_x = 0.0
#         self.last_pose_y = 0.0
#         self.last_pose_yaw = 0.0
#         self.goal_pose_x = 0.5
#         self.goal_pose_y = -1.5
#         self.goal_pose_yaw = math.pi/2
#         self.step = 1
#         self.initOdom = False
#         #Inicializar publicadores
#         self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel",10)
#         #Incializar el suscriptor
#         self.odomsub = self.create_subscription(Odometry, "/odom", self.odomcallback, 10)
#         #Crear timer
#         self.timer = self.create_timer(0.01, self.updateOdom)
#         self.get_logger().info("Hemos inicializado el nodo control de navegacion")

#     def odomcallback(self, msg):
#         pose = msg.pose.pose
#         self.postn = pose.position
#         self.orient = pose.orientation
#         self.last_pose_y = self.postn.x
#         self.last_pose_x = self.postn.y
#         _, _, self.last_pose_yaw = self.Quaternion2Euler(self.orient.x, self.orient.y, self.orient.z, self.orient.w)
#         self.initOdom = True


#     def updateOdom(self):
#         if self.initOdom == True:
#             self.generate_path()

#     def generate_path(self):
#         vel =  Twist()
#         #Paso 1, orientar al robot
#         if self.step == 1:
#             self.path_theta = math.atan2(self.goal_pose_x - self.last_pose_x,self.goal_pose_y - self.last_pose_y)
#             self.angle = self.path_theta - self.last_pose_yaw
#             self.angular_velocity = 0.06
#             vel, self.step = self.Turn(self.angle, self.angular_velocity, self.step)

#             print("step:", self.step)
#         #Paso 2, desplazar al robot
#         elif self.step == 2:

#             self.ditance = math.sqrt((self.goal_pose_x - self.last_pose_x)**2 + (self.goal_pose_y - self.last_pose_y)**2 )
#             self.linear_velocity = 0.1 
#             print("llamado a straight")
           
#             vel, self.step = self.Straight(self.ditance, self.linear_velocity, self.step)
#         #Paso 3, girar el robot
#         elif self.step == 3:
#             self.angle = self.goal_pose_yaw - self.last_pose_yaw
#             self.angular_velocity = 0.06
#             vel, self.step = self.Turn(self.angle, self.angular_velocity, self.step)
#             vel.linear.x = 0.0
#         #Paso 4, parar el robot
#         elif self.step == 4:
#             vel.linear.x = 0.0
#             vel.angular.z = 0.0 

#         self.cmd_vel_pub.publish(vel)

#     def Quaternion2Euler(self, x, y, z, w):
#         roll = math.atan2(2*(w*x + y*z),1 - 2*(x*x + y*y)) 
#         pitch = math.asin(2*(w*y - x*z))
#         yaw = math.atan2(2*(w*z + x*y),1 - 2*(y*y + z*z)) 

#         return roll, pitch, yaw
    
#     def Turn(self, angle, angular_velocity, step):
#         vel = Twist()
#         if math.fabs(angle) > 0.01:
#             if angle >= 0:
#                 vel.angular.z = angular_velocity
#             elif angle < 0:
#                 vel.angular.z = -angular_velocity
#         else:
#             step = step + 1
        
#         return vel, step
    
#     def Straight(self, distance , linear_velocity, step):
    
#         vel = Twist()
#         if distance > 0.08:
#             vel.linear.x = linear_velocity
#         else:
#             step = step + 1

#         return vel, step


# def main(args = None):
#     rclpy.init(args = args)
#     control = ControlNavegacion()
#     rclpy.spin(control)
#     control.destroy_node()
#     rclpy.shutdown()


#     [-1,-0.575,-0.25])

#     [-0.25,0,0.25])

#     [0.25,0.575,1])

  