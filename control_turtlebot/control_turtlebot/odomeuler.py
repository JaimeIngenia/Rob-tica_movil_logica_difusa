import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Odom(Node):
    def __init__(self):
        super().__init__("nodo_odom")
        #Inicializar variables
        self.initOdom = False
        #Incializar el suscriptor
        self.odomsub = self.create_subscription(Odometry, "/odom", self.odomcallback, 10)
        #Crear timer
        self.timer = self.create_timer(0.5, self.updateOdom)
        self.get_logger().info("Estamos sacando las posiciones y orientaciones")

    def odomcallback(self, msg):
        pose = msg.pose.pose
        self.postn = pose.position
        self.orient = pose.orientation
        self.initOdom = True
    def updateOdom(self):
        if self.initOdom == True:
            print("Posicion x ", self.postn.x, "|","Posicion y", self.postn.y, "|","Posicion z", self.postn.z)
            print("Orientacion x ", self.orient.x, "|","Orientacion y", self.orient.y, "|","Orientacion z", self.orient.z)
            print("")

#Inicializar el nodo
def main(args = None):
    rclpy.init(args = args)
    nodo = Odom()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()