def visual_imagen(self, msg):
    self.image = msg
    self.cvImage = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
    self.EstadoInicial = True