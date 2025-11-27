import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SimpleBraitenberg(Node):

    def __init__(self):
        super().__init__('simple_camera')

        self.cv_bridge = CvBridge()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.p0=[]
        self.init = False

        self.scan_subscriber = self.create_subscription(Image, 'image_raw', self.cam_callback, qos)
        self.scan_subscriber  
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Constantes
        self.dmax = 3.0  # Distance maximale pour normalisation
        self.vmax = 0.22  # Vitesse linéaire maximale (m/s)
        self.wmax = 1.5   # Vitesse angulaire maximale (rad/s)

        self.lk_params = dict( winSize = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                              10, 0.03))


        
        self.color = np.random.randint(0, 255, (100, 3))

        self.get_logger().info("AAAAAAAA")  

    def cam_callback(self, image):

        cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')


        #cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)

        

        #height = image.height
        #width = image.width

        #cv2.imshow("Au secours !!!", cv_image)
        #cv2.waitKey(100)

        if not self.init:
            self.init_flow_optique(cv_image)
        else:
            self.boucle_flow_optique(cv_image)


        """

        # Limitation des vitesses
        v = float(max(-self.vmax, min(self.vmax, v)))
        w = float(max(-self.wmax, min(self.wmax, w)))
        
        # Envoi de la commande en vitesse au turtlebot
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        # Constantes
        
        # Affichage pour debug
        self.get_logger().info(
            f'dfront: {dfront:.2f}, dleft: {dleft:.2f}, dright: {dright:.2f} | v: {v:.2f}, w: {w:.2f} '
        )
        
        self.vel_publisher.publish(vel_msg)

        """

    def init_flow_optique(self, old_frame):
        
        self.old_gray = cv2.cvtColor(old_frame,cv2.COLOR_BGR2GRAY)

        max_corner = 20
        quality_level = 0.01
        min_distance = 10

        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, max_corner,quality_level,min_distance)

        self.init = True

        self.mask = np.zeros_like(old_frame)


    def boucle_flow_optique(self, new_frame):


        frame_gray = cv2.cvtColor(new_frame,cv2.COLOR_BGR2GRAY)

        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None)

        # Select good points
        
        print(p1)


        try:

            if len(p1) == 0:
                self.init = False
                return
        except:
            self.init = False
            return
    
        # On assume que p0 et p1 ont la meme taille
        if(len(self.p0) != len(p1)):
            self.get_logger().error("p0 et p1 ont une taille différente")
        N = len(p1)
        c0 = sum(self.p0)/N
        self.get_logger().info(f"c0 = {c0}")
        
        R0 = []
        for R0i in self.p0:
            R0.append(np.abs(R0i - c0))
        self.get_logger().info(f"R0 = {R0}")

        R1 = []
        for R1i in p1:
            R1.append(np.abs(R1i - c0))
        self.get_logger().info(f"R1 = {R1}")

        DeltaR = []
        for i in range(len(R0)):
            DeltaR.append((R1[i] - R0[i]) / (R0[i]) + 0.0001)
        self.get_logger().info(f"DeltaR = {DeltaR}") 


















        Dexp = np.median(DeltaR)
        self.get_logger().info(f"Dexp = {Dexp}")

        good_new = p1[st == 1]
        #good_old = self.p0[st == 1]

        """


        # draw the tracks
        for i, (new, old) in enumerate(zip(good_new, 
                                        good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            a = int(a)
            b = int(b)
            c = int(c)
            d = int(d)
            self.mask = cv2.line(self.mask, (a, b), (c, d),self.color[i].tolist(), 2)
            
            new_frame = cv2.circle(new_frame, (a, b), 5,self.color[i].tolist(), -1)
            
        img = cv2.add(new_frame, self.mask)


        cv2.imshow('frame', img)
        cv2.waitKey(1)

        
        
        """

        

        # Updating Previous frame and points 
        self.old_gray = frame_gray.copy()
        self.p0 = good_new.reshape(-1, 1, 2)
        



def main(args=None):
    rclpy.init(args=args)

    simple_braitenberg = SimpleBraitenberg()
    rclpy.spin(simple_braitenberg)

    simple_braitenberg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 