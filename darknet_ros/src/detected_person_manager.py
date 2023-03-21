import rospy
from sensor_msgs.msg import Image
from hri_msgs.msg import NormalizedRegionOfInterest2D
import random
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBox

class Person():
    def __init__(self, coordinates, rgb_img):
        self.label = self.generate_label()
        self.coordinates = coordinates
        self.image = rgb_img
        self.bridge = CvBridge()

        # Messages to publish
        self.msg_croppedImg = self.crop_img_msg()
        self.msg_roi = self.calculateROI()

        # Topics to publish
        self.pub_croppedImg = rospy.Publisher("humans/bodies/"+self.label+"/cropped", Image, queue_size=1)
        self.pub_roi = rospy.Publisher("humans/bodies/"+self.label+"/roi", NormalizedRegionOfInterest2D, queue_size=1)

    def generate_label(self):
        return "".join(random.sample("abcdefghijklmnopqrstuvwxyz", 5))

    def update_coordinates(self, coordinates):
        self.coordinates = coordinates
        self.msg_croppedImg = self.crop_img_msg()
        self.msg_roi - self.calculateROI()
        self.publish_topics()

    def crop_img_msg(self):
        cropped_cv_img = self.image[self.coordinates[0]:self.coordinates[1], self.coordinates[2]:self.coordinates[3]] #Ruim
        return self.bridge.cv2_to_imgmsg(cropped_cv_img, encoding="passthrough")

    def calculateROI(self):
        return 0
    
    def publish_topics(self):
        self.pub_croppedImg.publish(self.msg_croppedImg)
        self.pub_roi.publish(self.msg_roi)

    class DetectedPersonManager():
        def __init__(self, topic_rgbImg, topic_boundingBoxes):

            # Control flag for new YOLO detection
            self.new_detection = False

            # Messages
            self.msg_rgbImg = None  # Image
            self.msg_bBoxes = None  # BoundingBox 

            # Subscribers
            self.sub_rgbImg = rospy.Subscriber(topic_rgbImg, Image, self.callback_rgbImg)
            self.sub_bBoxes = rospy.Subscriber(topic_boundingBoxes, BoundingBox, self.callback_bBoxes)

            # Every Person instance of a detection
            self.person_list = None # Person

            # Time
            self.loopRate = rospy.Rate(30)

            # Main
            self.mainLoop()

        def callback_rgbImg(self, msg):
            self.msg_rgbImg = msg

        def callback_bBoxes(self, msg):
            self.msg_bBoxes = msg
            self.new_detection = True

        def mainLoop(self):
            while rospy.is_shutdown() == False:
                self.loopRate.sleep()

                if(self.new_detection == True):
                    for bbox in self.msg_bBoxes:
                        
                self.new_detection = False
