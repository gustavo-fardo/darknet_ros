#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
import random
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class Person():
    def __init__(self, coordinates, rgb_img):
        self.label = self.generate_label()
        # rospy.loginfo("Person "+self.label+" instance created")
        self.image = rgb_img
        self.matched = False
        self.bridge = CvBridge()

        # Messages to publish
        self.msg_roi = self.formatROI(coordinates)
        self.msg_croppedImg = self.crop_img_msg()

        # Topics to publish
        self.pub_croppedImg = rospy.Publisher("humans/bodies/"+self.label+"/cropped", Image, queue_size=1)
        self.pub_roi = rospy.Publisher("humans/bodies/"+self.label+"/roi", RegionOfInterest, queue_size=1)

        self.publish_topics()

    def __del__(self):
        rospy.loginfo("Person "+self.label+" instance destroyed")
        self.pub_croppedImg.unregister()
        self.pub_roi.unregister()

    def generate_label(self):
        return "".join(random.sample("abcdefghijklmnopqrstuvwxyz", 5))

    def set_match(self, value):
        self.matched = value

    def get_match(self):
        return self.matched

    def update_coordinates(self, coordinates):
        self.msg_croppedImg = self.crop_img_msg()
        self.msg_roi = self.formatROI(coordinates)
        self.publish_topics()

    def crop_img_msg(self):
        cropped_cv_img = self.image[self.msg_roi.y_offset:self.msg_roi.height, self.msg_roi.x_offset:self.msg_roi.width]
        return self.bridge.cv2_to_imgmsg(cropped_cv_img, encoding="passthrough")

    def formatROI(self, coordinates):
        roi = RegionOfInterest()
        roi.x_offset = coordinates[0]
        roi.y_offset = coordinates[1]
        roi.width = coordinates[2]
        roi.height = coordinates[3]
        return roi

    def getROI(self):
        return self.msg_roi
    
    def publish_topics(self):
        self.pub_croppedImg.publish(self.msg_croppedImg)
        self.pub_roi.publish(self.msg_roi)

class DetectedPersonManager():
    def __init__(self, topic_rgbImg, topic_boundingBoxes):

        # Control flag for new YOLO detection
        self.new_detection = False

        # Messages
        self.msg_rgbImg = None  # Image
        self.msg_bBoxes = None  # BoundingBox List

        self.cv_img = None      # CvImage
        self.bridge = CvBridge()

        # Subscribers
        self.sub_rgbImg = rospy.Subscriber(topic_rgbImg, Image, self.callback_rgbImg)
        self.sub_bBoxes = rospy.Subscriber(topic_boundingBoxes, BoundingBoxes, self.callback_bBoxes)

        # Every Person instance of a detection
        self.person_list = [] # Person

        # ROS node
        rospy.init_node('detected_person_manager', anonymous=True)
        
        # Time
        self.loopRate = rospy.Rate(30)
        
        # Main
        self.mainLoop()

    def callback_rgbImg(self, msg):
        self.msg_rgbImg = msg
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
    def callback_bBoxes(self, msg):
        self.msg_bBoxes = msg
        for bbox in self.msg_bBoxes.bounding_boxes:
            coordinates = self.extract_coordinates(bbox)
            repeated_person = False

            for person in self.person_list:
                if(repeated_person == False):
                    # if(person.get_match() == False):
                        repeated_person = self.compareROI(coordinates, person.getROI())
                        person.set_match(repeated_person)
                        rospy.loginfo(repeated_person)
                        rospy.loginfo(len(self.person_list))
                else:
                    break

            if (repeated_person == False):
                self.person_list.append(Person(coordinates, self.cv_img))

        for person in self.person_list:
            if(person.get_match() == False):
                self.person_list.remove(person)
                del person
        
        # for bbox in msg.bounding_boxes:
        #     if(bbox.Class == "person"):
        #         coordinates = self.extract_coordinates(bbox)
        #         same_person = False

        #         for person in self.person_list:
        #             # rospy.loginfo(person.get_match())
        #             if(same_person == False and person.get_match() == False):
        #                 same_person = self.compareROI(coordinates, person.getROI())
        #                 person.set_match(same_person)
        #                 rospy.loginfo(same_person)
        #                 rospy.loginfo(len(self.person_list))

        #         rospy.loginfo(same_person)
        #         if(same_person == False):
        #             self.person_list.append(Person(coordinates, self.cv_img))

        # for person in self.person_list:
        #     if(person.get_match() == False):
        #         self.person_list.remove(person)
        #         del person
        # self.new_detection = True

    def extract_coordinates(self, msg):
        return [msg.xmin, msg.ymin, msg.xmax, msg.ymax]

    def compareROI(self, coordinates, msg):
        left    = coordinates[0]
        btm     = coordinates[1]
        right   = coordinates[2]
        top     = coordinates[3]
        rospy.loginfo("l:{}, b:{}, r:{}, t:{}".format(left, btm, right, top))
        msg_area = msg.height*msg.width
        overlap_perc = ((max(left, msg.x_offset)-min(right, msg.x_offset+msg.width))*(max(btm, msg.y_offset)-min(top, msg.y_offset+msg.height)))/msg_area
        rospy.loginfo(overlap_perc)
        return (overlap_perc > 0.5 and (top-btm)*(right-left) > 0.5*msg_area)

    def mainLoop(self):
        while rospy.is_shutdown() == False:
            self.loopRate.sleep()
            
if __name__ == "__main__":
    DetectedPersonManager(
        "/camera/rgb/image_raw",
        "/darknet_ros/bounding_boxes")


