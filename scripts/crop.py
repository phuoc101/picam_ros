# import glob
import os
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCropper(object):
    def __init__(self):
        self.has_radius_and_center = False
        self.lower_thresh = rospy.get_param("lower_thresh", default=150)
        self.higher_thresh = rospy.get_param("higher_thresh", default=255)
        self.image_topic = rospy.get_param(
            "image_topic", default="/picam360/image_raw")
        self.crop_topic = rospy.get_param(
            "crop_topic", default="/picam360/crop")
        rospack = rospkg.RosPack()
        self.output_path = os.path.join(
            rospack.get_path("picam_ros"),
            rospy.get_param("output_path", default="input_raw_picam360"))
        self.image_subscriber = rospy.Subscriber(
            self.image_topic, Image, self.image_cb)
        self.radius, self.center = None, None
        self.img_n = 0
        self.bridge = CvBridge()

    def get_fisheye_offset(self, img_sample):
        gray = cv2.cvtColor(img_sample, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.lower_thresh,
                                self.higher_thresh, cv2.CV_8UC1)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        largest_contour = contours[0]
        for con in contours:
            if cv2.contourArea(con) > cv2.contourArea(largest_contour):
                largest_contour = con
        center, radius = cv2.minEnclosingCircle(largest_contour)
        center = tuple(np.asarray(center).astype(np.int32))
        radius = np.around(radius, 0).astype(np.int32)
        if not self.has_radius_and_center:
            self.has_radius_and_center = True
        return center, radius

    def crop(self, img):
        if not self.has_radius_and_center:
            self.center, self.radius = self.get_fisheye_offset(
                img_sample=img)
        img_cropped = img[self.center[1]-self.radius:self.center[1] +
                          self.radius, self.center[0]-self.radius:self.center[0]+self.radius]
        filename = f"frame_{self.img_n:05d}.png"
        filepath = os.path.join(self.output_path, filename)
        cv2.imwrite(filepath, img_cropped)
        self.img_n += 1
        rospy.loginfo(f"Saved image {filepath}")

    def image_cb(self, msg):
        img_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.crop(img_cv)

def main():
    rospy.init_node("image_cropper", anonymous=True, log_level=rospy.INFO)
    image_cropper = ImageCropper()
    rospy.spin()


if __name__ == "__main__":
    main()
