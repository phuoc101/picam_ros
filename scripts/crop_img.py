#!/usr/bin/env python
import rospy
import rospkg
import os
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImgCropperNode:
    def __init__(self):
        rospy.init_node("img_cropper_node")
        rospy.loginfo("Starting ImgCropperNode.")

        rospack = rospkg.RosPack()
        self.config_path = os.path.join(rospack.get_path("picam_ros"), "config")
        self.yaml_file = os.path.join(self.config_path, rospy.get_param("~yaml", "picam_tierankatu_2k.yaml"))
        os.makedirs(self.config_path, exist_ok=True)
        self.center, self.radius = self.load_fisheye_config()

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/picam360/image_raw", Image, self.img_cb)
        self.cropped_img_pub = rospy.Publisher("/picam360/image_cropped", Image, queue_size=10)
        self.rate = rospy.Rate(30)

    def img_cb(self, msg):
        img_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_cropped = img_cv[
            max(0, self.center[1] - self.radius) : self.center[1] + self.radius,
            max(0, self.center[0] - self.radius) : self.center[0] + self.radius,
        ]
        img_cropped_msg = self.bridge.cv2_to_imgmsg(img_cropped, encoding="bgr8")
        self.cropped_img_pub.publish(img_cropped_msg)

    def load_fisheye_config(self):
        with open(self.yaml_file, "r") as f:
            data = yaml.full_load(f)
            cx = data["center"]["cx"]
            cy = data["center"]["cy"]
            center = [cx, cy]
            radius = data["radius"]
            f.close()
        self.has_radius_and_center = True
        return center, radius


if __name__ == "__main__":
    img_cropper_node = ImgCropperNode()
    rospy.spin()
