import glob
import os
import cv2
import numpy as np
import rospkg
import rospy
from rosbag.bag import Bag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageCropper(object):
    """
    Image Cropper class: Automatically detects contour of fisheye circle and crop image wrt it
    """

    def __init__(self):
        self.has_radius_and_center = False
        self.lower_thresh = rospy.get_param("~lower_thresh")
        self.higher_thresh = rospy.get_param("~higher_thresh")
        self.image_topic = rospy.get_param("~image_topic")
        rospack = rospkg.RosPack()
        self.output_path = os.path.join(
            rospack.get_path("picam_ros"),
            "data",
            rospy.get_param("~output_path"))
        self.img_path = os.path.join(self.output_path, "frames")
        self.video_path = os.path.join(self.output_path, "video")
        self.bagfile = os.path.join(
            rospack.get_path("picam_ros"),
            "bags",
            rospy.get_param("~bagfile"))
        self.bag = Bag(self.bagfile, 'r')
        self.img_count = self.bag.get_message_count(self.image_topic)
        # os.makedirs(self.output_path, exist_ok=True)
        os.makedirs(self.img_path, exist_ok=True)
        os.makedirs(self.video_path, exist_ok=True)

        self.radius, self.center = None, None
        self.img_n = 0
        self.bridge = CvBridge()

    def get_fisheye_circle_info(self, img_sample):
        """
        Get fisheye circle radius and center
        """
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
        """
        Crop image around the fisheye circle and save in predetermined img_path
        """
        if not self.has_radius_and_center:
            self.center, self.radius = self.get_fisheye_circle_info(
                img_sample=img)
        img_cropped = img[self.center[1]-self.radius:self.center[1] +
                          self.radius, self.center[0]-self.radius:self.center[0]+self.radius]
        filename = f"frame_{self.img_n:05d}.png"
        filepath = os.path.join(self.img_path, filename)
        cv2.imwrite(filepath, img_cropped)
        rospy.loginfo(f"Saved {filepath}")
        self.img_n += 1

    def get_cropped_images(self):
        """
        Open bag file and crop images
        """
        for topic, msg, t in self.bag.read_messages(self.image_topic):
            img_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.crop(img_cv)
        self.bag.close()

    def get_video(self):
        """
        Convert frames collected to mp4 video
        """
        video_name = rospy.get_param("video_name", default="output.mp4")
        os.system(
            f"ffmpeg -r 30 -i {self.img_path}/frame_%05d.png -y {os.path.join(self.video_path, video_name)}")
        # rospy.loginfo(f"Saved video {video_path}")


def main():
    rospy.init_node("image_cropper", anonymous=True, log_level=rospy.INFO)
    image_cropper = ImageCropper()
    image_cropper.get_cropped_images()
    image_cropper.get_video()


if __name__ == "__main__":
    main()
