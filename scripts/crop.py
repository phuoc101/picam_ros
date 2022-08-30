import glob
import os
import cv2
import numpy as np
import rospkg
import rospy
import yaml
from rosbag.bag import Bag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageCropper(object):
    """
    Image Cropper class: Detects contour of fisheye circle and crop image wrt it
    """

    def __init__(self):
        self.has_radius_and_center = False
        self.image_topic = rospy.get_param("~image_topic")
        rospack = rospkg.RosPack()

        # data output paths
        self.output_path = os.path.join(
            rospack.get_path("picam_ros"),
            "data",
            rospy.get_param("~output_path"))
        self.img_path = os.path.join(self.output_path, "frames")
        self.video_path = os.path.join(self.output_path, "video")
        os.makedirs(self.img_path, exist_ok=True)
        os.makedirs(self.video_path, exist_ok=True)

        # ROS bag set up
        self.bagfile = os.path.join(
            rospack.get_path("picam_ros"),
            "bags",
            rospy.get_param("~bagfile"))
        self.bag = Bag(self.bagfile, 'r')
        self.img_count = self.bag.get_message_count(self.image_topic)

        # YAML config setup
        self.config_path = os.path.join(
            rospack.get_path("picam_ros"),
            "config")
        self.yaml_file = os.path.join(
            self.config_path,
            rospy.get_param("~yaml", ""))
        os.makedirs(self.config_path, exist_ok=True)

        self.img_n = 0
        self.bridge = CvBridge()

    def config_fisheye_circle_info(self, img_sample):
        """
        Get fisheye circle radius and center (with GUI)
        """
        width, height = img_sample.shape[1], img_sample.shape[0]
        WINDOW_NAME = "image"
        scale = 1 if width <= 1000 else 2
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, width//scale + 100, height//scale + 100)
        N = 2 * max(img_sample.shape)
        cv2.createTrackbar("radius", WINDOW_NAME, 0, N, self.on_trackbar)
        cv2.createTrackbar("Cx", WINDOW_NAME, N // 2, N, self.on_trackbar)
        cv2.createTrackbar("Cy", WINDOW_NAME, N // 2, N, self.on_trackbar)

        radius = 0
        cx, cy = (0, 0)
        while True:
            if True:
                if scale != 1:
                    frame = cv2.resize(
                        img_sample, (width//scale, height//scale), cv2.INTER_AREA)
                else:
                    frame = img_sample.copy()
                radius = cv2.getTrackbarPos("radius", WINDOW_NAME)
                cx = cv2.getTrackbarPos("Cx", WINDOW_NAME)
                cy = cv2.getTrackbarPos("Cy", WINDOW_NAME)
                frame = cv2.circle(frame, (cx//scale, cy//scale),
                                   radius//scale, (0, 200, 0), 2)
                cv2.imshow(WINDOW_NAME, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()
        center = [cx, cy]
        circle_info = {"center": {"cx": cx, "cy": cy}, "radius": radius}
        with open(self.yaml_file, 'w') as f:
            yaml.dump(circle_info, f)
            f.close()
        self.has_radius_and_center = True
        return center, radius

    def on_trackbar(self, x):
        pass

    def get_fisheye_circle_info(self):
        with open(self.yaml_file, 'r') as f:
            data = yaml.full_load(f)
            cx = data["center"]["cx"]
            cy = data["center"]["cy"]
            center = [cx, cy]
            radius = data["radius"]
            f.close()
        self.has_radius_and_center = True
        return center, radius

    def crop(self, img):
        """
        Crop image around the fisheye circle and save in predetermined img_path
        """
        if not self.has_radius_and_center:
            if not os.path.exists(self.yaml_file):
                self.center, self.radius = self.config_fisheye_circle_info(
                    img_sample=img)
            else:
                self.center, self.radius = self.get_fisheye_circle_info()
        img_cropped = img[max(0, self.center[1]-self.radius):self.center[1]+self.radius,
                          max(0, self.center[0]-self.radius):self.center[0]+self.radius]
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
