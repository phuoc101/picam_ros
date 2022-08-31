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


class ImageUwbSync(object):
    """
    Image Uwb Sync class: Detects contour of fisheye (GUI or load yaml) circle and crop image wrt it
    Sync with Uwb position
    """

    def __init__(self):
        self.has_radius_and_center = False
        self.image_topic = rospy.get_param("~image_topic")
        rospack = rospkg.RosPack()

        # data output paths
        self.output_path = os.path.join(
            rospack.get_path("picam_ros"), "data",
            rospy.get_param("~output_path"))
        self.img_path = os.path.join(self.output_path, "frames")
        self.video_path = os.path.join(self.output_path, "video")
        os.makedirs(self.img_path, exist_ok=True)
        os.makedirs(self.video_path, exist_ok=True)

        # ROS bag set up
        self.bag_path = os.path.join(rospack.get_path("picam_ros"), "bags")
        self.bagfile = os.path.join(
            self.bag_path,
            rospy.get_param("~bagfile"))
        self.bag = Bag(self.bagfile, 'r')
        self.img_count = self.bag.get_message_count(self.image_topic)

        # YAML config setup for circle information
        self.config_path = os.path.join(rospack.get_path("picam_ros"), "config")
        self.yaml_file = os.path.join(
            self.config_path,
            rospy.get_param("~yaml", ""))
        os.makedirs(self.config_path, exist_ok=True)

        # UWB setups
        self.bag_topics_list = list(self.bag.get_type_and_topic_info()[1].keys())
        self.uwb_topics = [tp for tp in self.bag_topics_list if 'tag' in tp]
        self.uwb_tags = [tp.split('/')[3] for tp in self.uwb_topics]
        self.__t_prev = [None for x in self.uwb_topics]
        self.uwb_path = os.path.join(self.output_path, "uwb_labels")
        os.makedirs(self.uwb_path, exist_ok=True)

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
        with open(self.yaml_file, 'w+') as f:
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

    def get_crop_and_annotations(self):
        """
        Open bag file and crop images
        UWB annotation format: "tagname_0 x0 y0 ... tagname_n xn yn"
        """
        for _, msg_img, t_img in self.bag.read_messages(self.image_topic):
            img_cv = self.bridge.imgmsg_to_cv2(msg_img, desired_encoding="bgr8")
            if not self.has_radius_and_center:
                if not os.path.exists(self.yaml_file):
                    self.center, self.radius = self.config_fisheye_circle_info(
                        img_sample=img_cv)
                else:
                    self.center, self.radius = self.get_fisheye_circle_info()
            img_cropped = img_cv[max(0, self.center[1]-self.radius):self.center[1]+self.radius,
                              max(0, self.center[0]-self.radius):self.center[0]+self.radius]
            filename = f"frame_{self.img_n:05d}.png"
            filepath = os.path.join(self.img_path, filename)
            cv2.imwrite(filepath, img_cropped)
            rospy.loginfo(f"Saved {filepath}")
            uwb_filename = f"frame_{self.img_n:05d}.txt"
            uwb_filepath = os.path.join(self.uwb_path, uwb_filename)
            anno = ['' for x in self.uwb_topics]
            if self.uwb_topics:
                for i, topic in enumerate(self.uwb_topics):
                    for _, msg_pos, t_pos in self.bag.read_messages(topic, start_time=self.__t_prev[i]):
                        if t_pos >= t_img:
                            rospy.logdebug(f"Tag: {self.uwb_tags[i]} Closest Coord x: {msg_pos.pose.position.x} y: {msg_pos.pose.position.y}")
                            anno[i] = f"{self.uwb_tags[i]} {msg_pos.pose.position.x} {msg_pos.pose.position.y}\n"
                            self.__t_prev[i] = t_pos
                            break
                with open(uwb_filepath, 'w+') as f:
                    for txt in anno:
                        f.write(txt)
                    rospy.logdebug(f"Written uwb labels {uwb_filepath}")
                    f.close()

            self.img_n += 1
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
    rospy.init_node("image_cropper", anonymous=True, log_level=rospy.DEBUG)
    image_cropper = ImageUwbSync()
    image_cropper.get_crop_and_annotations()
    image_cropper.get_video()


if __name__ == "__main__":
    main()
