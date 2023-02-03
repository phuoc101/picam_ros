#!/usr/bin/env python
# import glob
# import numpy as np
# from sensor_msgs.msg import Image
import os
import cv2
import rospkg
import rospy
import yaml
import numpy as np
from rosbag.bag import Bag
from cv_bridge import CvBridge


class ImageTrackSync(object):
    """
    ImageTrackSync class: Detects contour of fisheye (GUI or load yaml) circle and crop
    image wrt it
    Sync with Optitrack position
    """

    def __init__(self):
        self.has_radius_and_center = False
        self.image_topic = rospy.get_param("~image_topic", "/picam360/image_raw")
        rospack = rospkg.RosPack()

        # data output paths
        self.output_path = os.path.join(
            rospack.get_path("picam_ros"),
            "extracted_data",
            rospy.get_parm("~output_path", "Tierankatu"),
        )
        self.img_path = os.path.join(self.output_path, "frames")
        self.video_path = os.path.join(self.output_path, "video")
        os.makedirs(self.img_path, exist_ok=True)
        os.makedirs(self.video_path, exist_ok=True)

        # ROS bag set up
        self.bag_path = os.path.join(rospack.get_path("picam_ros"), "bags")
        self.bagfile = os.path.join(
            self.bag_path,
            rospy.get_param(
                "~bagfile",
                "Tierankatu/picam_test_2K_with_telloblack_2022-11-29-15-51-41.bag",
            ),
        )
        self.bag = Bag(self.bagfile, "r")
        self.img_count = self.bag.get_message_count(self.image_topic)

        # YAML config setup for circle information
        self.config_path = os.path.join(rospack.get_path("picam_ros"), "config")
        self.yaml_file = os.path.join(
            self.config_path, rospy.get_param("~yaml", "default.yaml")
        )
        os.makedirs(self.config_path, exist_ok=True)

        # Optitrack setups
        self.cam_pos_topic = rospy.get_param(
            "~cam_pos_topic", "/vrpn_client_node/pycam/pose"
        )
        self.cam_pos = [None, None]
        self.tracked_topics = rospy.get_param(
            "~tracked_topics",
            [
                "/vrpn_client_node/cardboard/pose",
                "/vrpn_client_node/greylid/pose",
                "/vrpn_client_node/telloblack/pose",
            ],
        )
        self.tracked_dist = [None for tp in self.tracked_topics]
        self.t_prev = [None for x in self.tracked_topics]
        self.track_path = os.path.join(self.output_path, "labels", "tracker_labels")
        os.makedirs(self.track_path, exist_ok=True)

        self.img_n = 0
        self.bridge = CvBridge()

    def fisheye_config(self, img_sample):
        """
        Get fisheye circle radius and center (with GUI)
        """
        width, height = img_sample.shape[1], img_sample.shape[0]
        WINDOW_NAME = "image"
        scale = 1 if width <= 1000 else 2
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, width // scale + 100, height // scale + 100)
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
                        img_sample, (width // scale, height // scale), cv2.INTER_AREA
                    )
                else:
                    frame = img_sample.copy()
                radius = cv2.getTrackbarPos("radius", WINDOW_NAME)
                cx = cv2.getTrackbarPos("Cx", WINDOW_NAME)
                cy = cv2.getTrackbarPos("Cy", WINDOW_NAME)
                frame = cv2.circle(
                    frame, (cx // scale, cy // scale), radius // scale, (0, 200, 0), 2
                )
                cv2.imshow(WINDOW_NAME, frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        cv2.destroyAllWindows()
        center = [cx, cy]
        circle_info = {"center": {"cx": cx, "cy": cy}, "radius": radius}
        with open(self.yaml_file, "w+") as f:
            yaml.dump(circle_info, f)
            f.close()
        self.has_radius_and_center = True
        return center, radius

    def on_trackbar(self, x):
        pass

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

    def get_crop_and_track_annotations(self):
        """
        Open bag file and crop images
        If has optitrack info, save image and annotation
        """
        for topic, msg, t in self.bag.read_messages(
            [self.image_topic] + self.tracked_topics + [self.cam_pos_topic]
        ):
            if topic == self.image_topic:
                img_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if not self.has_radius_and_center:
                    if not os.path.exists(self.yaml_file):
                        self.center, self.radius = self.fisheye_config(
                            img_sample=img_cv
                        )
                    else:
                        self.center, self.radius = self.load_fisheye_config()
                if self.cam_pos != [None, None] and None not in self.tracked_dist:
                    img_cropped = img_cv[
                        max(0, self.center[1] - self.radius) : self.center[1]
                        + self.radius,
                        max(0, self.center[0] - self.radius) : self.center[0]
                        + self.radius,
                    ]
                    filename = f"frame_{self.img_n:05d}"
                    img_filepath = os.path.join(self.img_path, filename + ".png")
                    cv2.imwrite(img_filepath, img_cropped)
                    anno_fileppath = os.path.join(self.track_path, filename + ".txt")
                    anno = ""
                    for i, topic in enumerate(self.tracked_topics):
                        anno += f"{topic} {self.tracked_dist[i]}\n"
                    with open(anno_fileppath, "w") as f:
                        f.write(anno)
                        f.close()
                    self.img_n += 1
            elif topic == self.cam_pos_topic:
                self.cam_pos = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ]
                rospy.logdebug(f"Cam pos: {self.cam_pos}")
            elif topic in self.tracked_topics:
                target_pos = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ]
                if self.cam_pos != [None, None]:
                    self.tracked_dist[
                        self.tracked_topics.index(topic)
                    ] = np.linalg.norm(np.array(self.cam_pos) - np.array(target_pos))
                    rospy.logdebug(
                        "Target: {} Dist: {}".format(
                            topic,
                            self.tracked_dist[self.tracked_topics.index(topic)],
                        )
                    )

        self.bag.close()

    def get_video(self):
        """
        Convert frames collected to mp4 video
        """
        video_name = rospy.get_param("video_name", default="output.mp4")
        video_path = os.path.join(self.video_path, video_name)
        os.system(f"ffmpeg -r 30 -i {self.img_path}/frame_%05d.png -y {video_path}")
        rospy.loginfo(f"Saved video {video_name}")


def main():
    rospy.init_node("image_cropper", anonymous=True, log_level=rospy.INFO)
    image_cropper = ImageTrackSync()
    image_cropper.get_crop_and_track_annotations()
    image_cropper.get_video()


if __name__ == "__main__":
    main()
