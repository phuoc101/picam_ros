#!/usr/bin/sh
DEFAULT_BAGFILE_NAME="picam_raw"
TOPICS_TO_COLLECT="/picam360/image_raw /picam360/camera_info"
BAGFILE_NAME=$DEFAULT_BAGFILE_NAME
while getopts ":n:t:" flag
do
  case "${flag}" in
    n) BAGFILE_NAME=${OPTARG};;
    t) TOPICS_TO_COLLECT=${OPTARG};;
    \?) echo "Invalid option -$OPTARG" >&2
    exit 1;;
  esac
done
echo "BAG NAME: $BAGFILE_NAME";
echo "TOPICS: $TOPICS_TO_COLLECT"


# v4l2-ctl --device=/dev/video2 --list-formats-ext #to show camera formats
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=640 image_height:=480 framerate:=30 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=800 image_height:=600 framerate:=30 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=1024 image_height:=768 framerate:=30 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=1280 image_height:=720 framerate:=30 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=1920 image_height:=1080 framerate:=30 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=1600 image_height:=1200 framerate:=30 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
roslaunch picam_ros picam_read.launch record_cam:=true image_width:=2048 image_height:=1536 framerate:=30 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=2592 image_height:=1944 framerate:=20 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=3840 image_height:=2160 framerate:=25 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=3264 image_height:=2448 framerate:=20 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
# roslaunch picam_ros picam_read.launch record_cam:=true image_width:=3840 image_height:=2880 framerate:=5 bagfile_name:="$BAGFILE_NAME" video_device:=/dev/video2 topics_to_collect:="$TOPICS_TO_COLLECT"
