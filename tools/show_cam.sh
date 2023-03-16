#!/usr/bin/sh
# v4l2-ctl --device=/dev/video2 --list-formats-ext #to show camera formats
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=640 image_height:=480 framerate:=30 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=800 image_height:=600 framerate:=30 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=1024 image_height:=768 framerate:=30 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=1280 image_height:=720 framerate:=30 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=1920 image_height:=1080 framerate:=30 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=1600 image_height:=1200 framerate:=30 video_device:=/dev/video2
roslaunch picam_ros picam_read.launch show_img:=true image_width:=2048 image_height:=1536 framerate:=30 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=2592 image_height:=1944 framerate:=20 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=3840 image_height:=2160 framerate:=25 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=3264 image_height:=2448 framerate:=20 video_device:=/dev/video2
# roslaunch picam_ros picam_read.launch show_img:=true image_width:=3840 image_height:=2880 framerate:=15 video_device:=/dev/video2
