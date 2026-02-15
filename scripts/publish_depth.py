#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def depth_to_point_cloud(depth_img, fx, fy, cx, cy, scale=1000.0):
    """
    将深度图转为点云
    参数:
        depth_img: 输入深度图 (16UC1)
        fx, fy, cx, cy: 相机内参
        scale: 深度缩放因子 (默认1m=1000)
    返回:
        点云数组 (N x 3)
    """
    height, width = depth_img.shape
    i, j = np.meshgrid(np.arange(width), np.arange(height), indexing='xy')

    z = depth_img.astype(np.float32) / scale
    x = (i - cx) * z / fx
    y = (j - cy) * z / fy

    valid = z > 0
    x = x[valid]
    y = y[valid]
    z = z[valid]

    points = np.stack((x, y, z), axis=-1)
    return points

def publish_point_cloud(points, frame_id="map"):
    """
    将 (N x 3) 点云数组发布为 PointCloud2 消息
    """
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    cloud_msg = pc2.create_cloud_xyz32(header, points)
    return cloud_msg

if __name__ == '__main__':
    rospy.init_node('depth_to_pointcloud_publisher')
    pub = rospy.Publisher('/depth_pointcloud', PointCloud2, queue_size=1)

    # 相机内参（需替换为你的相机参数）
    # fx = 525.0
    # fy = 525.0
    # cx = 319.5
    # cy = 239.5
    # scale = 1.0  # mm 到 m

    width = 1280
    height = 960
    fx = width / 2.0
    fy = width / 2.0
    cx = (width - 1.0) / 2.0
    cy = (height - 1.0) / 2.0
    scale = 65535.0  # mm 到 m

    # 读取深度图路径（替换为你的图像路径）
    depth_image_path = "/home/robotlab/dataset/MySimDataset/gazebo_dataset_2/depth/8185.589000000.png"
    depth_image_path = "/home/robotlab/dataset/MySimDataset/GroundObjects/depth/000000.png"
    depth_image_path = "/home/robotlab/dataset/MySimDataset/gazebo_dataset_4/depth/8130.657000.tiff"
    depth_image_path = "/home/robotlab/dataset/debug/depth/depth000000.png"
    depth = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

    if depth is None:
        rospy.logerr("❌ 无法读取深度图像: %s" % depth_image_path)
        exit(1)

    rospy.loginfo("✅ 成功读取深度图像，开始发布点云...")

    rate = rospy.Rate(1)  # 1Hz 发布一次
    while not rospy.is_shutdown():
        points = depth_to_point_cloud(depth, fx, fy, cx, cy, scale)
        cloud_msg = publish_point_cloud(points)
        pub.publish(cloud_msg)
        rate.sleep()