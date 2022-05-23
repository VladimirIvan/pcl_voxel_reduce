#pragma once

#include <string>

#include <pcl/common/common.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudProcessor
{
public:
    PointCloudProcessor();

protected:
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher cloud_pub_;
    tf::TransformListener tf_listener_;

    std::string world_name_;
    sensor_msgs::PointCloud2 cloud_;
    float grid_resolution_;
    float box_min_x_;
    float box_min_y_;
    float box_min_z_;
    float box_max_x_;
    float box_max_y_;
    float box_max_z_;
    int min_points_per_voxel_;
};
