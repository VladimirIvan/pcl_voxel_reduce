#include <pcl_voxel_reduce/pcl_voxel.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/crop_box.h>

void PointCloudProcessor::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    if ((cloud->width * cloud->height) == 0)
    {
        return;  //return if the cloud is not dense!
    }
    {
        // Concatenate
        sensor_msgs::PointCloud2 tmp;
        pcl_ros::transformPointCloud(world_name_, *cloud, tmp, tf_listener_);

        // Downsample
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr cloud_reduced(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(tmp, *pcl_cloud);

        pcl::CropBox<pcl::PCLPointCloud2> crop;
        crop.setInputCloud(pcl_cloud);
        Eigen::Vector4f box_min(box_min_x_, box_min_y_, box_min_z_, 1.f);
        Eigen::Vector4f box_max(box_max_x_, box_max_y_, box_max_z_, 1.f);
        crop.setMax(box_max);
        crop.setMin(box_min);
        crop.filter(*cloud_filtered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ> cloud_reduced_pcl;
        pcl::fromPCLPointCloud2( *cloud_filtered, *cloud_pcl);

        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setMinimumPointsNumberPerVoxel(static_cast<unsigned int>(min_points_per_voxel_));
        grid.setInputCloud(cloud_pcl);
        grid.setLeafSize(grid_resolution_, grid_resolution_, grid_resolution_);
        grid.filter(cloud_reduced_pcl);
        pcl::toPCLPointCloud2(cloud_reduced_pcl, *cloud_reduced);

        // Publish
        pcl_conversions::fromPCL(*cloud_reduced, tmp);
        tmp.header.frame_id = world_name_;
        cloud_pub_.publish(tmp);
    }
}

PointCloudProcessor::PointCloudProcessor() : nh_("~"), tf_listener_()
{
    nh_.param<std::string>("frame", world_name_, "base_link");
    nh_.param<float>("resolution", grid_resolution_, 0.1f);
    nh_.param<int>("min_points", min_points_per_voxel_, 0);
    nh_.param<float>("min_x", box_min_x_, -10.0f);
    nh_.param<float>("min_y", box_min_y_, -10.0f);
    nh_.param<float>("min_z", box_min_z_, -10.0f);
    nh_.param<float>("max_x", box_max_x_,  10.0f);
    nh_.param<float>("max_y", box_max_y_,  10.0f);
    nh_.param<float>("max_z", box_max_z_,  10.0f);

    sub_ = nh_.subscribe("input", 30, &PointCloudProcessor::cloud_cb, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 30, true);
    ROS_INFO_STREAM("Point cloud processor started ...");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_processor");
    PointCloudProcessor pc;
    ros::spin();
    return 0;
}
