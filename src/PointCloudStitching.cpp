// param io
#include <param_io/get_param.hpp>

// pointcloud stitching
#include "pointcloud_stitching/PointCloudStitching.hpp"

// pcl_ros
#include <pcl_ros/transforms.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>

// ros
#include <tf_conversions/tf_eigen.h>

// c++
#include <fstream>

// c++
#include <string>
#include <map>
#include <vector>

// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace PointCloudStitching {

using namespace param_io;

PointCloudStitching::PointCloudStitching(ros::NodeHandle& nh)
    : nh_(nh)
{
  firstCloudSubscriber_ = nh_.subscribe(
              param<std::string>(nh_, "subscribers/firstInput/topic", "/velodyne_points"),
              param<int>(nh_, "subscribers/firstInput/queue_size", 1),
              &PointCloudStitching::pointCloudFirstCallback, this);

  secondCloudSubscriber_ = nh.subscribe(
                param<std::string>(nh_, "subscribers/secondInput/topic", "/velodyne_points"),
                param<int>(nh_, "subscribers/secondInput/queue_size", 1),
                &PointCloudStitching::pointCloudSecondCallback, this);

  pointCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
              param<std::string>(nh_, "publishers/stitchedCloud/topic", "/stitched_velodyne_points"),
              param<int>(nh_, "publishers/stitchedCloud/queue_size", 1),
              param<bool>(nh_, "publishers/stitchedCloud/latch", true));

  targetFrameId_ = param<std::string>(nh_, "targetFrameId_", "odom");

  publishOnlyFirst_ = param<bool>(nh_, "publishOnlyFirst_", false);
}

PointCloudStitching::~PointCloudStitching()
{

}

void PointCloudStitching::pointCloudFirstCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    if(!tfListener_.waitForTransform(
                cloud->header.frame_id,
                targetFrameId_,
                cloud->header.stamp,
                ros::Duration(1.0)))
    {
        ROS_ERROR_STREAM("PointCloudStitching: Could not get transformation from '" << cloud->header.frame_id << "' to '" << targetFrameId_ << "'.");
        return;
    }

    if (!pcl_ros::transformPointCloud(targetFrameId_, *cloud, firstCloud_, tfListener_)) {
        ROS_ERROR_STREAM("PointCloudStitching: Could not transform from '" << cloud->header.frame_id << "' to '" << targetFrameId_ << "'.");
    }

    sensor_msgs::PointCloud2 stitchedCloud;
    pcl::concatenatePointCloud(firstCloud_, secondCloud_, stitchedCloud);

    if (publishOnlyFirst_)
    {
      pointCloudPublisher_.publish(firstCloud_);
    }
    else
    {
      pointCloudPublisher_.publish(stitchedCloud);
    }

    return;
}

void PointCloudStitching::pointCloudSecondCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    if(!tfListener_.waitForTransform(
                cloud->header.frame_id,
                targetFrameId_,
                cloud->header.stamp,
                ros::Duration(1.0)))
    {
        ROS_ERROR_STREAM("PointCloudStitching: Could not get transformation from '" << cloud->header.frame_id << "' to '" << targetFrameId_ << "'.");
        return;
    }

    if (!pcl_ros::transformPointCloud(targetFrameId_, *cloud, secondCloud_, tfListener_)) {
        ROS_ERROR_STREAM("PointCloudStitching: Could not transform from '" << cloud->header.frame_id << "' to '" << targetFrameId_ << "'.");
    }
    return;
}


} // pointcloud_stitching
