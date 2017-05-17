// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

namespace PointCloudStitching {

class PointCloudStitching
{
protected:
  //! ROS node handle
  ros::NodeHandle& nh_;

  //! ROS pointcloud subscriber
  ros::Subscriber firstCloudSubscriber_;
  ros::Subscriber secondCloudSubscriber_;

  //! ROS pointcloud publisher
  ros::Publisher pointCloudPublisher_;

  //! tf listener
  tf::TransformListener tfListener_;
  //! Map frame id
  std::string targetFrameId_;

 //! PointClouds
  sensor_msgs::PointCloud2 firstCloud_;
  sensor_msgs::PointCloud2 secondCloud_;

  //! Parameters
  bool publishOnlyFirst_;

public:
  PointCloudStitching(ros::NodeHandle& nh);
  virtual ~PointCloudStitching();

protected:
  void pointCloudFirstCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void pointCloudSecondCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};


} // pointcloud_stitching
