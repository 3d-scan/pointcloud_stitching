// pointcloud stitching
#include "pointcloud_stitching/PointCloudStitching.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_stitching");
  ros::NodeHandle nodeHandle("~");

  try {
    PointCloudStitching::PointCloudStitching node(nodeHandle);
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  return 0;
}
