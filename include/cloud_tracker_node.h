#ifndef __CLOUD_TRACKER_NODE_H
#define __CLOUD_TRACKER_NODE_H

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cloud_tracker/CloudTrackerConfig.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/tracker.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

typedef pcl::PointXYZRGB RefPointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;


class CloudTrackerNode {

 public:
  CloudTrackerNode(ros::NodeHandle nh = ros::NodeHandle("~"));
  ~CloudTrackerNode();
  void mainloop();

 private:

  //dyanmic reconfigure callback
  void reconfigure_cb(cloud_tracker::CloudTrackerConfig &config, uint32_t level);

  //new pointcloud received
  void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &points_msg);

  void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size);
  void setObjectPose(Eigen::Affine3f &transformation);

  // ROS Structures
  ros::NodeHandle &nh_;

  //topic in which we listen for filtered pointclouds from
  //this is grabbed from the ros param server.
  ros::Subscriber input_cloud_subscriber;
  std::string input_cloud_topic;

  std::string camera_frame_id; //camera_rgb_optical_frame
  std::string tracked_object_frame_id; //tracked_object_frame_id

  CloudPtr target_cloud;
  CloudPtr cloud_pass_downsampled_;

  boost::shared_ptr<ParticleFilter> tracker_;
  double downsampling_grid_size_;

  tf::TransformBroadcaster br;
  tf::Transform trackedTransformMsg;

  float downsampling_grid_size;

  // ROS Dynamic Reconfigure
  dynamic_reconfigure::Server<cloud_tracker::CloudTrackerConfig> reconfigure_server_;
};

#endif // ifndef __CLOUD_TRACKER_NODE_H
