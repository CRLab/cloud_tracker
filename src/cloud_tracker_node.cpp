#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <tf/transform_broadcaster.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <cloud_tracker_node.h>

using namespace pcl::tracking;


CloudTrackerNode::CloudTrackerNode(ros::NodeHandle nh) :
  nh_(nh),
  reconfigure_server_(nh),
  downsampling_grid_size(0.002),
  trackedTransformMsg(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
  tracked_object_frame_id("/detected_object")
{

  target_cloud.reset(new Cloud());
  if(pcl::io::loadPCDFile ("/home/jvarley/pitcher_transformed2.pcd", *target_cloud) == -1){
    std::cout << "pcd file not found" << std::endl;
    exit(-1);
  }

  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;

  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

  ParticleT bin_size;
  bin_size.x = 0.1f;
  bin_size.y = 0.1f;
  bin_size.z = 0.1f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;

  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (100);
  tracker->setDelta (0.99);
  tracker->setEpsilon (0.2);
  tracker->setBinSize (bin_size);

  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (60);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setUseNormal (false);

  //Setup coherence object for tracking
  ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
    (new ApproxNearestPairPointCloudCoherence<RefPointType> ());

  boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
    = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
  coherence->addPointCoherence (distance_coherence);

  boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance (0.01);

  tracker_->setCloudCoherence (coherence);

  //prepare the model of tracker's target
  Eigen::Vector4f target_origin;
  Eigen::Affine3f trackedTransform = Eigen::Affine3f::Identity ();
  CloudPtr transed_ref (new Cloud);
  CloudPtr transed_ref_downsampled (new Cloud);

  pcl::compute3DCentroid<RefPointType> (*target_cloud, target_origin);
  trackedTransform.translation ().matrix () = Eigen::Vector3f (target_origin[0], target_origin[1], target_origin[2]);
  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trackedTransform.inverse());
  gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trackedTransform);


  input_cloud_subscriber = nh_.subscribe("/filtered_pc", 1, &CloudTrackerNode::pointcloud_cb, this);
  reconfigure_server_.setCallback(boost::bind(&CloudTrackerNode::reconfigure_cb, this, _1, _2));

}

void CloudTrackerNode::mainloop()
{
  ros::Rate loop_rate(40);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    br.sendTransform(tf::StampedTransform(trackedTransformMsg, ros::Time::now(), camera_frame_id, tracked_object_frame_id));
  }
}

void CloudTrackerNode::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  camera_frame_id = cloud_msg->header.frame_id;

  //Convert to pcl pointcloud2
  pcl::PCLPointCloud2 original_pc2;
  pcl_conversions::toPCL(*cloud_msg, original_pc2);

  //Convert to pcl pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(original_pc2, *original_pc);

  //downsample observed cloud for spped
  cloud_pass_downsampled_.reset (new Cloud);
  gridSampleApprox (original_pc, *cloud_pass_downsampled_, downsampling_grid_size_);

  //give new observation to tracker
  tracker_->setInputCloud (cloud_pass_downsampled_);
  tracker_->compute ();

  //update current tracked object transform with tracker current estimate
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
  setObjectPose(transformation);
}

//update variables from config server
void CloudTrackerNode::reconfigure_cb(cloud_tracker::CloudTrackerConfig &config, uint32_t level)
{
    downsampling_grid_size_ = config.downsampling_grid_size;
}




void CloudTrackerNode::setObjectPose(Eigen::Affine3f &transformation)
{
    Eigen::Matrix3f rotationMatrix=transformation.rotation();
    Eigen::Quaternionf quaternion(rotationMatrix);

    float x = transformation.translation().x();
    float y = transformation.translation().y();
    float z = transformation.translation().z();

    float qx=quaternion.x();
    float qy=quaternion.y();
    float qz=quaternion.z();
    float qw=quaternion.w();

    trackedTransformMsg = tf::Transform(tf::Quaternion(qx,qy,qz,qw),
                                      tf::Vector3(x,y,z));

}

void CloudTrackerNode::gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

