#include "ros/ros.h"

#include <boost/format.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <cloud_tracker_node.h>

using namespace pcl::tracking;


CloudTrackerNode::CloudTrackerNode(ros::NodeHandle nh, std::string input_cloud_topic_) :
  nh_(nh),
  reconfigure_server_(nh),
  downsampling_grid_size(0.002),
  trackedTransformMsg(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
  tracker_initialized(false),
  has_received_cloud(false),
  track_cloud_service_topic("/trackObject"),
  input_cloud_topic(input_cloud_topic_)
{
  target_cloud.reset(new Cloud());

  trackCloudService = nh_.advertiseService(track_cloud_service_topic, &CloudTrackerNode::initializeTracker, this);

  ROS_INFO_STREAM("Setting up pointcloud subscriber for topic: " << input_cloud_topic_ << std::endl);
  input_cloud_subscriber = nh_.subscribe(input_cloud_topic, 1, &CloudTrackerNode::pointcloud_cb, this);
  reconfigure_server_.setCallback(boost::bind(&CloudTrackerNode::reconfigure_cb, this, _1, _2));

}


bool CloudTrackerNode::initializeTracker(cloud_tracker::TrackCloud::Request  &req, cloud_tracker::TrackCloud::Response &res)
{
  ROS_INFO("Received New TrackObject Service Request");
  ROS_INFO_STREAM("InitialPose: " << req.initialObjectPose << std::endl;);

  //convert cloud and transform into camera frame to match the reference frame of the filtered pc, which we are tracking in.
  tf::TransformListener listener;
  tf::StampedTransform transformMsg;
  Eigen::Matrix4f transformEigen = Eigen::Matrix4f::Identity ();
  listener.waitForTransform(camera_frame_id,req.initialObjectPose.header.frame_id,
                            ros::Time::now(), ros::Duration(3.0));
  listener.lookupTransform(camera_frame_id,req.initialObjectPose.header.frame_id,
                           ros::Time(0), transformMsg);

  pcl_ros::transformAsMatrix(transformMsg, transformEigen);

  pcl::PCLPointCloud2 original_pc2;
  pcl_conversions::toPCL(req.objectCloud, original_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(original_pc2, *original_pc);

  target_cloud = original_pc;
  tracked_object_frame_id = "/detected_object";

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

  Eigen::Affine3f trackedTransform;
  trackedTransform.matrix() = transformEigen;
  CloudPtr transed_ref (new Cloud);
  CloudPtr transed_ref_downsampled (new Cloud);

  gridSampleApprox (original_pc, *transed_ref_downsampled, downsampling_grid_size_);

  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trackedTransform);

  tracker_initialized = true;
}


void CloudTrackerNode::mainloop()
{
  ros::Rate loop_rate(40);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (tracker_initialized)
      {
        if(!has_received_cloud)
          {
            ROS_WARN_STREAM("Tracker Initialized, but we have never received a pointcloud to track the object on the topic: "  << input_cloud_topic << std::endl);
          }
        br.sendTransform(tf::StampedTransform(trackedTransformMsg, ros::Time::now(), camera_frame_id, tracked_object_frame_id));
      }
  }
}


void CloudTrackerNode::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  has_received_cloud = true;
  camera_frame_id = cloud_msg->header.frame_id;

  if (! tracker_initialized)
    return;

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

