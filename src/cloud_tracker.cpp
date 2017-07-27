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

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <tf/transform_broadcaster.h>
#include <pcl/filters/approximate_voxel_grid.h>

ros::Publisher filtered_pc_pub;
tf::TransformListener *tf_listener;
ros::NodeHandle* n;

float x_clip_min_;
float x_clip_max_;
float y_clip_min_;
float y_clip_max_;
float z_clip_min_;
float z_clip_max_;

std::string filtered_frame_id;
std::string observed_frame_id;
std::string input_pc_topic;
std::string output_pc_topic;


using namespace pcl::tracking;

typedef pcl::PointXYZRGB RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

CloudPtr target_cloud;
CloudPtr cloud_pass_downsampled_;

boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;


double x,y,z;
double quaternion_x,quaternion_y,quaternion_z,quaternion_w;


void publishObjectPose(Eigen::Affine3f &transformation)
{
    Eigen::Matrix3f rotationMatrix=transformation.rotation();

    x=transformation.translation().x();
    y=transformation.translation().y();
    z=transformation.translation().z();



    Eigen::Quaternionf quaternion(rotationMatrix);
    quaternion_x=quaternion.x();
    quaternion_y=quaternion.y();
    quaternion_z=quaternion.z();
    quaternion_w=quaternion.w();

/*
    std::cout<<"x: " << x <<std::endl;
    std::cout<<"y: " << y <<std::endl;
    std::cout<<"z: " << z <<std::endl;
//    std::cout<<"roll is:" <<atan2( rotationMatrix(2,1),rotationMatrix(2,2) ) <<std::endl;
//    std::cout<<"pitch is:" <<atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  ) <<std::endl;
//    std::cout<<"yaw is:" <<atan2( rotationMatrix(1,0),rotationMatrix(0,0) ) <<std::endl;



    std::cout<<"quaternion_x is:" <<quaternion_x <<std::endl;
    std::cout<<"quaternion_y is:" <<quaternion_y <<std::endl;
    std::cout<<"quaternion_z is:" <<quaternion_z <<std::endl;
    std::cout<<"quaternion_w is:" <<quaternion_w <<std::endl;

*/

}

void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void filterCallback(const sensor_msgs::PointCloud2ConstPtr& sensor_message_pc)
{

  ROS_INFO("Filtering PointCloud Callback");
  pcl::PCLPointCloud2 original_pc2;
  pcl_conversions::toPCL(*sensor_message_pc, original_pc2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(original_pc2, *original_pc);

  cloud_pass_downsampled_.reset (new Cloud);
  gridSampleApprox (original_pc, *cloud_pass_downsampled_, downsampling_grid_size_);

  if(counter < 10){
          counter++;
    }else{
  //Track the object
  std::cout << "Input PC size: " << original_pc->size() << std::endl;
  tracker_->setInputCloud (cloud_pass_downsampled_);
  tracker_->compute ();
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
  publishObjectPose(transformation);

  std::cout << "New translation " << transformation.translation() << std::endl;
  std::cout << "New rotation " << transformation.rotation() << std::endl;
    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker");
  ros::NodeHandle n_;
  tf::TransformListener tfl;
  tf_listener = &tfl;
  n = &n_;

  target_cloud.reset(new Cloud());
  //if(pcl::io::loadPCDFile ("/home/jvarley/tactile_completion_results/rubbermaid_ice_guard_pitcher_blue_20_y17_m07_d21_h12_m47_s20/partial_cf.pcd", *target_cloud) == -1){
  if(pcl::io::loadPCDFile ("/home/jvarley/pitcher_transformed2.pcd", *target_cloud) == -1){
    std::cout << "pcd file not found" << std::endl;
    exit(-1);
  }

  counter = 0;

  //Set parameters
  new_cloud_  = false;
  downsampling_grid_size_ =  0.002;

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
  Eigen::Vector4f c;
  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  CloudPtr transed_ref (new Cloud);
  CloudPtr transed_ref_downsampled (new Cloud);

  pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
  gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trans);

  std::cout << "REFERNCE PC SIZE: " << transed_ref->size() << std::endl;

  ros::Subscriber original_pc_sub = n_.subscribe("/filtered_pc", 1, filterCallback);
  //ros::Subscriber original_pc_sub = n_.subscribe("/camera/depth_registered/points", 1, filterCallback);


  tf::TransformBroadcaster br;
  tf::Transform transform;

  x=0;
  y=0;
  z=0;

  quaternion_x=0;
  quaternion_y=0;
  quaternion_z=0;
  quaternion_w=1;


     ros::Rate loop_rate(40);
     while(ros::ok())
     {
         ros::spinOnce();
         loop_rate.sleep();
         transform.setOrigin( tf::Vector3(x, y, z) );
         transform.setRotation( tf::Quaternion( quaternion_x,quaternion_y,quaternion_z,quaternion_w  ) );
         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_rgb_optical_frame", "object"));

     }
  return 0;
}

