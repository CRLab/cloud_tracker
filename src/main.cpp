#include <ros/ros.h>
#include <cloud_tracker_node.h>

int main(int argc, char** argv) {

  ros::init(argc, argv,"cloud_tracker");

  ros::NodeHandle nh = ros::NodeHandle("~");

  if(argc != 2)
    {
      ROS_INFO("Need to pass the pointcloud topic to this node");
      ROS_INFO("Ex rosrun cloud_tracker cloud_tracker /pc_filter");
      return 0;
    }

  std::string input_cloud_topic_ = argv[1];

  ROS_INFO_STREAM("Starting  cloud_tracker node to listen to " << input_cloud_topic_ << std::endl);

  CloudTrackerNode *node = new CloudTrackerNode(nh, input_cloud_topic_);
  node->mainloop();

  return 0;
}
