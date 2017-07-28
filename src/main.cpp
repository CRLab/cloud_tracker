#include <ros/ros.h>
#include <cloud_tracker_node.h>

int main(int argc, char** argv) {

  ros::init(argc, argv,"cloud_tracker");

  ros::NodeHandle nh = ros::NodeHandle("~");


  std::string input_cloud_topic_;

  for (int i = 0; i < argc - 1; i++)
  {
    if (!strcmp(argv[i], "input_cloud_topic"))
    {
      input_cloud_topic_ = argv[i + 1];
    }
  }

  CloudTrackerNode *node = new CloudTrackerNode(nh, input_cloud_topic_);
  node->mainloop();

  return 0;
}
