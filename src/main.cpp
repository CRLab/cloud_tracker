#include <ros/ros.h>
#include <cloud_tracker_node.h>

int main(int argc, char** argv) {

  ros::init(argc, argv,"cloud_tracker");

  CloudTrackerNode *node = new CloudTrackerNode();
  node->mainloop();

  return 0;
}
