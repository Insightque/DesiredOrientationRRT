#include <ros/ros.h>
#include "decision_maker.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_maker_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  planner::DecisionMaker dm(nh, priv_nh);

  std::cout << "[+] decision maker has started..." << std::endl;

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  /* ros::MultiThreadedSpinner spinner(4); // Use 4 threads */
  /* spinner.spin(); // spin() will not return until the node has been shutdown */

  // ros::spin();

  return 0;
}
