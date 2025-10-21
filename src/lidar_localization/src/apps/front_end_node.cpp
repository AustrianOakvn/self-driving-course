#include <rclcpp/rclcpp.hpp>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]){
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;


  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("front_end_node");

  std::string cloud_topic, odom_topic;
  //Declare param with default values
  // declare and get params needs to be coupled with node
  node->declare_parameter<std::string>("cloud_topic", "/synced_cloud");
  node->declare_parameter<std::string>("odom_topic", "/laser_odom");
  
  std::string cloud_topic = node->get_parameter("cloud_topic").as_string();
  std::string odom_topic = node->get_parameter("odom_topic").as_string();

  std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(node,
                          cloud_topic,
                          odom_topic,
                          "odom_ppath");

  rclcpp::Rate rate(100);
  while(rclcpp::ok()){
    rclcpp::spin_some(node);
    front_end_flow_ptr->Run();
    rate.sleep();
  }

  return 0;
}
