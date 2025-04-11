#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "glog/logging.h"
#include <iostream>

namespace lidar_localization {
  CloudSubscriber::CloudSubscriber(std::shared_ptr<rclcpp::Node> node,
                                    const std::string &topic_name, 
                                    size_t buff_size):node_(node)
  {
    subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(buff_size)),
        std::bind(&CloudSubscriber::msg_callback, this, std::placeholders::_1)
        );
  }

  void CloudSubscriber::msg_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& cloud_msg_ptr)
  {
    std::lock_guard<std::mutex> lock(buff_mutex_);
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.sec + cloud_msg_ptr->header.stamp.nanosec * 1e-9;

    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));
    new_cloud_data_.push_back(cloud_data);
  }

  void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff){
    std::lock_guard<std::mutex> lock(buff_mutex_);
    if (!new_cloud_data_.empty()){
      std::cout << "Insert pointcloud to buffer" << std::endl;
      cloud_data_buff.insert(cloud_data_buff.end(), 
                              new_cloud_data_.begin(),
                              new_cloud_data_.end());
      new_cloud_data_.clear();
    }
  }
}


