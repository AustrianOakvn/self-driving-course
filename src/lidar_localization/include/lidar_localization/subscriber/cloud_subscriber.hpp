#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_ 

#include <deque>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"


namespace lidar_localization{
  class CloudSubscriber{
    public:
      CloudSubscriber(std::shared_ptr<rclcpp::Node> node,
                      const std::string & topic_name, 
                      size_t buff_size);
      CloudSubscriber() = default;
      void ParseData(std::deque<CloudData>& deque_cloud_data);
    private:
      void msg_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& cloud_msg_ptr);

    private:
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
      std::deque<CloudData> new_cloud_data_;

      std::mutex buff_mutex_;
  };
}

#endif

