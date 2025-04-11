#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_


#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "lidar_localization/sensor_data/gnss_data.hpp"

namespace lidar_localization{
  class GNSSSubscriber {
    public:
      GNSSSubscriber(std::shared_ptr<rclcpp::Node> node,
                      const std::string &topic_name, 
                      size_t buff_size);
      void ParseData(std::deque<GNSSData>& deque_gnss_data);
    private:
      void msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_ptr);
    private:
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;
      std::deque<GNSSData> new_gnss_data_;
      std::mutex buff_mutex_;
  };
}

#endif

