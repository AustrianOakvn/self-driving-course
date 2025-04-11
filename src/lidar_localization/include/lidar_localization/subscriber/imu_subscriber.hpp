#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
  class IMUSubscriber {
    public: 
      IMUSubscriber(std::shared_ptr<rclcpp::Node> node,
                    const std::string &topic_name,
                    size_t buff_size);
      void ParseData(std::deque<IMUData>& deque_imu_data);
    private:
      void msg_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);
    private:
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
      std::deque<IMUData> new_imu_data_;
      std::mutex buff_mutex_;
  };
}



#endif
