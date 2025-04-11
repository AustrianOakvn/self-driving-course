#include <lidar_localization/subscriber/gnss_subscriber.hpp>
#include "glog/logging.h"
#include <iostream>

namespace lidar_localization {
  GNSSSubscriber::GNSSSubscriber(std::shared_ptr<rclcpp::Node> node,
                                    const std::string &topic_name, 
                                    size_t buff_size):node_(node)
  {
    subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(buff_size)),
        std::bind(&GNSSSubscriber::msg_callback, this, std::placeholders::_1)
        );
  }

  void GNSSSubscriber::msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_ptr)
  {
    std::lock_guard<std::mutex> lock(buff_mutex_);

    GNSSData gnss_data;
    gnss_data.time = rclcpp::Time(nav_sat_fix_ptr->header.stamp).seconds();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);
  }

  void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff){
    std::lock_guard<std::mutex> lock(buff_mutex_);

    if (!new_gnss_data_.empty()){
      std::cout << "insert gnss data to buff" << std::endl;
      gnss_data_buff.insert(gnss_data_buff.end(),
                            new_gnss_data_.begin(),
                            new_gnss_data_.end());
      new_gnss_data_.clear();
    }
  }
  
}




