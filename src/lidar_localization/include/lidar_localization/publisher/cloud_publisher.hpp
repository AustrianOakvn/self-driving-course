#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "lidar_localization/sensor_data/cloud_data.hpp"


namespace lidar_localization {
  class CloudPublisher {
    public:
      CloudPublisher(std::shared_ptr<rclcpp::Node> node,
                      const std::string& topic_name,
                      const std::string& frame_id,
                      size_t buff_size);
      CloudPublisher() = default;

      void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time);
      void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);


      bool HasSubscribers();
    private:
      void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, rclcpp::Time time);
    private:
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
      std::string frame_id_;
  };
}


#endif
