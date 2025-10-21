#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>


namespace lidar_localization{ 
  class OdometryPublisher {
    public:
      OdometryPublisher(std::shared_ptr<rclcpp::Node> node,
                        const std::string& topic_name,
                        const std::string& path_topic_name,
                        const std::string& base_frame_id,
                        const std::string& child_frame_id,
                        size_t buff_size);
      OdometryPublisher() = default;

      void Publish(const Eigen::Matrix4f& transform_matrix, double time);
      void Publish(const Eigen::Matrix4f& transform_matrix);

      bool HasSubscribers();
    private:
      void PublishData(const Eigen::Matrix4f& transform_matrix, rclcpp::Time time);
    private:
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
      nav_msgs::msg::Odometry odometry_;
      nav_msgs::msg::Path path_;
  };
}

#endif
