#ifndef LIDAR_LOCALIZATION_TF_LISTENER_HPP_
#define LIDAR_LOCALIZATION_TF_LISTENER_HPP_

#include <string>
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h> 
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace lidar_localization {
  class TFListener {
    public:
      TFListener(std::shared_ptr<rclcpp::Node> node,
                  std::string base_frame_id, 
                  std::string child_frame_id);
      TFListener() = default;

      bool LookupData(Eigen::Matrix4f transform_matrix);

    private:
      bool TransformToMatrix(const geometry_msgs::msg::TransformStamped& transform,
                              Eigen::Matrix4f& transform_matrix);
    private:
      std::shared_ptr<rclcpp::Node> node_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      std::string base_frame_id_;
      std::string child_frame_id_;
  };
}

#endif
