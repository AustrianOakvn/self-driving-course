
#include "lidar_localization/publisher/odometry_publisher.hpp"



namespace lidar_localization{
  OdometryPublisher::OdometryPublisher(std::shared_ptr<rclcpp::Node> node,
                      const std::string& topic_name,
                      const std::string& path_topic_name,
                      const std::string& base_frame_id,
                      const std::string& child_frame_id,
                      size_t buff_size):node_(node)
  {
    publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(buff_size))
        );
    path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
        path_topic_name,
        rclcpp::QoS(rclcpp::KeepLast(buff_size))
        );
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;

    path_.header.frame_id = base_frame_id;
  }

  void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix, 
                                  double time)
  {
    rclcpp::Time ros_time(time);
    PublishData(transform_matrix, ros_time);
  }

  void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix)
  {
    rclcpp::Time time = rclcpp::Clock().now();
    PublishData(transform_matrix, time);
  }


  void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix, 
                                      rclcpp::Time time)
  {
    odometry_.header.stamp = time;
    odometry_.pose.pose.position.x = transform_matrix(0, 3);
    odometry_.pose.pose.position.y = transform_matrix(1, 3);
    odometry_.pose.pose.position.z = transform_matrix(2, 3);
    
    Eigen::Quaternionf q;
    q = transform_matrix.block<3, 3>(0, 0);

    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_->publish(odometry_);

    // Add to the path (for visualization)
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = time;
    pose_stamped.header.frame_id = odometry_.header.frame_id;
    pose_stamped.pose = odometry_.pose.pose;

    path_.poses.push_back(pose_stamped);
    path_.header.stamp = time;

    path_publisher_->publish(path_);
  }

  bool OdometryPublisher::HasSubscribers(){
    return publisher_->get_subscription_count() != 0;
  }
}
