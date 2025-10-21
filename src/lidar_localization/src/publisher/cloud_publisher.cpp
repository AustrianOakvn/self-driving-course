#include "lidar_localization/publisher/cloud_publisher.hpp"


namespace lidar_localization {
  CloudPublisher::CloudPublisher(std::shared_ptr<rclcpp::Node> node,
                                  const std::string& topic_name,
                                  const std::string& frame_id,
                                  size_t buff_size):node_(node), frame_id_(frame_id)
  {
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(buff_size))
        );
  }

  void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input,
                                double time)
  {
    rclcpp::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
  }

  void CloudPublisher::Publish(CloudData::CLOUD_PTR& cloud_ptr_input)
  {
    rclcpp::Time time = rclcpp::Clock().now();
    PublishData(cloud_ptr_input, time);
  }

  void CloudPublisher::PublishData(CloudData::CLOUD_PTR& cloud_ptr_input,
                                    rclcpp::Time time)
  {
    auto cloud_ptr_output = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_->publish(*cloud_ptr_output);
  }

  bool CloudPublisher::HasSubscribers(){
    return publisher_->get_subscription_count() != 0;
  }
}
