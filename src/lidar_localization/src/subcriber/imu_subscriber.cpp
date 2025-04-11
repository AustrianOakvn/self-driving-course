#include "lidar_localization/subscriber/imu_subscriber.hpp"



namespace lidar_localization{
  IMUSubscriber::IMUSubscriber(std::shared_ptr<rclcpp::Node> node,
                                const std::string & topic_name,
                                size_t buff_size):node_(node)
  {
    subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(buff_size)),
        std::bind(&IMUSubscriber::msg_callback, this, std::placeholders::_1)
        );
  }

  void IMUSubscriber::msg_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr)
  {
    std::lock_guard<std::mutex> lock(buff_mutex_);

    IMUData imu_data;
    imu_data.time = rclcpp::Time(imu_msg_ptr->header.stamp).seconds();

    imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;


    imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;


    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;
    imu_data.orientation.w = imu_msg_ptr->orientation.w;

    new_imu_data_.push_back(imu_data);
    buff_mutex_.unlock();
  }

  void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff)
  {
    std::lock_guard<std::mutex> lock(buff_mutex_);
    if (!new_imu_data_.empty()){
      imu_data_buff.insert(imu_data_buff.end(),
                          new_imu_data_.begin(),
                          new_imu_data_.end());
      new_imu_data_.clear();
    }
  }
}
