#include "lidar_localization/tf_listener/tf_listener.hpp"
#include <Eigen/Geometry>
#include <rclcpp/qos.hpp>

namespace lidar_localization {
  TFListener::TFListener(std::shared_ptr<rclcpp::Node> node,
                        std::string base_frame_id,
                        std::string child_frame_id):node_(node), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id)
  {
    //auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    //qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  bool TFListener::LookupData(Eigen::Matrix4f transform_matrix)
  {
    try{
      geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(base_frame_id_, child_frame_id_,
                                    rclcpp::Time(0));
      return TransformToMatrix(transform_stamped, 
                              transform_matrix);
    }catch (const tf2::TransformException& ex){
      RCLCPP_WARN(node_->get_logger(), "Could not get transform: %s", ex.what());
      return false;
    }
  }

  bool TFListener::TransformToMatrix(const geometry_msgs::msg::TransformStamped& transform,
                                      Eigen::Matrix4f& transform_matrix)
  {
    Eigen::Translation3f tl_btol(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
        );
    Eigen::Quaternionf q(
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z
        );
    Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

    transform_matrix.setIdentity();
    transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
    transform_matrix.block<3, 1>(0, 3) = tl_btol.vector();

    return true;
  }

}
