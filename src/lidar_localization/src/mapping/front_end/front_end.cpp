#include "lidar_localization/mapping/front_end/front_end.hpp"
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/models/registration/ndt_registration.hpp"
//#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"


#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

namespace lidar_localization {
  FrontEnd::FrontEnd():local_map_ptr_(new CloudData::CLOUD())
  {
    InitWithConfig();
  }

  bool FrontEnd::InitWithConfig(){
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "" << std::endl;
    InitParam(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
  }

  bool FrontEnd::InitParam(const YAML::Node& config_node){
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;
  }

  bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,
                                  const YAML::Node& config_node){
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "Selected pointcloud matching mathod" << registration_method << std::endl;

    if (registration_method == "NDT"){
      registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }else {
      LOG(ERROR) << "Not Found" << registration_method << "matching method";
      return false;
    }
    return true;
  }

  bool FrontEnd::InitFilter(std::string filter_user,
                            std::shared_ptr<CloudFilterInterface>& filter_ptr,
                            const YAML::Node& config_node)
  {
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "Front end" << filter_user << "method: " << filter_method << std::endl;

    if (filter_method == "voxel_filter"){
      filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    }else if (filter_method == "no_filter"){
      filter_ptr = std::make_shared<NoFilter>();
    }else {
      LOG(ERROR) << "Not found" << filter_user << filter_method << "method";
      return false;
    }
    return true;
  }

  bool FrontEnd::Update(const CloudData& cloud_data,
                        Eigen::Matrix4f& cloud_pose){
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, 
                                *current_frame_.cloud_data.cloud_ptr,
                                indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Indentity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    if (local_map_frames_.size() == 0){
      current_frame_.pose = init_pose_;
      UpdateWithNewFrame(current_frame_);
      cloud_pose = current_frame_.pose;
      return true;
    }

    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, 
                                predict_pose,
                                result_cloud_ptr,
                                current_frame_.pose);
    cloud_pose = current_frame_.pose;

    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
        fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
        fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) > key_frame_distance_) 
        {
          UpdateWithNewFrame(current_frame_);
          last_key_frame_pose = current_frame_.pose;
        }
    return true;
  }

  bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose){
    init_pose = init_pose;
    return true;
  }

  bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame){
    Frame key_frame = new_key_frame;
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    local_map_frames_.push_back(key_frame);
    while(local_map_frames_.size() > static_cast<size_t>(local_frame_num_)){
      local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i=0; i<local_map_frames_.size(); ++i){
      pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                *transformed_cloud_ptr,
                                local_map_frames_.at(i).pose);
      *local_map_ptr_ += *transformed_cloud_ptr;
    }

    if (local_map_frames_.size() < 10){
      registration_ptr_->SetInputTarget(local_map_ptr_);
    }else{
      CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
      local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
      registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }
    return true;
  }
}
