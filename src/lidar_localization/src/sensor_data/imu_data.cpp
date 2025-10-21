#include "lidar_localization/sensor_data/imu_data.hpp"
#include <cmath>
#include <glog/logging.h>


namespace lidar_localization {
  Eigen::Matrix3f IMUData::GetOrientationMatrix(){
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;
  }

  bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, 
                        std::deque<IMUData>& SyncedData,
                        double sync_time)
  {

  }
}
