# Autonomous Driving (Learning)

This code is followed by the tutorial in Chinese [here](https://zhuanlan.zhihu.com/p/113616755). Author also provided code for this tutorial in this [repo](https://github.com/Little-Potato-1990/localization_in_auto_driving), however it is in ROS 1 and currently not maintained. My implementation is based on his repo and written to be compatible with ROS 2 humble. Translation to English (Vietnamese) will be added when I have time :).

## KITTI dataset
Check file `sample_downloader.sh`

Converting raw data format to ROS 2 `bag` data format by the following repo:
```
https://github.com/Chris7462/kitti_to_ros2bag
```
Modify the to the data of the `kitti_to_ros2bag.yaml` file

Compile and run:

```shell
colcon build --symlink-install --packages-select kitti_to_ros2bag
source ./install/setup.bash
ros2 launch kitti_to_ros2bag kitti_to_ros2bag_launch.py
```
The above method produce bag file in ROS 2 format but missing the `tf`.

Another way is to convert the KITTI format to ROS 1 and then use tool to convert from ROS 1 to ROS2.
Convert from KITTI to ROS 1 using repo [kitti2bag](https://github.com/ulterzlw/kitti2bag)
Use docker for convenient because it is required to have ROS 1 installed.

```shell
docker run -v `pwd`:/data -it ulterzlw/kitti2bag -t 2011_10_03 -r 0027 raw_sync
```

Then install `rosbags` using python pip
```shell
pip install rosbags
```

Use `rosbags-convert` to convert from ROS 1 to ROS 2 format
```shell
rosbags-convert --src 2011_10_03_drive_0027_sync.bag --dst ./2011_10_03_drive_0027_sync_bag/
```






