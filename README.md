# MaixSense_ROS

[**MaixSense-A010 official docs**](https://wiki.sipeed.com/hardware/en/maixsense/maixsense-a010/maixsense-a010.html)

## Build and run instructions

```bash
$ cd <ros2 workspace>
$ colcon build --packages-up-to sipeed_tof_ms_a010
$ source install/setup.sh
$ ros2 run sipeed_tof_ms_a010 publisher --ros-args -p device:="/dev/ttyUSB0"
```