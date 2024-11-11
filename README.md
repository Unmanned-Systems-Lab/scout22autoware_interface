## 文件位置
```
|--autoware_for_scout
  |--src
    |--vehicle
      |--external
        |--scout22autoware_interface
        |--scout_base
        |--scout_description
        |--scout_msgs
        |--ugv_sdk(需要）
```
scout的CMakelists和package.xml有部分更改
***
## 编译
- autoware_for_scout源文件编译
- 单独编译 scout的三个文件和ugv_sdk
```bash
#在 /autoware_for_scout 处
colcon build --packages-up-to scout_base scout_description scout_msgs ugv_sdk
```
- 单独编译scout22autoware_interface
```bash
colcon build --packages-up-to scout22autoware_interface
```
***
## 使用
- 可以直接 ros2 launch scout22autoware_interface interface_v2.launch.py
- 整体上已经include到另外一个launch文件中（vehicle_interface.launch.xml）
