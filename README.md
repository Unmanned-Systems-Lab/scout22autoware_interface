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
        |--ugv_sdk
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
- 需要启动scout的节点。

- 节点同步启动
```bash
ros2 launch scout22autoware_interface interface_scout.launch.py
```

- 启动纯跟踪
```bash
 ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/autoware_map_test vehicle_model:=scout_vehicle sensor_model:=scout_sensor_kit lateral_controller_mode:=pure_pursuit
```
- MPC
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/autoware_map_test vehicle_model:=scout_vehicle sensor_model:=scout_sensor_kit lateral_controller_mode:=mpc
```
- Simulation
```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/lhl/autoware_map_test vehicle_model:=scout_vehicle sensor_model:=scout_sensor_kit lateral_controller_mode:=mpc
```

- 编译&指定编译
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select scout22autoware_interface
```
## 说明
 - 20241122：适配v1.0的autoware文件。
 - 20241125: 正向速度、逆向转向角求解(后轴-中心 中心-中心的变换都是错误的)
  ![中心-中心变换解析图像](https://raw.githubusercontent.com/Unmanned-Systems-Lab/scout22autoware_interface/refs/heads/v1_0/%E8%A7%A3%E6%9E%90%E5%87%BD%E6%95%B0%E5%9B%BE%E5%83%8F.png)
 - 20241203: 初步测试完毕。需要在pid横向最高级控制中加大delay_compensation这个参数(目前是1)。
