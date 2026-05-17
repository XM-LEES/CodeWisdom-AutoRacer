# 阶段 4 离线预检记录

日期: 2026-05-17 11:00 CST
测试人员: Codex
阶段: 阶段 4A / 4B
Case: `map_load` / `cmd_vel_isolation` / `adapter_ackermann_output` / `reverse_plan_controlled`
上位机仓库/分支/提交: `CodeWisdom-AutoRacer` `feature/ackermann-chassis` `b18c6d2`，本轮有未提交改动
下位机仓库/分支/提交: `RCCar-new` `feature/ackermann-chassis` `f3e1ff7`，本轮未修改
根仓库提交: `main` `f440661`，本轮有未提交文档改动
run id: `stage4-offline-precheck-20260517-1100`

## 结论

阶段 4 不上车预检 PASS，但不计阶段 4 上车 PASS。

本轮证明内容:

- 4A 默认参数仍是 Smac Hybrid-A* `DUBIN`、RPP `allow_reversing=false`、Collision Monitor、`/nav2_cmd_vel -> /safe_nav2_cmd_vel -> /ackermann_cmd` adapter 链路。
- 4B 候选参数 `stage4_nav2_reverse_params.yaml` 已落地，使用 Smac Hybrid-A* `REEDS_SHEPP`、RPP `allow_reversing=true`、velocity smoother 保守倒车 `vx >= -0.30 m/s`。
- `twist_to_ackermann` 离线数学检查覆盖前进限幅、转角限幅、原地旋转拒绝、NaN 停车、4A 倒车禁用、4B 倒车限幅和倒车转向符号。
- 4A 和 4B dry launch 均能加载阶段 3 固定地图并启动 Nav2/Collision Monitor/adapter；因本轮故意关闭底盘/EKF，最终停在缺少 `odom -> base_footprint` TF，属于预期阻塞。

仍需上车验证: AMCL 初始位姿、`map -> odom` 稳定性、真实 `/scan`、真实 `/odom`、Nav2 goal、Collision Monitor 障碍停车、topic graph `/cmd_vel` 隔离、到点停车和 4B 倒车安全。

## 命令和结果

```bash
cd CodeWisdom-AutoRacer
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/acceptance/stage4_navigation_contract_check.py
colcon build --symlink-install --packages-select autoracer_robot_nav2 autoracer_bringup autoracer_robot_urdf
```

结果:

- `stage4_navigation_contract_check.py`: PASS `stage4_launch` / `robot_description_tf` / `nav2_params` / `nav2_reverse_params` / `adapter_math` / `adapter_diagnostics_contract`
- `colcon build --symlink-install --packages-select autoracer_robot_nav2 autoracer_bringup autoracer_robot_urdf`: PASS，3 packages finished

4A dry launch:

```bash
timeout 12s ros2 launch autoracer_bringup stage4_navigation.launch.py \
  "map:=/home/asshole/Desktop/New Folder/CodeWisdom-AutoRacer/docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.yaml" \
  counts_per_meter:=13.545 \
  start_imu:=false \
  start_chassis:=false \
  start_lidar:=false \
  start_robot_description:=true \
  start_nav2:=true \
  use_rviz:=false \
  use_composition:=False
```

关键输出:

- map server 读取 `stage3-final-floor2-loop-20260517-101119.pgm: 860 X 1201 map @ 0.05 m/cell`
- Collision Monitor 创建 `front_stop`、`front_slowdown`、`rear_stop`、`rear_slowdown`、`left_side_stop`、`right_side_stop`
- planner 配置 Smac Hybrid-A*，日志显示 `Using motion model: Dubin`
- 预期阻塞: `Timed out waiting for transform from base_footprint to odom`，因为 `start_chassis:=false`
- 日志目录: `/home/asshole/.ros/log/2026-05-17-10-59-14-190235-asshole-ThinkPad-T480-68807`

4B reverse dry launch:

```bash
timeout 25s ros2 launch autoracer_bringup stage4_navigation.launch.py \
  "map:=/home/asshole/Desktop/New Folder/CodeWisdom-AutoRacer/docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.yaml" \
  "params_file:=/home/asshole/Desktop/New Folder/CodeWisdom-AutoRacer/src/autoracer_robot_nav2/param/stage4_nav2_reverse_params.yaml" \
  counts_per_meter:=13.545 \
  allow_reverse:=true \
  start_imu:=false \
  start_chassis:=false \
  start_lidar:=false \
  start_robot_description:=true \
  start_nav2:=true \
  use_rviz:=false \
  use_composition:=False
```

关键输出:

- map server 读取同一阶段 3 固定地图
- Collision Monitor 创建前、后、侧向安全区
- planner 配置 Smac Hybrid-A*，日志显示 `Using motion model: Reeds-Shepp`
- 预期阻塞: `Timed out waiting for transform from base_footprint to odom`，因为 `start_chassis:=false`
- 日志目录: `/home/asshole/.ros/log/2026-05-17-11-00-12-699138-asshole-ThinkPad-T480-69625`

## 追踪表

| Case | 自动检查 | 人工检查 | 证据文件 | 状态 |
| --- | --- | --- | --- | --- |
| `map_load` | dry launch 中 map server 成功读取阶段 3 固定地图 | 尚未在现场确认地图与当前环境一致 | 本记录、ROS log | BLOCKED，上车前不计 PASS |
| `amcl_localization` | AMCL 节点启动并订阅地图 | 尚未设置初始位姿，缺真实 `/scan` 和 `/odom` | 本记录、ROS log | BLOCKED |
| `global_plan_forward` | 4A 参数契约 PASS，dry launch 显示 `DUBIN` | 尚未发送实车目标点 | contract 输出、ROS log | BLOCKED |
| `rpp_cmd_vel` | RPP 插件加载 PASS，4A `allow_reversing=false` | 尚未跟踪真实 path | contract 输出、ROS log | BLOCKED |
| `collision_monitor_stop` | 前/后/侧向 safety zones 均能创建 | 尚未用实物障碍验证 stop/slowdown | ROS log | BLOCKED |
| `cmd_vel_isolation` | launch 契约 PASS，Nav2 `/cmd_vel` remap 到 `/nav2_cmd_vel` | 尚未在完整上车图中采集 `ros2 node info` | contract 输出 | BLOCKED |
| `adapter_ackermann_output` | adapter 数学和 diagnostics 契约 PASS | 尚未用 live `/safe_nav2_cmd_vel` 采集 `/ackermann_cmd` | contract 输出 | BLOCKED |
| `goal_reached_stop` | 未发送目标点 | 未上车 | 无 | BLOCKED |
| `blocked_no_path` | 未构造阻断场景 | 未上车 | 无 | BLOCKED |
| `reverse_plan_controlled` | 4B reverse 参数、Reeds-Shepp dry launch、adapter 倒车限幅均 PASS | 倒车路径和后方/侧方安全尚未上车验证 | contract 输出、ROS log | BLOCKED，上车前不计 PASS |

## 未覆盖风险

- 本轮没有启动 STM32、EKF、IMU、LiDAR 或 RViz，不证明真实 `map -> odom`、`/amcl_pose`、`/scan` 和 costmap 质量。
- 本轮没有发布 Nav2 goal，不证明 `/nav2_cmd_vel`、`/safe_nav2_cmd_vel` 和 `/ackermann_cmd` 的动态 topic 证据。
- 4B 倒车仍是候选配置；未完成后方/侧方现场安全确认前，不允许记录自动倒车 PASS。
