# 阶段 4A 实车前向导航记录

日期: 2026-05-17 11:45-12:35 CST
测试人员: Codex + 现场人员
阶段: 阶段 4A
Case: `map_load` / `amcl_localization` / `global_plan_forward` / `rpp_cmd_vel` / `cmd_vel_isolation` / `adapter_ackermann_output` / `goal_reached_stop`
上位机仓库/分支/提交: `CodeWisdom-AutoRacer` `feature/ackermann-chassis`，本轮有未提交改动
下位机仓库/分支/提交: `RCCar-new` `feature/ackermann-chassis`，本轮未修改
run id: `stage4-live-forward-nav-20260517`

## 结论

阶段 4A 前向单目标点导航 PASS。LiDAR 网络阻塞已定位并恢复；阶段 4 完整验收仍剩实物障碍、不可达目标和 4B 倒车扩展项。

已通过:

- 使用阶段 3 固定地图启动 Stage 4 Nav2，Nav2 localization、navigation、Collision Monitor 均进入 active。
- `/cmd_vel` 在 Stage 4 图中为 unknown；controller 和 behavior server 输出 `cmd_vel_nav`，再经 `velocity_smoother -> /nav2_cmd_vel -> collision_monitor -> /safe_nav2_cmd_vel -> twist_to_ackermann -> /ackermann_cmd`。
- `NavigateToPose` 1m、2m 前向目标成功，action `status=4`，controller 日志 `Reached the goal!`，bt navigator 日志 `Goal succeeded`。
- 速度上限按现场要求为 `0.50 m/s`；最新 2m 成功 bag 中 `max_cmd=0.500`、`max_safe=0.500`、`max_ack=0.500`、`max_actual=0.525`。
- 停车后 `/ackermann_cmd speed_mps=0.0 brake=true`，`/chassis_state actual_speed_mps=0.0 brake_active=true rc_override_active=false estop_active=false`。
- LiDAR 网络恢复方式为将主机有线口 `enp0s31f6` 配到历史主机地址 `192.168.1.102/24`；雷达仍为 `192.168.1.200`，并非雷达 IP 变化。恢复后 `ping 192.168.1.200` 为 `0%` 丢包，`/scan` 约 `18-20 Hz`。

本轮修正:

- RPP 接近目标最低速度从 `0.05/0.12 m/s` 提高到 `0.18/0.18 m/s`，避免进入实车爬行不动区。
- `NavigateToPose` 行为树切换为 `stage4_navigate_to_pose_goal_update_only.xml`，只在目标更新时重规划，避免短目标运行中重规划 action 握手超时。
- 到点容差调整为 `xy_goal_tolerance=0.35 m`，记录为当前阶段 4A 实车容差；该容差已完成 1m 和 2m 上车复测。

仍阻塞:

- 12:21 LiDAR 驱动连续 `poll() timeout`，`/scan` 不稳定，AMCL 无法恢复 `map -> base_footprint`；已停止跑车。复查确认本机有线口没有 `192.168.1.x` 地址，`ping 192.168.1.200` 100% 丢包。12:29 将 `enp0s31f6` 配置为 `192.168.1.102/24` 后 LiDAR 恢复，`/scan` 和 AMCL/TF 恢复。
- `collision_monitor_stop` 未用实物障碍验证。
- `blocked_no_path` 未构造不可达目标验证。
- `reverse_plan_controlled` 属于阶段 4B，仍为后续。

## 现场安全边界

- 现场人员先确认前方约 `10 m`、左右各约 `2 m` 范围安全；后续更新为前方 `8 m`、左右各 `2 m`，最终更新为前方 `15 m`、左右各 `2 m` 范围确定安全。
- 所有非零跑车命令均在确认后执行。
- LiDAR `/scan` 中断后未继续发运动目标；网络恢复、`/scan`、AMCL/TF 和 Nav2 lifecycle 均恢复后，才执行 1m 和 2m 前向复测。

## 关键命令

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autoracer_bringup stage4_navigation.launch.py \
  "map:=/home/asshole/Desktop/New Folder/CodeWisdom-AutoRacer/docs/test-records/maps/stage3-final-floor2-loop-20260517-101119.yaml" \
  counts_per_meter:=13.545 \
  use_rviz:=false

python3 tools/acceptance/stage4_navigation_contract_check.py
colcon build --symlink-install --packages-select autoracer_robot_nav2 autoracer_bringup autoracer_robot_urdf
```

## rosbag 和日志

| run | 结果 | 证据 |
| --- | --- | --- |
| `stage4-1m-forward-0p5-20260517-115026-a` | FAIL，接近目标时 RPP 降到约 `0.054 m/s`，底盘不再前进，progress checker abort | `docs/test-records/rosbags/stage4-1m-forward-0p5-20260517-115026-a` |
| `stage4-1m-forward-0p5-min018-20260517-115830` | FAIL，BT 运行中重规划 action 握手超时，已安全停车 | `docs/test-records/rosbags/stage4-1m-forward-0p5-min018-20260517-115830` |
| `stage4-1m-forward-0p5-goalupdatebt-20260517-120515` | PASS 1m 前向目标；同 bag 内 2m 扩展段接近目标后 progress checker abort | `docs/test-records/rosbags/stage4-1m-forward-0p5-goalupdatebt-20260517-120515` |
| 12:21 原地 LiDAR 复查 | BLOCKED，未发运动目标；`ros2 topic hz /scan` 6 秒无消息，`ping 192.168.1.200` 100% 丢包，`enp0s31f6` 显示 disconnected | launch 日志、`ip -br addr`、`nmcli device status` |
| `stage4-1m-forward-0p5-after-lidar-recovery-20260517-123246` | PASS，LiDAR 网络恢复后 1m 前向目标成功 | `docs/test-records/rosbags/stage4-1m-forward-0p5-after-lidar-recovery-20260517-123246` |
| `stage4-2m-forward-0p5-after-lidar-recovery-20260517-123430` | PASS，LiDAR 网络恢复后 2m 前向目标成功 | `docs/test-records/rosbags/stage4-2m-forward-0p5-after-lidar-recovery-20260517-123430` |
| `stage4-rviz-demo-forward-0p5-20260517-124908` | PARTIAL，RViz/手动目标调试：前两个目标 planner 无有效路径 abort；目标 `(8.32, 0.44)` 规划和跟踪成功；随后较远目标 `(14.90, 2.33)` 开始跟踪后 progress checker abort。停止后确认 `/ackermann_cmd speed=0 brake=true`、`/chassis_state actual_speed_mps=0 brake_active=true`。后续观察到 AMCL/map 位姿跳变，继续 RViz 调试前需重新 `2D Pose Estimate` 对齐。 | `docs/test-records/rosbags/stage4-rviz-demo-forward-0p5-20260517-124908` |

成功 bag 指标:

```text
bag=stage4-1m-forward-0p5-after-lidar-recovery-20260517-123246
odom_disp=0.664
wheel_odom_disp=0.664
sum_delta_s=0.664
max_cmd=0.500
max_safe=0.475
max_ack=0.475
max_actual=0.459
scan_hz=19.41
last_chassis actual=0.000 brake=True rc=False estop=False timeout=False
last_ack speed=0.000 steering=0.000 brake=True

bag=stage4-2m-forward-0p5-after-lidar-recovery-20260517-123430
odom_disp=1.548
wheel_odom_disp=1.548
sum_delta_s=1.550
max_cmd=0.500
max_safe=0.500
max_ack=0.500
max_actual=0.525
scan_hz=18.37
last_chassis actual=0.000 brake=True rc=False estop=False timeout=False
last_ack speed=0.000 steering=0.000 brake=True

bag=stage4-rviz-demo-forward-0p5-20260517-124908
duration=130.047s
plan_count=2
success_segment target=(8.32,0.44) odom_disp=2.291 wheel_odom_disp=2.279 max_ack=0.500 max_actual=0.569 result=Goal succeeded
long_goal_segment target=(14.90,2.33) odom_disp_before_stop=2.090 wheel_odom_disp_before_stop=2.067 max_ack=0.500 max_actual=0.501 result=Failed to make progress / abort
planner_abort_targets=(0.64,0.04),(1.93,0.20) reason=failed to create plan, exceeded maximum iterations
final_manual_stop_confirmed last_ack speed=0.000 steering=0.000 brake=True; last_chassis actual=0.000 brake=True rc=False estop=False timeout=False
```

## Case 状态

| Case | 状态 | 证据 |
| --- | --- | --- |
| `map_load` | PASS | map server 读取阶段 3 固定地图 |
| `amcl_localization` | PASS | LiDAR 网络恢复后 `/scan` 约 `18-20 Hz`，AMCL 输出 `/amcl_pose`，`map -> base_footprint` 可查询 |
| `global_plan_forward` | PASS | `ComputePathToPose` 对 1m、2m 目标均生成 `map` path |
| `rpp_cmd_vel` | PASS | 1m、2m 成功段 RPP 输出经 velocity smoother 到 `/nav2_cmd_vel`，最大 `cmd_vel_nav=0.500 m/s` |
| `collision_monitor_stop` | BLOCKED | 本轮未放置障碍；LiDAR 后续 timeout 后停止测试 |
| `cmd_vel_isolation` | PASS | `/cmd_vel` unknown，底盘桥 legacy 订阅 remap 到 `/stage4_legacy_cmd_vel_disabled` |
| `adapter_ackermann_output` | PASS | `/safe_nav2_cmd_vel -> /ackermann_cmd`，停车后 `brake=true` |
| `goal_reached_stop` | PASS | 1m、2m `NavigateToPose` action `status=4`，停车证据齐全 |
| `blocked_no_path` | BLOCKED | 未构造不可达目标 |
| `reverse_plan_controlled` | 未实现/4B 后续 | 本轮未跑倒车 |

## 未覆盖风险

- 当前 PASS 以 1m、2m 前向单目标为核心证据，不等于长距离路线或动态避障通过。
- RViz 任意点导航尚不稳定：目标落在不可达区域会 planner abort；较远目标跟踪中可能触发 `Failed to make progress`；继续调试前必须先确认 AMCL 位姿和 `/scan` 与地图对齐。
- `xy_goal_tolerance=0.35 m` 是本轮实车收敛结果，已通过 LiDAR 恢复后的 1m/2m 上车复测；更严格容差需后续重新调参和复测。
- LiDAR UDP `poll() timeout` 的本次原因已定位为主机有线口未连通或未配置到 LiDAR 网段；后续重启测试前必须确认 `enp0s31f6=192.168.1.102/24`、`ping 192.168.1.200` 和 `/scan` 频率。
