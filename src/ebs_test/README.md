# EBS制动测试独立程序

维护日期：2026-10-28。先读[项目总README](../../README.md)，再按下表定位实现。本包复用直线加速的GPS/IMU、雷达、固定地图/赛道规划和PD跟踪，独立部署。

场地为25米加速区加10米制动区，目标在25米处由裁判测速达到40 km/h，现场人员通过独立RES急停触发硬件EBS。软件25/35米仅记录几何区域，不触发或解除硬件制动。**本包没有实车油门或目标速度下发协议，开放转向不能保证车辆加速。**

## 当前入口与范围

提供原协议整链输出入口，复用0x210转向及raw=2000软件停车。默认shadow入口无输出；输出入口必须显式选择。公开发布不代表已部署或正在运行。

```bash
# 默认无输出整链
roslaunch ebs_test ebs_shadow.launch
# 实车原协议整链；先停止旧业务，不可与旧控制器并行
roslaunch ebs_test ebs_vehicle_test.launch
```

两个入口均启动建图、规划和控制，复用现有GPS/IMU、雷达和聚类，不重复开设备。现场控制独立RES及驱动许可；程序不读取或伪造Go/EBS/Emergency状态。running不代表驱动获准或车辆正在行驶。

输出入口设置enable_vehicle_output=true、output_mode=original_protocol_test、profile=baseline_shadow；只改enable为true仍拒绝。highspeed_candidate仅供无输出研究，不允许车辆输出。不自动重试输出通道，不通过运行期参数开放输出。

## 按任务定位

| 工作 | 先看文件/方法 | 联动核对 |
|---|---|---|
| 启动、重映射、CAN参数 | launch/ebs_shadow.launch、ebs_vehicle_test.launch | 默认false、显式输出模式、profile |
| 输出准入、竞争与起步 | scripts/ebs_controller.py：configure_outputs、check_command_publishers、check_start_condition、refresh_output_readiness | 构造前检查；运行冲突锁存；waiting阻止冲突起步 |
| 独占与原协议 | scripts/ebs_output.py：exclusive_client_class | 不设置SO_REUSEADDR/PORT，调用旧CANUDPClient.send_steering_frame |
| 进度与日志 | ebs_controller.py：update_finish_distance、control_loop；ebs_core.py：CourseProgress | 冻结首个新鲜位置/方向；净进度；不触发75米finish |
| 前视、PD、超时 | config/baseline_shadow.yaml | 实际实现../acceleration_event/src/genzong_realtime.py及guard |
| 长前视候选 | config/highspeed_candidate.yaml | 只供shadow；未高速标定 |
| 安装 | CMakeLists.txt | ebs_core、ebs_output均复制devel并安装 |
| 回归 | tests/test_ebs.py | 实际适配类/父类、真实UDP回环、独占与锁存 |
| 目标ROS隔离验证 | 现场独立验证工具（不随源码发布） | 独立Master、合成输入、仅回环UDP，不在实车Master注入故障 |

依赖同工作区acceleration_event和mapping；devel从源码导入父类，install通过roslib定位。更改共享算法需联动回归并带上配套部署。

## 接口、参数与保护

| 项目 | 当前行为 |
|---|---|
| /GPS_data、/imu_data、/clustered_points | 沿用原车接口、IMU约定与外参 |
| 位姿、地图、路径、道路、候选/控制状态 | 隔离到/ebs_test/ |
| /ebs_test/candidate_steering | 仅shadow，弧度，不能接执行器 |
| /control/steering | 输出模式的软件转角，不是实际轮角反馈 |
| UDP目的/源 | 192.168.0.7:20005 / 本地11312 |
| 正常CAN0x210 | raw=round(500+角度弧度×1000/π)，uint16小端；零转角500 |
| 软件故障及退出 | 同帧raw=2000；独立硬件RES仍由现场触发 |
| /fssim/cmd | EBS所有模式禁用；无实车油门或车速报文 |

独占本机源端口防止旧SO_REUSEADDR发送者共享11312；冲突则启动失败。每秒检查其他/control/steering、/fssim/cmd发布者，查询失败同样阻止起步或运行后停车锁存。仅覆盖本机源端口和已知ROS话题，不能证明其他设备/端口没有发送者。

- GPS合理性预算14 m/s，控制估速预算13.888889 m/s，均不是实车限速/油门。
- 前视clip(2.5+0.35v,2.5,4)米，横向PD=0.60/0.05、航向=0.20/0.01、误差alpha=0.12；转角±0.50 rad，20Hz周期变化≤0.012 rad。未针对40 km/h完成标定。
- 位姿/IMU各0.8秒、道路/路径各1秒保护保留。运行后首故障锁存，输入恢复不解锁；不为起步取消保护。
- 名义宽3.15米，配对2.5～3.8米，固定参考方向预算0.10；当前B系每周期重投影固定中心线。真实预滤前向视野仍0～6米。
- 规划平滑实际键为new_path_weight；旧centerline_alpha键不覆盖它。
- 25/35/75米不自动finish，正常制动由独立RES触发。软件未知制动后转向权限，仍沿用原协议；需现场验证VCU/RES覆盖关系。

## 状态、日志与验证

```bash
rostopic echo /ebs_test/tracker_status
rostopic echo /ebs_test/status
rostopic echo /ebs_test/road_status
```

shadow为mode=shadow_only、tracker_state=shadow_tracking；输出为original_protocol_test/tracking。started仅表示曾启动，结合state、stop_reason、waiting_for。udp_frames_sent、udp_stop_frames_sent、last_udp_send_ok仅是本地发送结果，vcu_acknowledged为null；hardware_go、hardware_ebs_active、hardware_emergency、hardware_standstill、actual_speed_mps、test_passed均为null。GPS差分估速不是裁判测速。

日志默认~/.ros/ebs_test_logs，含控制CSV和独立EBS status.jsonl。锁存后CSV不保证继续记行，JSONL继续记录；不能用末条CSV推算停车距离。

```bash
python -B -m unittest discover -s src/ebs_test/tests -v
python -B -m unittest discover -s src/acceleration_event/tests -p test_track_frame_nodes.py -v
```

公开版本验证：运行上述离线测试，覆盖协议UDP回环、端口独占、停车锁存和几何逻辑。离线测试不代表ROS构建或实车动态验收。没有实车油门或速度协议，不能保证40 km/h或10米停车。CSV工具只做几何进度诊断。

已知限制：动态记录曾出现反向纠偏延迟，误差滤波和每周期转角变化限制需要结合实际轮角与延迟继续标定；高速稳定性、RES触发和停车距离尚未完成验收。
