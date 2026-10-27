# 直线加速无人赛车算法 / Straight-line Acceleration

ROS 1 锥桶直线赛道感知、定位建图、固定赛道规划与前视PD控制。仓库：[subing509/straight-line-acceleration](https://github.com/subing509/straight-line-acceleration)。项目自有代码采用[MIT](LICENSE)，第三方保留[原许可](THIRD_PARTY_NOTICES.md)。

## 后续工作先读这里

维护日期：2026-10-27。本节是项目接续工作的上下文入口；后续先读本README，按下表只打开本次改动直接相关的文件和测试，不默认遍历整个项目。每次完成代码、参数、部署或验证变更，必须同步本README的对应条目。

README用于确定修改范围，不替代相关代码核验：具体实现、实际ROS参数或线上文件与本文冲突时，以直接证据为准，修正文档。未重新联网核验，不能把历史部署或停车状态说成“现在正在运行”。

### 1. 当前主线、版本与完成程度

- 当前维护主线是 `acceleration_event.launch` → `jiantu.py` + `zhixian_realtime.py` + `genzong_realtime.py`，使用固定地图/赛道坐标。文件名包含realtime，不表示使用另一套 `genzong_lidar.py`。
- `acceleration_lidar.launch` 是独立实验雷达链，默认shadow、输出false；主线验收不能套用到它。
- 主线配置：方向预算0.10、名义宽3.15米、配对距离2.5～3.8米、75米finish；最近保存的工控机核验日期为2026-10-23。本次文档维护没有连接工控机。
- 版本证据：本地主规划器SHA256为 `549b073262d4aec9fda49324d39043b2d3beeb4e3956c3ea42fd04815c4a12eb`，与该次部署记录相同；其他文件及实时进程不能仅凭这个哈希认定一致。
- 有门限调整前版本约75米自主行驶到finish的记录；新门限曾通过35秒无输出及65秒完整输出暖机静态检查，之后又有冷启动road_stale记录。静态成功、一次跑动和后续启动故障应分别保留，不能混成“最新版已稳定跑完”。真实轮角、VCU回执、制动距离和新配置动态稳定性仍待验证。

### 2. 按任务找文件：无需先遍历目录

下表文件均在 `src/acceleration_event/`，明确注明其他路径的除外。先搜索函数名，避免依赖容易过时的行号。

| 工作/现象 | 先看文件 | 关键函数或位置 | 关联验证 |
|---|---|---|---|
| 改当前运行参数、启动节点、关闭输出 | `launch/acceleration_event.launch` | 三个node的私有param；launch值优先于源码默认值 | `tests/test_track_tolerance.py`；相关参数读取点 |
| 开机航向错、GPS跳变、初始化等不到 | `src/jiantu.py`、`gps_integrity.py`、`heading_integrity.py` | `gps_callback`、`imu_callback`、`check_initialization` | 根目录`tests/test_gps_integrity.py`、`test_heading_integrity.py`、`test_mapping_gps_integrity.py` |
| 地图锥桶错位、安装方向不对 | `src/jiantu.py` | `transform_lidar_to_vehicle`、`transform_vehicle_to_world`、`_merge_cone_unlocked` | 外参、GPS/IMU时序；`test_localization_mapping.py` |
| 配对不到、宽度不适配、初始赛道轴不对 | `src/track_frame.py` | `paired_centers`、`bootstrap_frame` | `test_track_frame_nodes.py`、`test_track_tolerance.py` |
| 方向门限误拒绝、单排如何更新 | `src/zhixian_realtime.py` | `estimate_centerline`、`update_centerline` | 同上；首次拒绝时间与停车时间 |
| 车头转了却仍沿旧偏差控制 | `src/track_frame.py`、`src/genzong_realtime.py` | `body_line`、`estimate_local_centerline`、`get_local_centerline_target` | 当前位姿重投影、支持年龄；`test_track_frame_nodes.py` |
| 路径回归太快/太慢、路径不更新 | `src/zhixian_realtime.py` | `generate_local_path`、`publish_path` | 路径点与固定系几何 |
| 前视、转角、PD、振荡或响应迟缓 | `src/genzong_realtime.py` | `update_dynamic_lookahead`、`find_lookahead_point`、`filter_tracking_errors`、`pid_control` | 目标/误差/指令/实际轮角同时间线 |
| 一直waiting或一直停车 | `src/genzong_realtime.py`、`src/legacy_run_guard.py` | `check_start_condition`、`control_loop`、`evaluate`、`send_guard_stop` | `test_legacy_run_guard.py`；首个stop_reason |
| 过早finish、距离异常 | `src/genzong_realtime.py` | `update_finish_distance` | 根目录`tests/test_finish_net_progress.py` |
| 聚类无桶、CPU占用/消息积压 | `src/lidar/src/lidar_nodes/src/clustering.py`、`clustering_realtime.py`（工作区根路径） | `compute_clustering`、`ClusteringNode`、`LatestFrameWorker` | 根目录`tests/test_clustering_realtime.py`；`/perception/clustering_status` |
| 雷达视野裁切、地面误分 | `src/lidar/src/lidar_nodes/src/input_and_prefilter.py`、地面分割launch/config（工作区根路径） | `prefilter`、`segmentation_params.yaml` | 原始/预滤/障碍物点云同步对照 |
| 串口输入、IMU驱动CPU | `src/imu/src/daoyuan_simple.py`（工作区根路径） | 数据帧解析、航向赋值、日志节流 | 根目录`tests/test_imu_logging_cpu.py` |
| 加日志、自动诊断 | `src/run_diagnostics.py`、`run_diagnostics_core.py` | `launch/run_diagnostics.launch` | `test_run_diagnostics.py`；旁路不接管车辆 |

### 3. 主算法：修改时必须保持的关系

```text
雷达点云 → 范围预滤 → 地面分割 → 自适应DBSCAN → /clustered_points
GPS/IMU → 有效性与连续初始化 → /vehicle_pose
候选桶 + 当前位姿 → 固定地图 /cone_map（保留来源时间）
地图 → 固定赛道T配对 → d=k·s+b → 平滑路径 /planned_path
固定中心线 → 每控制周期投影到当前车体B → 局部前视目标
路径目标 + 局部目标 → 误差融合/滤波 → 横向+航向PD → 转角限幅 → UDP/CAN
来源时效与固定轴75米进度 → 运行保护 → 停车锁存
```

- M地图系、T赛道系固定；B车体系随当前位姿变化。历史k/b不能直接当当前车体k/b。`body_line`每周期重投影是持续纠偏的关键。
- 无需先推动车辆。GPS和IMU各通过准入且均新鲜后才开始3秒初始化等待；原点提交前异常或失鲜重新计时。一组完整左右锥桶可建线，单个锥桶不够。
- 配对不依赖颜色：欧氏桶距2.5～3.8米、T中纵向差≤0.8米，名义宽3.15米参与评分；最多两排，同一桶不复用，排中心纵向间距≥1.5米。候选相对车辆纵向−8～25米、横向距离≤6米、最多20桶；不是按旧`local_max_x=24`读实际筛选。
- 首双排方向相对T轴检查；接受后冻结 `centerline_reference_k`。以后判断 `abs(atan(k_raw)-atan(k_ref))≤atan(0.10)`，约5.71°，不是限制车头只能转5.71°。参考不随累计漂移移动。只有一排时保留方向、更新位置。
- 超方向门限拒绝整次观测；尚未建成有效线时允许撤销启动假设重试。通过后斜率变化先限±0.02、乘0.12，再整体平滑。真实车头偏转不直接作为固定赛道异常斜率。
- 路径每10Hz生成前方30米、间隔0.5米，双排用6米余弦过渡、单排12米，横向回归量限±1.5米。`total_length=175`不是当前停车距离。
- 控制约20Hz：`Ld=clip(2.5+0.35v,2.5,4.0)`米，v用米/秒；局部目标横向限±1米。两排局部权重0.65、单排0.45，过期局部目标回退路径，仍检查道路失鲜。
- 这是前视选点+PD，不是标准Pure Pursuit曲率控制。e_y是前视目标的侧向量，e_psi是目标方位误差，不能分别当作真实赛道垂距、中心线切线误差。
- 地图整体来源时间只由有效非空检测刷新；历史桶保留6秒不等于允许盲开6秒。来源新鲜也不证明每个历史桶本帧重新被看到。
- 聚类计算的置信度未随PoseArray传下去，不能与地图观测次数置信度混为一谈。控制器现用规划器的fixed_road，旧`_pair_candidates`、`local_max_slope`等不是主环另一次独立识别。
- IMU约定是 `orientation.z=航向度数`，不是标准四元数；`/vehicle_pose`才发布四元数。变更驱动必须核对单位、正负及坐标。

### 4. 实际参数速查

| 位置/读取键 | 当前值 | 含义/注意事项 |
|---|---|---|
| 规划`straight_slope_limit` | 0.10 | 固定参考方向角差约5.71° |
| 规划`track_width`；`pair_distance_min/max` | 3.15m；2.5/3.8m | 名义宽评分；实际配对欧氏距离 |
| 规划`pair_max_longitudinal_diff`；`max_pair_count` | 0.8m；2 | 同排错位上限；最多排数 |
| 规划`max_slope_change`；`slope_alpha` | 0.02；0.12 | 方向变化截断与平滑 |
| 规划实际读取`new_path_weight` | 默认0.15 | 源码变量名centerline_alpha；launch中的`centerline_alpha=0.15`没有覆盖它！调平滑先核对此键 |
| 规划`local_path_length`；`path_point_spacing` | 30m；0.5m | 实际路径窗口，不是175m |
| 控制`lookahead_min/max`；`lookahead_speed_gain` | 2.5/4m；0.35s | 动态前视 |
| 控制`kp_lateral/kd_lateral`；`kp_heading/kd_heading` | 0.60/0.05；0.20/0.01 | 实际PD，无积分项 |
| 控制`error_filter_alpha` | 0.12 | 误差一阶平滑 |
| 控制`heading_weight_min`；`heading_weight_full_lateral_error` | 0.35；0.80m | 航向项随侧向误差降权 |
| 控制`max_lateral_derivative`；`max_heading_derivative` | 0.60m/s；0.35rad/s | 微分限幅 |
| 控制`steering_sign`；`max_steering`；`max_steering_step` | +1；0.50rad；0.012rad/周期 | 软件限幅±28.65°；不等于实际轮角 |
| 控制`max_vehicle_speed_mps`；`fixed_speed` | 4.1667m/s | 约15km/h的估速/仿真配置，不能当作已向实车下发油门或实际限速 |
| 建图`cone_merge_distance`；`cone_timeout` | 0.8m；6s | 历史锥桶合并与保留 |
| 安装`lidar_offset_x/y/z`；`lidar_y_sign`；`lidar_yaw_offset_deg` | 1.76618/−0.11304/0.42850m；−1；0.5° | 原车外参，需按实车核验 |
| 预滤硬编码窗口 | x:0～6m，y:−3～3m，z<0.8m | 前向有效点云不是25m |
| 聚类`max_frame_age`；K/MIN_PTS | 默认0.4s；5/5 | 最新帧来源预算；自适应密度聚类 |

### 5. 状态机、时间预算和停车报文

| 项目 | 当前逻辑 |
|---|---|
| waiting | 输入未齐或输出客户端未就绪；等待可恢复，不因尚未启动时缺数据新锁存停车 |
| running | 路径/位姿具备、位姿/IMU/路径/道路均通过时效、输出条件满足；即使车还静止也开始检查运行超时 |
| stopped | 已运行后首次故障或finish锁存；后续继续请求停车，输入恢复不自动解除 |
| 位姿/IMU保护 | 各0.8秒 |
| 路径/道路保护 | 各1.0秒；按来源年龄加运输及接收后时间，不靠重复发布续期 |
| 局部融合预算 | 0.35秒；超龄先回退路径 |
| 规划新拟合预算 | 地图来源≤0.50秒 |
| finish | 从启动位置沿首次冻结赛道方向投影的净进度≥75米，不累计GPS抖动路程 |
| 正常转向 | CAN 0x210，`raw≈round(500+δ·1000/π)`，δ弧度；UDP目的参数来自launch |
| 停车请求 | 同帧raw=2000；正常零转角raw=500，两者不同 |

`started=true`仅表示曾启动；结合 `state`、`stop_reason`、`waiting_for` 判断。`output_ready`和`stop_udp_sent`只表示软件通道就绪/本地发送，不是VCU回执。规划器3秒无桶警告不是实际1秒停车门限。其他原因包括pose/imu/path_stale、clock_rewind、no_lookahead_target、control_exception、can_send_failed。

### 6. 已知问题与下一项工作

1. 有效检测间断：最近保存的冷启动记录先在06:36:05.471锁存road_stale，06:36:47.392才首次方向拒绝；不能倒因为“方向门限导致这次首次停车”。后续12秒119帧中75空、39单候选、5帧至少两个候选，属于停车后补充采样，不能还原首次超时的每帧原因。
2. 排查先取同一时间线的原始点云、预过滤、障碍物、候选、地图、位姿、道路支持和控制状态，定位可见性、裁切、地面分割、密度和关联的损失位置；不先取消保护或继续放宽方向。
3. 持续单侧转角尚需区分真实赛道偏差、外参/航向、目标几何与转向执行映射。实际轮角、轴距、执行/测量延迟未标定；软件输出持续不代表纠偏收敛。
4. 原轨迹记录停止于停车锁存前后，不能用最后CSV点推算真实刹停距离；无完整原始点云包时不宣称完整重建真实赛道。
5. 新门限已静态验证，仍需冷启动感知连续性与新配置低速动态验收。README记录是截至该次证据的结论，不是当前远程车辆状态。

### 7. 验证与日志：按修改范围选择

主机选定几何/方向/锁存回归（41项；无需ROS、传感器或车辆网络）：

```bash
# 在本README所在目录执行；进入子目录后结束该shell即可回到原目录
(cd src/acceleration_event/tests && python3 -B -m unittest -v test_track_frame_nodes test_track_tolerance test_legacy_run_guard)
```

公开副本可直接 `python3 tools/run_offline_checks.py`。41项结果不是全历史目录全部通过。文档或链接变更核对内容与链接即可，不为排版更改重跑车辆测试。

目标Ubuntu/ROS中，独立工作区构建后可选：

```bash
python src/acceleration_event/scripts/smoke_ros_validation.py
python src/acceleration_event/scripts/ros_fault_validation.py --output ~/nrt_ros_fault_new
```

这些脚本自建隔离Master并关闭车辆输出；不在已连接实车的正式ROS Master混入合成输入。真实传感器/节点部署检查与离线回归是不同层级，不能互相代替。

| 读取内容 | 位置/接口 |
|---|---|
| 控制状态和首次停车原因 | `/control/legacy_status` |
| 固定道路k/b、frame、pairs、观测年龄 | `/planning/legacy_road_status` |
| 聚类输入、替换、超龄、错误、发布及处理耗时 | `/perception/clustering_status` |
| GPS/IMU准入原因 | `/localization/gps_status`、`/localization/imu_status` |
| 控制CSV | `/tmp/path_tracking_logs/control_log_*.csv`，停车锁存后不能假设它仍持续记新行 |
| ROS日志 | `~/.ros/log/latest/`或当次run目录 |
| 旁路诊断 | `roslaunch acceleration_event run_diagnostics.launch mode:=legacy`；默认`/tmp/run_diagnostics/` |

诊断器只监听，`/diagnostics/run_active`只控制诊断报警，不是发车门禁。无输出候选话题为 `/control/legacy_candidate_steering`；正式软件转角为 `/control/steering`。`/tmp`可能在重启后清理，应持久化所需记录。

### 8. 后续修改完成时的README更新约定

每次改动同步记录：①具体功能/文件/函数；②实际参数键、值、单位和加载入口；③验证命令、结果及未覆盖项；④仅本地修改、公开副本同步还是已部署车机；⑤新增问题和下一步。保留历史证据的日期，不能把旧静态成功写成当前动态验收。

日常流程：读README → 按任务索引查看相关函数/测试 → 做最小修改 → 执行对应层级检查 → 更新README → 按明确任务同步发布副本或部署。仅改文档不重启节点、不发控制报文；更新公开副本不等于更新工控机。未被要求时不自动上线运行车辆。

详细公式见[算法说明](docs/ALGORITHMS.zh-CN.md)。本公开副本不包含本地EBS/八字研究包、现场日志、录包、备份和远程连接资料。与其他赛项的状态不得混用。

## 环境与构建

历史目标环境为 Ubuntu 18.04、ROS Melodic、Python 2.7。主机离线测试使用Python 3，不代表节点可直接迁移到其他ROS/Python组合。硬件输入需要Velodyne驱动、匹配的串口设备与项目航向格式。

主要依赖：catkin、rospy、tf、标准ROS消息、NumPy、OpenCV、PyYAML、pyserial、ros_numpy、SciPy/scikit-learn、Velodyne；地面分割另需catkin_simple、glog_catkin及其依赖。先按目标ROS环境解决依赖，不复用其他机器的build/devel。

完整构建示例（Ubuntu终端，在新建独立工作区中进行）：

```bash
git clone https://github.com/subing509/straight-line-acceleration.git
cd straight-line-acceleration
source /opt/ros/melodic/setup.bash
# 如未安装地面分割的源码依赖，使用vcstool导入；需事先安装vcstool。
vcs import src < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
catkin_init_workspace src
catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python2
source devel/setup.bash
rospack find acceleration_event
rosmsg show mapping/ConeArray
```

`dependencies.repos`跟随上游分支，首次成功构建后应自行冻结依赖提交；本次发布未在全新Ubuntu环境重做完整硬件依赖构建。只验证业务链时，可把 `mapping`、`acceleration_event` 复制到新的catkin工作区构建。

## 无车辆输出验证

```bash
# 已构建并source的独立ROS环境
roslaunch acceleration_event validation.launch synthetic_inputs:=true
```

该入口显式关闭车辆输出和可视化，提供合成输入；不要同时连接正式车辆ROS Master。标准主入口 `acceleration_event.launch` **默认启用车辆输出**。同名节点的私有参数必须正确加载，不能仅凭另一个匿名节点名下的参数断言输出已关闭。

仅在明确具备实车测试条件、已核对安装外参/转向符号/底盘协议后使用硬件链：

```bash
roslaunch lidar_nodes lidar.launch
rosrun imu daoyuan_simple.py
# 首次读取真实传感器，先保持输出关闭
roslaunch acceleration_event acceleration_event.launch enable_vehicle_output:=false
```

串口驱动将航向角（度）放在 `/imu_data.orientation.z`，不是标准四元数；不能直接替换为任意IMU驱动。CAN目的地址、雷达安装偏移和转向映射是原车配置，需要适配本车。

## 离线回归

```bash
python3 tools/run_offline_checks.py
```

入口只运行指定的无ROS/无底盘输出几何与保护测试；历史测试目录不是全部兼容当前配置，不能将选定回归通过写成“全目录全部通过”。真实ROS启动、实际轮角和车辆闭环仍需分别验证。

## 当前验证边界

源码快照包含方向门限和2.7～3.6米赛道适配修改。历史有一次调整前版本的约75米自主行驶记录；调整后曾通过暖机静态验证，随后重新开机仍出现有效锥桶检测间断、`road_stale`停车锁存。因此本仓库不宣称已完成新配置的动态赛道验收。

`running`只表示软件运行；UDP发送成功不表示VCU确认或轮角达到。停车锁存后数据恢复不自动重新驱动。软件转角限幅±0.50弧度不等于实际前轮机械角。

公开副本仅整理许可证、文档、构建安装规则和LF换行，不重新调参，也不修改原工作区或工控机。发布差异见 [发布说明](docs/RELEASE_NOTES.md)。

2026-10-27：README补齐接续工作索引、参数与验证边界；本次仅文档更新。
