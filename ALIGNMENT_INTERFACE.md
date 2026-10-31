# 局部对准接口

当前实现软件流程和 ROS 2 接口，不包含电控驱动，也没有本车实测标定文件。
不自动插入整轮任务，不接管 STM32 主流程。任务调度器、STM32 和本模块只能由明确
的上层流程协调，不能同时控制底盘。当前只支持原料区指定颜色，不冒充位置 1/2/3。

## 流程与完成条件

`start` → 请求固定观察姿态 → 动作端确认停稳 → 等待稳定时间 → 请求新帧观测 →
按误差判断：连续新帧在容差内则 aligned；否则关闭本次识别、请求限幅 X/Y 修正 →
动作端确认修正完成并停稳 → 再等待、重新请求新帧。

修正和停止反馈必须匹配运行/指令 ID。`accepted` 只表示收到，不代表停稳或动作成功。
重新观测的采集时间必须晚于停稳反馈被接收并经过稳定时间，不能继续使用修正前图像。
同一次观察需保持相机会话，换会话即失败；相机、分辨率、点定义、HSV 版本、区域都须
与标定匹配。空目标、同色多目标、相机故障、动作失败均请求停止，不盲目重抓或补动作。

只有在已确认停稳时才观察。`aligned` 只表示最后连续观测的像素误差满足配置条件；
它不等于抓取/放置成功。输出仍附采集时间与失效期限，期限过后状态变 expired、清空目标。
后续抓放驱动必须在动作开始前重新检查期限、标定与机械条件。下层本地看门狗/急停不由
ROS 消息替代。停止未确认时拒绝重新 start；不得通过软件重启掩盖真实运动状态。

## 参数及标定

`AlignmentCalibration` 必须来自本车固定观察姿态和目标平面的测量：

| 字段 | 含义 |
| --- | --- |
| calibration_id | 本次本车标定版本 |
| camera_id、width、height | 相机身份与原图分辨率 |
| pose_id、plane_id | 固定观察姿态与目标平面/高度 |
| point_kind | 当前为 contour_bounding_box_center，不能改名冒充圆心或抓取点 |
| hsv_profile_sha256 | 产生观测的 HSV 参数版本 |
| reference_pixel | 该观察姿态下期望对准的原图像素 |
| correction_matrix_mm_per_pixel | 2×2 局部映射：像素误差 → 动作端 X/Y 修正，轴方向/符号需实测 |
| simulation_only | 真实标定为 false，模拟专用为 true，严格隔离 |

控制器不假定毫米运动正方向与像素正方向相同。该矩阵只在已标定的局部范围内适用；
俯视高度变化、倾斜、物料顶面/地面/上层码垛不能直接共用。当前不会自动生成真实标定
文件，模拟文件中的比例和参考像素不允许作为本车参数。设备标定按项目规则保持 Git 忽略。

默认控制预算：像素各轴容差 3 px、至少 3 帧且跨度至少 0.2 秒、帧间隔最多 0.7 秒；
单步欧氏距离最多 10 mm、累计请求距离最多 60 mm、最多 8 次修正；来源有效期 1 秒、
每请求 4 秒、总对准 30 秒、停稳等待 0.1 秒。这些仅是可配置的软件预算，未经本车
机械容差/响应测量验证，不是比赛要求或实机精度承诺；当前没有用它们驱动硬件。
预算通过 `AlignmentController` 初始化参数配置；CLI 暂使用这些默认值。

## ROS 2 通道

真实模式前缀 `alignment`；模拟为 `alignment_sim`。二者同时通过主题和
`simulation` 布尔字段隔离，模拟动作端拒绝真实命令。

| 主题后缀 | 内容 |
| --- | --- |
| control | start/cancel，schema_version=1、simulation、request_id、issued_at_unix_s；start 带 color_id 和 zone=raw |
| vision_command / vision_feedback | 与现有 schema 1 scan_material / observation、stop 兼容 |
| motion_command / motion_feedback | hold_observation_pose、adjust_xy、stop；有 run_id/command_id/issued_at_unix_s/timeout_s |
| status | phase、error、stop_confirmed、修正次数、累计请求距离、目标及失效期限 |

control 只接受 2 秒内的新请求，去重上限 4096；指令和反馈 reliable/volatile，
状态 transient-local。`motion_feedback` 的 done 必须带 stopped=true、对应 pose_id、
calibration_id；adjust_xy 还需 adjusted=true。stop 的 done 要求 stopped=true。
adjust_xy 参数包含 dx_mm、dy_mm、units=mm、coordinate_frame=calibrated_motion_xy、
来源观测及 target_expires_at_unix_s。动作端应检查期限、标定身份和自身限位/速度约束，
按指令 ID 幂等执行；收到过期指令应拒绝，不能靠自动重发补救。

视觉接入可复用既有 `vision_ros`，将其 mission/command 和 mission/feedback 分别
重映射到 alignment/vision_command 与 alignment/vision_feedback。这样不另开相机。
任务节点此时不得同时向它发识别/运动请求。当前尚未配置常驻运行或实际电控消费者。

## 运行与软件检查

Mac 终端或树莓派终端，在工程根目录运行纯软件模拟：

```bash
python3 -m pi_vision.alignment_demo
python3 -m pi_vision.alignment_demo --failure target_lost
python3 -m pi_vision.alignment_demo --failure stop_timeout
```

模拟正常例需多次观察和修正后连续确认；失败例必须进入 fault，stop_timeout 还须
保留 stop_confirmed=false。所有结果标记 simulation，不包含相机采集或真实动作。

树莓派终端，ROS 2 有界模拟检查，不打开相机、串口、电机：

```bash
cd /home/cuiguo/competition_vision
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=75
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m tools.alignment_ros_smoke
python3 -m tools.alignment_ros_smoke --failure target_lost
python3 -m tools.alignment_ros_smoke --failure motion_timeout
```

节点入口 `python3 -m pi_vision.alignment_ros` 默认待机且没有标定，收到 start 会拒绝。
`--simulation` 使用专用假标定和模拟主题；仍需模拟动作端，不自动开始。
未来真实运行通过 `--calibration <本机实测JSON>` 提供标定；本轮未创建此文件或接实机动作。

下一步是位置 1/2/3 身份、观察姿态与真实电控接口的联调；双摄、转盘停稳门控以及
固定安装/标定分别验收。局部对准软件模拟不能作为动态转盘抓取或整车闭环证据。
