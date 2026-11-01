# 初赛任务框架

当前是可运行的调度层和模拟后端，不是已经能开车、抓取的驱动。
不依赖网页，不开启相机或串口，不修改已验证 HSV，也不包含雷达。
现场标定和硬件反馈可以后接，不再以逐项人工摆放测试阻塞框架开发。
新增调度层目前与原 STM32 主导的视觉串口路径分开。后续须明确一个任务流程所有者：
若采用树莓派调度，STM32 执行动作和本地保护；若 STM32 保留主流程，本框架可作模拟/
规划参考，不能两边同时发号施令。此次没有改变旧串口协议或接管电控。

## 已实现的主线

待机 → 连续新帧确认任务码 → 请求车载任务显示 → 第一批原料取料装车 →
粗加工按位置放置 → 粗加工再次取料装车 → 暂存平放 → 第二批重复原料和粗加工 →
暂存同颜色码垛 → 返回抽签启停区 → 请求显示实际反馈计数 → 完成。

每次只有一个待完成动作。物料抓起且装到托盘才完成 `pick_to_tray`；不能用手爪
夹着物料跨区域移动。容量可配 1/2/3，默认保守取 1；可先用容量 3 模拟主要流程。
区域 `raw/rough/staging/home`、位置 1/2/3 都是符号，不包含凭空编造的地图/毫米坐标。

四组码严格复用现有校验器。`156+123+516+231` 第二批在粗加工按 2/3/1 放，
在暂存按第一批同色位置 2/1/3 码垛；不是沿用第四组作为暂存位置。
每批全部粗加工放置完毕后才进入再次抓取阶段。

## 不用硬件直接运行

Mac 终端或树莓派终端，在工程根目录：

```bash
python3 -m pi_vision.mission_demo --task-code '156+123+516+231' --capacity 3
python3 -m pi_vision.mission_demo --capacity 3 --trace
python3 -m pi_vision.mission_demo --capacity 3 --fail-operation pick_to_tray
```

最后一条应以故障结束、发送停止意图，不应变成完成。正常模拟有 12 次装车抓取、
12 次放置（其中 3 次码垛），这些都是模拟计数，不是官方得分或实车成功记录。
不需要重复安装 Python 包；模拟不导入 OpenCV、ROS、串口或相机。

## 树莓派 ROS 2 接口

树莓派终端，工程根目录，用系统 Python，不覆盖现有 apt 环境：

```bash
source /opt/ros/jazzy/setup.bash
python3 -m tools.mission_ros_smoke --capacity 3
python3 -m tools.mission_ros_smoke --capacity 3 --fail-operation pick_to_tray
```

上述有界检查通过真实 ROS 2 发布/订阅传递模拟指令、反馈和状态，使用
`mission_sim/command`、`mission_sim/feedback`、`mission_sim/status`，不访问硬件。
需要单节点持续模拟可运行：

```bash
python3 -m pi_vision.mission_ros --ros-args -p simulation:=true -p auto_mock:=true -p autostart:=true -p capacity:=3
```

默认 `python3 -m pi_vision.mission_ros` 是非模拟、待机模式；不会自行开相机、
电机或串口。主题为 `mission/command`、`mission/feedback`、`mission/control`、
`mission/status`。模拟与非模拟同时按主题和消息 `simulation` 布尔值隔离，禁止混用。
真实模式拒绝自动启动和自动假反馈。
`capacity`、`home_id`、`observation_timeout` 以及各操作的 `timeouts.<operation>`
均可用 ROS 参数在启动时配置，例如 `-p timeouts.navigate:=30.0`。
采集/动作时限都是工程预算，不自动当作比赛允许的停留时间。

主题当前使用 `std_msgs/msg/String` 中的 schema 1 JSON，目的是先固定接口和跑通主线；
尚不是定版自定义 ROS 消息，更不是已有 STM32 字节串口协议。指令/反馈 reliable、
volatile，不向晚加入的驱动回放历史动作；状态 transient-local。相机结果后端按下文接入，
现已支持扫码/原料物料；导航、机械臂、电控桥和车载显示后端仍待接入。

## 消息契约

指令包含 `schema_version=1`、`simulation`、`run_id`、`command_id`、`step_index`、
`operation`、`params`、`issued_at_unix_s`、`timeout_s`。同一动作仅发送一次。
接收驱动必须按 `(run_id, command_id)` 幂等去重，断连不盲目重发不可逆动作。

反馈必须匹配 schema、模拟模式、运行和指令 ID：

```json
{"schema_version":1,"simulation":false,"run_id":"当前运行ID","command_id":"当前指令ID","status":"done","data":{"grasped":true,"loaded":true}}
```

`accepted` 只表示收到，不推进任务。`failed` 进入停止流程。

| operation | 后端责任 / 完成反馈 data |
| --- | --- |
| `read_task` | 扫码相机唯一采集入口输出 `observation`，由调度器连续新帧确认，不能直接 `done` |
| `display_task` | 实际车载显示回传 `displayed=true`；调试网页不能代替 |
| `navigate` | 解析目标区、执行导航，到位且停稳才回 `arrived=true, stopped=true` |
| `scan_material` | 物料相机返回指定区、指定色；粗加工重抓还要匹配位置编号 |
| `pick_to_tray` | 校准坐标转换和实际动作，`grasped=true, loaded=true` 才计数 |
| `locate_position` | 识别放置位置 1/2/3；第二批暂存还须观察到正确同色底层 |
| `place_from_tray` | 实际释放且验证放置，`released=true, placement_verified=true` 才计数 |
| `report_counts` | 将已反馈的抓取/放置统计送车载显示，回 `displayed=true` |
| `stop` | 取消所有待执行动作并停稳，回 `stopped=true` |

感知反馈 `status="observation"`，data 的来源字段：
`status=ok/no_target`、`valid`、`acquired_unix_s`、`source_sequence`、
`source_session_id`、`camera_id`、`camera_role=qr/material`。
`read_task` 另含 `task_code` 字符串或 null。无效帧打断扫码连续确认；重复来源不重复计票。

物料/位置感知另含 `zone`、`coordinate_frame="camera_image"`、`units="pixels"`、
`width/height`、`targets` 列表。每个目标含 `u/v`、`point_kind` 和 `color_id` 或
`position_id`。粗加工重抓目标同时含其 `position_id`；码垛位置目标还含 `base_color_id`。
位置号和评分同心环号不是一回事，不允许用固定 `ring_no=1` 占位。
像素左上为原点，右/下为正；用识别原图，不用预览缩放坐标。

单个匹配目标、有限且在画幅内的像素才被采用；多目标歧义、错色、错区、错位置、
过期/重复/请求前采集的帧都不推进。默认来源最大年龄 1 秒。采集时间用 Unix 秒，
跨主机须同步时钟；调度动作期限使用本机单调时钟。相机错误立即请求停止。
旧串口没有完整这些字段，需要桥接适配，不能声称自动兼容或已经完成 STM32 联通。

抓取/放置意图携带 `target` 的像素/来源信息和 `target_expires_at_unix_s`，
`requires_calibrated_backend=true`。驱动**必须在开始动作前**检查期限和标定条件；
没有本机标定应拒绝动作，不能使用截图比例、固定 20 cm 或把像素当毫米。
实际动作完成可以晚于图像有效期，但必须在动作期限内回报，且不能用已经过期的目标启动。

## 故障、取消与恢复

动作超时、后端报错或成功条件不成立均取消未派发意图、发布 `stop`，不自动重试抓放。
旧运行、旧指令或迟到成功不会推进、重复计数。停止有独立期限；未确认停止时不允许重开任务。
停止后可显式重新 `start` 建新 run，**不是从中断步骤无条件续跑**：需要操作者/主控核对
真实托盘、已放物料和场地状态。停机失败需真实本地电控看门狗/急停兜底，ROS 消息不能
保证电机已物理停止。框架未包含比赛中人工干预或遥控的许可。
状态中的 `stop_confirmed` 只标记是否存在未确认的停止事务，不是实时底盘停稳传感器；
真实底盘是否停稳仍由对应动作反馈和电控本地状态判断。

`mission/control` 的 start/cancel 还须带唯一 `request_id`、`issued_at_unix_s`、
schema 和模拟标记；仅接受 2 秒内的新请求。对同一请求去重，不接受历史 start 回放。

已增加下述单相机真实视觉后端；下一步扩展相机角色配置与真实电控适配器，再做固定安装
标定、位置识别及实车联调。ROS 任务节点与视觉后端可运行，不等于导航或整车已完成。

## 无网页的真实视觉后端

`pi_vision.vision_ros` 订阅 `mission/command`，向 `mission/feedback` 发布 schema 1
反馈。仅接受 `simulation=false` 的有效请求；不自动启动任务调度器，不打开串口、网页
或电机。使用现有 `FrameStreamer` 唯一采集线程和检测器，关闭预览绘制及 JPEG 编码。
节点启动后持有相机并持续采集，识别算法按请求启停；停止识别不等于释放相机。

目前一个显式指定的物理相机依次承担扫码/物料角色；不会偷偷打开配置中的第二相机或
切换失败后回退到另一个相机。双摄角色配置仍待实现和实测。启动前必须核对相机无占用，
不能与 `pi_vision.stream`、`qr_check` 等同时运行。

支持 `read_task`：发送每个新鲜帧的合法候选或空观测，任务层连续确认，绝不发送锁定旧码
冒充新观测。支持原料区 `scan_material`：显式 HSV、六色业务编号，原图坐标和
`point_kind=contour_bounding_box_center`，并附相机 ID、会话/来源帧号、采集 Unix 秒、
画幅和 HSV 参数哈希。采集时间为主机读帧完成时间，不是曝光时间；同机使用无需跨主机
相减单调时钟。无匹配目标发 `no_target`；多个同色目标保留歧义，由任务层拒绝推进。

粗加工重抓及 `locate_position` 需要独立的位置编号，不使用 `ring_no=1` 冒充位置。
PDF 平面图给出 `580×150 mm` 区域、三处靶心沿长边排列且首末间距 `300 mm`；每处靶心
包含中心位置数字及外侧断续同心评分环（T 第 5 页）。节点启动时显式提供
`--position-digits --position-orientation {0,90,180,270}` 作为基础观察方向；每个数字会
独立比较四种直角旋转并输出候选朝向，不假定三个数字方向一致。数字裁剪先用形态学长线核
移除尺寸线，再做组件筛选和模板比较。思路参考 ICAROB2024 论文的二值形态处理、轮廓筛选和
最小外接圆；当前 Hough 对断续圆弧保留作补充。低置信、缺字、遮挡或重复编号不输出目标，
位置号与评分环号分开。印刷字体、实物视角和抓取标定尚未验证，不可直接实机控制。
`scan_material` 的 `/state` 目标另带服务会话内 `track_id`、`tracking_status`、颜色、置信度
及当前帧原图像素框中心。关联按颜色、受限位移和时间间隔进行，不平滑坐标；短时漏检后可
延续身份，但空帧不会上报旧坐标。ROS 坐标字段是左上原点像素，并显式标记
`metric_coordinates_valid=false`、`calibration_required=true`，不是毫米抓取点。
超时、采集错误或处理超时终止请求，旧/重复/模拟指令及请求前帧不被采用。
请求去重上限 4096，达到上限需在待机时重启。`stop` 或后续动作指令会关闭识别；
此后端不返回整车 `stopped=true`，真实动作端负责该确认。

树莓派终端，确认唯一相机无占用后运行（参数沿用当前基线，不用于证明抓取标定）：

```bash
cd /home/cuiguo/competition_vision
source /opt/ros/jazzy/setup.bash
python3 -m pi_vision.vision_ros \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0 \
  --material-backend hsv --hsv-profile config/hsv_profile.example.yaml \
  --material-roi 480 60 520 450
```

默认 1280×720、不去畸变、采集/结果有效期各 1 秒、扫码 10 秒。不更改颜色参数，
该入口拒绝非空 `v4l2_controls`，避免启动时应用未经确认的相机控制。
运行只等待识别请求；不自动发送开车或抓放命令。Ctrl+C / SIGTERM 有序关闭。
原生相机读取永久卡住时仍可能需要进程级终止，继承现采集模块的恢复边界。

有界诊断程序 `tools.vision_ros_smoke` 直接用 ROS 指令/反馈检查观测，不启动任务调度。
为隔离动作消费者，要求独立 domain 74、仅本机发现；生成图像测试也使用真实后端
消息契约 `simulation=false`，因此只能在该隔离环境中运行。默认用生成图像和真实检测器，
只有显式传 `--camera` 才访问硬件；两者证据分别标记，不混为实机识别验收。

以下 ROS 视觉冒烟命令须在已同步本版源码的树莓派工程目录运行；当前位置识别代码
尚未部署到正式目录。

```bash
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m tools.vision_ros_smoke --material-backend hsv --seconds 3
python3 -m tools.vision_ros_smoke --material-backend hsv --operation read_task --seconds 4
python3 -m tools.vision_ros_smoke --material-backend hsv --operation locate_position --position-digits --position-orientation 0 --position-id 2 --seconds 3
python3 -m tools.vision_ros_smoke --material-backend hsv --operation locate_position --position-digits --position-orientation 0 --position-id 2 --position-scene missing --seconds 3
python3 -m tools.vision_ros_smoke --material-backend hsv --operation locate_position --position-digits --position-orientation 0 --position-id 1 --position-scene duplicate --seconds 3
python3 -m tools.vision_ros_smoke --material-backend hsv --operation locate_position --seconds 3
```

最后一条故意不启用位置识别，预期以 `position_identity_not_implemented` 失败关闭。
合成字体 ROS 检查只证明生成样本的检测器、请求/反馈和连续新帧确认可工作，不证明比赛
印刷字体或真实圆环可识别。位置数字选项仍只在 Mac 工作区，尚未同步 Pi 正式目录。

工具的 `ok` 只表示收到观测且检查期间没有采集/处理错误；真实相机模式可全部为
`no_target`，不能据此证明颜色正确或扫码成功。独立局部对准软件与 ROS 模拟见
`ALIGNMENT_INTERFACE.md`，尚未插入本任务计划。位置数字识别目前是未经现场字体验证的
可选实验功能；真实标定及动作端仍待接入，不以视觉结果发送成功宣称动作闭环完成。

## 当前验证证据

接入真实视觉后端后，Mac 全套视觉范围测试 `177 passed, 172 subtests passed`；Pi `176 passed, 1 skipped`，
跳过的是缺少 Node.js 的网页 JS 模拟，不是任务测试。Pi Jazzy 的 ROS 正常模拟经
60 条指令到 completed；模拟抓取失败经 6 条指令到 fault 并确认停止，无虚假抓放计数。
结果位于 Pi `output/material_diagnostics/mission_framework_20261031_Al67eI/` 的
`normal.json`、`failure.json` 及日志。这些证据只覆盖软件模拟和 ROS 收发，不覆盖真车。

位置数字、PDF 平面合成场景及物料跟踪新增后，Mac 全套视觉测试为 `234 passed, 172 subtests passed`。Pi 的 ROS 2 Jazzy
在临时源码目录、domain 74/LOCALHOST、生成图像条件下串行检查：正常样本、缺字、重复编号及
关闭态均符合预期。另生成 PDF 风格三处断续同心环及 `3/2/1` 不同旋转数字，完整
`RingDetector→NumberedRingDetector` 链得到中心 `(250,360)/(640,360)/(1030,360)`，方向
`180°/180°/0°`。tracker 位移、遮挡、超时、歧义由单测覆盖；本轮未在 Pi 对跟踪器执行
ROS/动态样本检查。没有打开相机、串口或电机，也未改 Pi 正式源码。合成图不能证明比赛字体、
磨损/反光或相机透视场景准确率。

## 参考视频方法的可选实现（2026-11-01）

保留现有 HSV 颜色阈值与默认框中心。新增 `--material-center centroid`，在相同颜色轮廓上
用图像矩计算面积质心，目标 `point_kind=contour_area_centroid`；默认 `box` 仍输出
`contour_bounding_box_center`。ROI 处理后恢复全图坐标。改变点定义须重新标定；旧框中心
标定不能用于质心，局部对准层会检查点定义。不输出毫米坐标或抓取成功。

圆环新增 `--ring-backend morphology --ring-roi X Y W H`，ROI 必须显式提供。流程为
Otsu 反向二值化 → 3×3 核膨胀 4 次 → 3×3 均值滤波/重二值化 → 腐蚀 2 次 →
外轮廓面积质心。最大工作宽度 640，次数与核大小是工作图像上的起始参数，不是本车标定值。
额外检查凸包圆度、填充率、边界和原始边缘的多层径向支持，拒绝裁断目标及普通单环/实心圆。
返回所有合格候选，保留多目标歧义，不无条件取全图最大黑块。位置身份仍通过
`--position-digits --position-orientation 0` 等显式启用；逐个数字比较四种直角旋转。
`center_method=morphology_external_contour_centroid`，`radius_px` 是原始径向支持层的
代表半径，不是外缘半径、评分层数或毫米尺寸。方法面向接近平面正视的环，不承诺强透视适配。

ROS 节点的相机/HSV 参数沿用前文；上述选项附加到既有 `pi_vision.vision_ros` 命令。
当前改动没有部署或启动 Pi 服务，不应与已有唯一相机采集入口同时运行。

Mac 终端或树莓派终端，离线比较已保存的原图（不访问相机/串口）：

```bash
python3 -m tools.vision_method_compare 原图.png --output output/method_compare --material-roi X Y W H --ring-roi X Y W H
```

输出目录必须尚不存在。报告包含框中心/面积质心和霍夫/形态学两种圆环结果；另外保存六色
分割图、圆环 binary/dilated/blurred/eroded 阶段及叠加图。没有 ring ROI 时不运行形态学环路径。
数字模板仍是 OpenCV 字体候选，视频截图不能替代本车拍摄的地面印刷数字验证。
