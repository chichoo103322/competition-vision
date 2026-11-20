# 提前就绪、实体按键启动

`tools/chassis_button_service.py` 作为树莓派user systemd服务提前运行，独占电控UART、复用已有雷达采集；没有采集服务时按本机配置启动。雷达持续运行，必须收到STM32正常STATUS和新鲜扫描后才进入READY。无需Mac参与扫描、规划、上传或运动控制。

READY后，短按并松开KEY1（区1）或KEY0（区2），自动执行：ACK选区→选区后新扫描窗口→八段规划→完整几何校验→绑定请求上传→START→八站闭环→返回起点。仍保留至少4秒新扫描窗口，不在发令前缓存一条旧路线直接发车。初始地图航向+90°，站间保持上一站工作航向；实际本机配置在Git忽略的config/chassis_button.local.json，两区使用已确认的不同中心坐标。

两区 profile 现须显式配置 `work_heading_mode_enabled: true`，与STM32开启配置一致。报文 yaw、行驶车头和作业航向的区分、phase6等待与姿态校验见 [WORK_AREA_HEADING.md](WORK_AREA_HEADING.md)。服务准备后另生成 `output/chassis_button/direction_review.html` 显示三种航向。

## 与规则的关系

全国命题原文《02-2027年第十届 智能+工程创新赛道竞赛命题与运行》第7页要求调试结束后待机发车，裁判统一发令计时，一键式启动且只有一次启动机会；第1页要求比赛中完全自主运行。提前启动软件后，裁判发令时由实体按键启动，不需要途中远程START或继续确认。代码可以在一次完成后回到READY供后续练习；这不代表比赛允许同一轮重复启动，正式运行次数仍须遵守本场规则和裁判指令。

本入口仍是**底盘测试**：配置 `qr_station` 后，二维码站使用真实USB相机扫码；其余站仍为2秒模拟驻留，未接入颜色、色环、抓放、任务显示和计数。它落实一键自主启动的结构，不等于完整比赛功能或比赛验收通过。正式接口electrical_link.run_after_key仍要求真实作业完成反馈，不能用测试驻留冒充成功。

## 测试入口的真实二维码站

树莓派本机 `config/chassis_button.local.json` 增加：

```json
"qr_station": {
  "camera": "/dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0",
  "vision_config": "config/vision_config.yaml",
  "timeout_s": 15
}
```

二维码解码依赖见 `pyproject.toml` 的 `qr` 可选依赖（zxing-cpp 3.1.1）。Pi缺失时会落到OpenCV备用解码；本次真实屏幕样本上备用解码失败而ZXing成功，部署需安装该依赖。

收到本轮新按键后，用唯一采集子进程提前打开指定相机并持续读帧，预热期不解码、不保存任务码；上传/START前要求采集心跳新鲜。到站且电控回报位置/工作航向合格（work_id=101、WAIT_WORK/phase4）后，才发送本轮扫码边界：重置确认、保守丢弃接下来的4帧，再接受边界之后开始读取的不同帧。复用原QRDetector、严格任务码合法性校验和TaskCodeConfirmation，至少3张连续新帧且跨度0.2秒，最大帧间隔1秒。时间戳是单调时钟的软件读帧时间，不冒充传感器曝光时间。

确认成功才发送本站WORK_DONE；扫码进程先释放相机再返回成功结果。超时、非法码、相机失败不会按原2秒驻留放行，而是尝试STOP并释放相机；取消、规划失败、路线结束同样关闭。下一轮新按键才重新预热，待机不占相机。扫码期间继续轮询电控故障，父进程对相机/解码阻塞执行同一15秒到站超时并回收子进程。每轮新建会话，不读取上一轮任务码。相机缺失时不进入READY。未配置该字段的旧测试仍为纯驻留模式。

`wire.jsonl` 的 `qr_task_confirmed` 和本轮 `result.json` 的 `qr_task_code` 保存真实任务码与route_id；它目前仅供后续任务接入，不改变原料/抓放站行为，也不额外打开视觉串口或启动颜色/色环检测。相机由扫码进程独占，测试时不要另开预览。

## 三轴视觉对齐测试（默认关闭，待标定）

新增可选本机字段 `arm_alignment`，结构见 `config/arm_alignment.example.json`；启用时将示例对象放入该字段。示例所有未测参数为空、`enabled=false`，不会修改旧测试行为。该模式只做每个受管站的一次指定目标对齐并收回机构，不是完整物料搬运或正式比赛后端；日志及结果明确 `physical_grasp_completed=false`、`physical_place_completed=false`。每站目标可明确指定业务颜色/位置，或通过 `task_group`＋`task_index` 选择本轮已确认二维码中的一项；后者不代表已实现三件抓放顺序。第二批暂存不得用第四组码代替同色底层位置，码垛层高及同色底层确认尚未接入。

已按新 `car_code1 2/Core/Src/arm_work.c` 接入0x31/0x32/0x83，仍复用原ElectricalClient和唯一电控UART；云台52字节状态与底盘36字节状态分开解析。电机轴角度相对人工记录原点，换算固定为 `Z_deg=z_mm×360/40`、`reach_deg=reach_mm×360/113.1`、`yaw_motor_deg=yaw_mechanism_deg×3`，另乘各轴实测正负方向，线上单位0.01°。不自动清零或烧录；新固件原点无效时拒绝START。固件自带的避让、拍照姿态和下降量仍需实测：当前 `ARM_PICK_DROP_CDEG=9000` 按40mm/圈仅代表10mm，不依据注释猜下降量。本次不修改或烧录电控源码。

启用前必须实测并填入：各轴方向/行程与固件避让姿态、原点处旋转中心到夹爪抓取中心的半径、观察高度/平面、相机稳定路径、夹爪参考像素、HSV参数哈希、原图分辨率、像素误差到机构径向/切向毫米修正矩阵及其适用姿态范围、对齐容差和调整限幅。使用旋转＋伸缩的极坐标几何，不把旋转当作直线X轴，不把地面环、物料顶面、上层码垛共用一套未验证标定。

复用现有单相机FrameStreamer `/state`，新增只读 `camera_source` 用于防止串用二维码相机。物料源须显式HSV、top_face和中央窗口；环目标须由位置数字1/2/3识别提供，不能用占位ring_no。该适配器不另外打开相机或串口，也不自动拉起未配置的视觉服务。配置的物料视觉服务须先在树莓派运行。stream已显式开放已有圆环/数字算法参数：`--ring-backend morphology --ring-roi X Y W H --position-digits --position-orientation 0|90|180|270`，与ROS入口复用同一配置函数；不提供假定的ROI或安装方向。环内数字仍是实验性模板候选，必须对真实印刷物、相机姿态范围/遮挡做验证，不能因软件测试通过宣称实物位置识别合格。

受管站102..107：底盘WAIT_WORK/姿态合格 → 查询云台拍照姿态到位 → 仅使用停稳＋settle之后的新来源帧 → 顶面/指定环相对夹爪参考点计算偏差 → 限幅ALIGN绝对角度 → 等云台ALIGNED及对应command_id → 重新看新帧 → 连续至少3帧满足像素容差和配置确认跨度 → FINISH → 等READY_LEAVE → WORK_DONE。期间持续轮询底盘，云台超时/故障/上下文改变、目标丢失/歧义、旧帧/错误相机、标定范围或累计行程超限沿用路线STOP，不退回2秒放行。对齐测试不发送PICK/GRIP_DONE；真实夹爪反馈和车上料盘转运未实现，不用固定延时补成功。

## 物料与色环相机服务

`deploy/material-vision.service` 使用 `config/material_vision.local.yaml` 中核对过的物料相机 by-id 路径，复制配置样例 `config/material_vision.example.yaml` 后填入设备。本服务单独开机启动，共用一个 FrameStreamer 采集入口处理物料和色环，`/state` 与预览使用端口8090。车头扫码相机仍由按键扫码进程按轮次管理。

当前转盘测试命令显式 HSV、独立顶面中心、原图1280×720接近窗口(480,240,320,240)，在窗口内找唯一指定颜色；按本轮任务码切换颜色。当前暂停数字圆环识别，后续再联调位置数字及环中心；`ring_no` 不能当作放置位置1/2/3。该窗口是沿用的观察范围，尚需根据固定安装画面复核。云台自动对齐启用仍需独立视觉/机械标定。夹爪和料盘舵机按当前任务范围后续接入。

树莓派终端可检查 `systemctl --user status material-vision`，查看日志用 `journalctl --user -u material-vision -n 30`。调整配置后重启该服务，避免额外启动相机采集程序。

## 转盘等待与相机控制台

本机 `turntable_work.enabled=true` 时，原料站102/105从本轮确认任务码的第一/第三组取三色顺序。例如231先等待黄色，再蓝色、红色。相机共用服务POST `/material-target` 切换目标并清空旧坐标。进入接近窗口且顶面连续新帧有效后，标定完整的alignment配置才能进入云台闭环；alignment为空时发布calibration_required、保持静止且不发送WORK_DONE。当前三色序列是对齐测试，两个舵机后续接入。非原料区数字圆环任务未标定时明确停止，避免模拟放行。

Mac控制台“相机与物料对齐”页提供双相机切换，页面随Mac应用打包。树莓派camera-console.service仅提供8091的状态和帧接口，通过已有SSH隧道访问。扫码查看使用同一QRStationWork采集器；发车预热前同步释放手动查看，切页或8秒无查看心跳后释放。软件更新在Mac进行，Pi保留采集和接口后端。

## 正式比赛入口也使用相同实体键启动

`tools/competition_button_service.py` 与 `deploy/competition-button.service` 是正式入口，复用同一个雷达预热和UART待机。KEY1/KEY0绑定选区后直接接正式执行，不需要第二次按键或电脑发START。每轮真实作业全部完成并收到DONE后自动回到READY；下一次新按键重新选择启停区，并重新采集和规划完整路线。两个systemd服务互斥，不能同时调度同一底盘。

正式本机配置 `config/competition_button.local.json` 增加 `competition_backend="真实模块:工厂函数"`。工厂接收本机配置，返回实际硬件适配对象，须满足：

- `simulation=False`；`ready()` 仅在真实视觉/机构等依赖已就绪时返回True。
- `plan(state, selection, route_id, check_current)` 返回已实车标定和允许运动的正式路线（valid_for_motion=True），不得带test_mode；使用指定route_id/start_zone，保持按键绑定。state为按键后新雷达计划，计算期间调用check_current以发现取消/换区。
- `on_work(client, status)` 每站仅调用一次启动真实作业；返回None后使用可选 `poll_work(client, status)` 非阻塞查询真实结果，True=完成、False=失败。缺少真实结果时继续等待。不得用固定驻留、mock输出伪造成功；不得另开线程抢读UART。

正式执行调用ElectricalClient.run_selected_route，保留作业门、请求绑定、超时和故障停止；不会运行测试版run_closed_loop或2秒定时释放。启动前仍复核扫描/几何及选区。正式后端未配置、未就绪时不会进入正式READY或发车，不自动回退测试。该后端涉及真实扫码/抓放/显示/统计，当前尚未接通；这里只完成正式一键启动和硬件流程接入入口，不宣称正式整车可参赛。

## 连续按键运行

- READY前按下的旧请求会被确认接收但忽略，不会在准备好后突然发车。上电时按住按键仍须按固件要求先松开。
- 不设置持久化的启动锁。READY后的每次新按键都会选择启停区、进行新的扫描和规划；完成后回到READY，失败后记录错误并等待控制器回到空闲再READY。
- 服务重启会重新读取控制器tick作为基线，重启前积压的旧按键会被忽略，不会自动重放路线。控制器处于运动或故障状态时只等待，不接收启动键。
- 多次运行是软件能力；正式比赛每轮启动次数仍按比赛规则执行，练习时可重复按键验证。
- STATUS超时或MCU重启时回到WAIT_CONTROLLER；通信恢复后重新建立READY基线。串口设备重枚举导致进程异常时由systemd重启服务。
- 设备断开/数据过期不会用旧计划启动；运动异常由现有run_closed_loop尝试STOP。电控没有通信断开自动停车的看门狗，此封装不新增该能力。

待机状态与每次状态转换记录在 `output/chassis_button/status.json`；全部UART TX/RX_RAW记录在wire.jsonl，准备快照和结果在prepared_live.json/result.json。STOP不是正常拐点控制命令。

查看状态（无需运行命令来解锁）：

```bash
cat /home/cuiguo/competition_vision/output/chassis_button/status.json
journalctl --user -u chassis-button.service -n 20 --no-pager
```

看到READY后，等裁判发令再按实体键。上电自动准备依赖user service已enable且用户开启linger；机器上电/断电启动能力需另外做完整断电实测，不能由一次systemctl启动成功推定。
