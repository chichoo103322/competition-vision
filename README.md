# 智能工程竞赛机器人视觉

面向树莓派的机器人视觉代码，包含二维码任务码确认、物料识别与跟踪、浏览器视频预览，以及独立的颜色/任务码/雷达数据处理模块。这里的测试主要验证软件逻辑，不等同于整车比赛验收。

## 代码范围

- `pi_vision/`：树莓派相机、二维码、物料、圆环和预览服务。
- `robot_perception/`：独立的任务码、颜色、雷达数据处理命令行工具。
- `tools/`：静态物料验收、模型诊断和训练资料审计工具。
- `tests/`：不需要真实相机或串口的自动化测试。
- `config/vision_config.example.yaml`：设备配置样例，不含本机网络和标定参数。

## 安装

要直接推进比赛任务主线，先看 [任务框架与电控/ROS 2 接口](MISSION_FRAMEWORK.md)。
已有无硬件完整模拟、动作完成反馈、两批次/码垛映射、取消和故障停止，保留现有视觉基线：

```bash
python3 -m pi_vision.mission_demo --task-code '156+123+516+231' --capacity 3
```

这只运行软件任务框架，不开网页、相机、串口或电机；真实动作仍须接入后端和标定。

建议使用 Python 3.10 或更新版本，并在树莓派或开发机上建立虚拟环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[inference,serial]'
```

若只运行不使用 ONNX 推理或串口的代码，可省略可选项。树莓派系统也可使用发行版提供的 OpenCV 和 NumPy，但需确保运行服务的 Python 环境能导入依赖。

## 设备配置与模型

运行前复制 `config/vision_config.example.yaml` 为 `config/vision_config.yaml`，再按实际相机、串口、标定和识别后端修改。`config/vision_config.yaml` 被 Git 忽略，不会上传。

ONNX 模型权重不包含在仓库中。使用 YOLO 物料识别时，将经确认的模型放到本地 `models/` 目录，并在设备配置中设置 `detection.yolo_model_path`。仓库不会下载或替用户选择模型。没有模型时，请按当前测试目的关闭物料检测，避免误把软件测试结果当作识别结果。

## 启动预览

在完成设备配置和模型准备后，从项目根目录启动：

```bash
pi-vision-stream --port 8090 --camera 0
```

`--camera` 应填写已在当前设备验证过的相机编号。预览服务监听设备网络接口，浏览器地址使用设备当前 IP 和端口；不要将开发时的局域网地址写入代码或共享配置。按 `Ctrl+C` 停止。

扫码检查可使用：

```bash
robot-perception qr --camera 0 --timeout 15 --diagnostics
```

该命令会打开相机。开始前先停止其他占用同一相机的程序。

## 软件测试

```bash
python -m pytest tests --ignore=tests/test_lidar.py
```

以上命令运行视觉与任务码软件测试，不运行雷达测试。测试不会打开真实相机或串口。硬件兼容性、光照变化下的识别率、坐标标定、抓取精度、串口闭环和长时间运行仍需在实际设备上验收。

## 数据与隐私

本仓库不包含训练数据、模型权重、现场采集图像、识别报告、设备专属配置、备份和一次性更新包。相关本地目录已列入 `.gitignore`。提交前仍应检查 `git status` 和暂存区内容，确认没有加入设备凭据或现场数据。

## 现场参数与原图诊断（显式 HSV）

当前 Ubuntu 24.04 树莓派使用系统 Python/apt 依赖，不需重新 pip 安装。Mac 开发与传文件，树莓派采集和实测。当前桌面光照、ROI 和 3 px 静止抖动目标均不是已确认的赛场配置或机械容差。

`config/hsv_profile.example.yaml` 完整列出已有基线参数。复制为本机 `config/hsv_profile.yaml` 后可修改颜色范围、轮廓面积/形状条件和蓝/浅蓝判定参数；本机副本被 Git 忽略。也可只写需要覆盖的字段。业务编号固定为红 1、黄 2、蓝 3、绿 4、黑 5、浅蓝 6；`bands.3` 提取蓝与浅蓝的共同区域，再由 `blue` 分组判色，不单独配置 `bands.6`。OpenCV 色相 H 使用 0—179，S/V 使用 0—255；红色跨色相零点用两个范围。未知字段、非法上下限和非有限值会在启动时报错。

在当前经过核对的启动命令中增加以下参数（树莓派终端，先停止已有实例释放相机）：

```bash
--material-backend hsv --hsv-profile config/hsv_profile.yaml
```

省略 `--hsv-profile` 保留原有颜色基线；修改文件需要重启生效。`/state` 返回有效参数的 `hsv_profile_sha256`，用于核对版本。ROI 继续通过 `--material-roi X Y W H` 调整，坐标始终返回完整识别画幅的像素，预览缩放不改变坐标。改变 ROI 大小也会改变基于 ROI 面积计算的面积筛选，需一起复验。轮廓框中心尚未标定为抓取点。

相机曝光/白平衡可在本机 `config/vision_config.yaml` 的 `camera.v4l2_controls` 中按实际设备配置；默认 `{}` 保持设备现状，不自动锁定。支持 `white_balance_automatic`、`white_balance_temperature`、`auto_exposure`、`exposure_time_absolute`、`gain`、`power_line_frequency`，值须为整数。先用树莓派终端的 `v4l2-ctl --device <已核对设备路径> --list-ctrls` 核对支持项、单位及范围；先关闭自动模式才可设置对应手动值。控制项在相机启动时应用，失败会终止启动并释放相机，不会假装成功。它们是设备相关配置，不能直接复制另一相机的曝光数值。独立扫码相机不继承主相机的控制项。

要保存诊断，在现有启动命令增加：

```bash
--diagnostics-dir output/material_diagnostics/session_new \
  --diagnostics-samples 30 --diagnostics-interval 1
```

目录必须是新目录，避免覆盖旧样本。每个样本保存未经预览缩放/叠字的全分辨率 PNG 和对应 JSON，包含同一来源帧序号、采集时间（Unix 和单调时钟）、原图大小、ROI、有效参数、参数哈希、检测结果及错误状态。坐标系为 `camera_image`，单位为像素；有效标记要求检测未报错且完成时帧龄不超过 1 秒。相机请求配置不等于设备实测值。采样复用唯一采集入口，队列满时丢弃诊断样本，不阻塞等待写盘；最多 300 个样本，达到上限后视觉继续运行。`/state` 的 `diagnostics` 返回保存/丢弃/错误计数。现场图片只保存在本机忽略目录，不上传。

树莓派终端，从项目根目录离线回放（不打开相机或串口）：

```bash
python3 -m tools.hsv_replay output/material_diagnostics/session_new \
  --output output/material_diagnostics/session_new/replay.json
```

增加 `--profile config/hsv_profile_candidate.yaml` 可以比较候选参数，同时仍验证原参数能否重现已记录结果。回放检查哈希、图片尺寸和检测有效性，原结果不一致时返回非零退出码。合成图软件测试、历史图回放、静止实测和整车动态验收分别记录；换场地先采样再比较参数，不以坐标平滑掩盖漏检或过期数据。ROS 2 接入仍需满足本项目的视觉基线确认顺序；当前预留的 Python profile/采样接口可供未来节点复用，并不表示已实现 ROS 节点。

## 采集时间、有效性和故障状态

当前 `pi_vision.stream` 的 `/state` 为 `schema_version: 2`。`session_id` 标识本次服务运行，消费者同时使用它和来源帧号判断重启/去重。相机、二维码、物料、圆环的处理信息分别在 `capture`、`qr_observation`、`material_observation`、`ring_observation` 中，算法之间的异常分别记录。

| 字段/状态 | 含义 |
| --- | --- |
| `acquired_unix_s` | 主机读帧完成的 Unix 秒，便于记录；不是相机传感器曝光时间 |
| `acquired_monotonic_s`、`completed_monotonic_s` | 同一主机上的单调采集/完成时刻；超时不依赖系统日期调整 |
| `source_sequence`、`sequence` | 来源采集帧号与该算法观测序号；相同来源帧不会重复推理来增加观测次数 |
| `age_ms`、`processed_ms`、`timeout_ms` | 从源帧读完到查询时的年龄、处理耗时及失效时限 |
| `ok` / `no_target` | 新鲜且成功处理；后者确实没有目标，结果列表为空。两者 `valid=true`，是否存在目标还须看 `has_target` |
| `waiting` / `disabled` | 等待首个新观测 / 算法关闭，均无可用坐标 |
| `stale` / `timeout` / `error` | 结果过期 / 处理或扫码会话超时 / 检测错误，均 `valid=false` |
| `camera_*` | 采集等待、失效、故障或停止导致结果无效；结合 `capture.status` 看原因 |

只有 `valid=true` 且 `has_target=true` 的结果可作为当前视觉位置；它仍是未标定的全图像素中心。物料、圆环无效时 `/state` 和 Python getter 均返回空列表，不把历史坐标放回有效输出。采集失败后即使新帧恢复，也要等对应算法处理新来源帧才恢复结果。关闭/重扫会丢弃在途旧结果；串口桥的旧 FOUND 协议不能表示“沿用旧坐标”，因此当前桥遇到无目标、过期或故障统一发 NOT_FOUND，不再启用该桥过去的遮挡旧坐标保留。协议仍不能区分这些原因，原因可在 `/state` 查询。历史独立 `app.py` 路径不等于已经迁入本接口，固定距离和放置占位的边界仍见项目上下文。

默认采集超时 1 秒、结果源帧年龄/处理时限 1 秒、本轮任务码确认超时 10 秒；可在启动命令用 `--capture-timeout`、`--result-timeout`、`--qr-timeout` 调整（单位秒，必须为有限正数，重启生效）。边界达到时即无效。第一帧一直不返回也有采集时限。超时数值是工程设置，不是比赛允许的等待时长；按实际处理频率、预览帧率和控制响应预算调整，不能为保留旧坐标无限放宽。

二维码的 `task` 是经过严格合法性及连续新帧确认的本轮锁定任务，`latched=true`，保留到停止/重扫/服务停止；这不是一直可见的二维码。实时二维码 `qr` 仅在已锁定任务对应的当前观测仍新鲜时包含定位多边形，无码或过期就清空几何位置。未确认候选、超过扫码时限才完成的结果、含非法定位点的解码均不能变成确认成功。

读帧返回失败时立即作废旧结果；连续失败后由唯一采集线程释放并重开，重试间隔退避至最多 5 秒。成功读帧和新结果分别恢复有效性。驱动或推理原生调用若永久阻塞，状态可超时失效，但 Python 线程不能安全强杀该调用：不会并发开第二个采集器或无限排队推理，仍可能需要外部进程级重启。这不等于已验证真实 USB 拔插自动恢复。

预览的 `/frame.jpg` 在服务端无有效新帧时仍返回 HTTP 503，不放宽采集有效期。网页图像另有仅供人工查看的显示期限 `--preview-timeout`（默认 3 秒），以 `X-Preview-Max-Age-Ms` 发送；`X-Frame-Max-Age-Ms` 仍为采集期限。画面保守年龄（服务端年龄＋完整请求耗时）超出采集期限时标明“预览有延迟，仅供查看”，隐藏图上检测标记；到显示期限后清除图像。短暂请求失败可在原显示期限内保留上一帧并明确提示，但不会续期；持续断流仍会清空。识别坐标继续独立按原有 1 秒有效期清除，不能把延迟预览用于控制。网页使用本地计时，不将 Pi 的绝对单调时间与另一台电脑的时钟相减。已有连接页面在升级后应刷新一次以加载新逻辑。

树莓派终端的可复用软件故障检查（生成假帧，只监听本机临时端口，不打开 USB 相机、串口或电机，可与当前视觉服务同时运行）：

```bash
python3 -m tools.vision_health_check \
  --output output/material_diagnostics/lifecycle_fault_injection.json
```

覆盖目标消失/恢复、检测异常/恢复、处理超时与迟到结果、采集阻塞过期、过期图像 HTTP 503、读帧失败/重开/恢复及单采集入口。软件注入结果、真实相机在线采样、物理拔插和整车实测须分别报告。静止验收工具同时排除无效观测、重复来源帧和跨服务会话数据，保留对旧报告的兼容。
