# 比赛机器人树莓派工程

Mac 用于开发和传代码，树莓派负责相机采集、识别、与电控通信及实车测试。

日常按键测试从 [按键启动说明](docs/guides/CHASSIS_BUTTON_START.md) 开始。当前测试入口已接入车头二维码相机的预热、任务码确认和关闭。夹爪视觉对齐代码已接入本地测试流程，启用前还需要电控配合与实物标定。最新部署情况和设备快照见 [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md)。

## 目录

```text
车子/
├── pi_vision/              树莓派视觉、任务状态、通信与对齐实现
│   └── detectors/          二维码、物料、颜色、圆环检测器
├── robot_perception/      独立任务码、颜色和雷达数据处理命令行包
├── tools/                 服务入口、现场检查和离线开发工具
├── config/                配置样例、场地图及被忽略的本机配置
├── deploy/                systemd 服务定义
├── tests/
│   ├── electrical/        底盘、串口协议、按键、机械臂与站点工作
│   ├── vision/            采集、识别、预览、定位与视觉验收
│   ├── mission/           任务流程与视觉对齐状态机
│   ├── desktop/           桌面连接与通信监看
│   ├── perception/        独立颜色、任务码与 CLI
│   ├── lidar/             雷达导航、显示和规划软件测试
│   └── test_lidar.py      独立雷达数据处理测试，按既有规则单独运行
├── docs/
│   ├── guides/            启动、操作和联调说明
│   └── design/            接口、任务框架和开发设计
├── examples/qr/           二维码测试图片
├── output/                本机采集、日志、报告与验证结果
├── archive/               本机历史更新包、恢复包、调查和旧笔记
├── backups/               本机备份
├── README.md              工程导航
├── PROJECT_CONTEXT.md     最新进度、部署记录与交接边界
├── AGENTS.md              长期开发规则
└── pyproject.toml         依赖、安装、命令入口与测试发现配置
```

`output/`、`archive/`、`backups/`、`tmp/`、构建产物和设备专属配置由 Git 忽略。原来的根目录更新包及旧本地笔记已集中到 `archive/`，目录迁移记录保存在 `output/project_organization_20261118/moves.json`。

## 从哪里看代码

| 工作 | 入口与实现 |
| --- | --- |
| 常用按键测试 | `tools/chassis_button_service.py` → `tools/electrical_route_test.py` |
| 车头扫码 | `tools/qr_station_work.py`、`pi_vision/task_confirmation.py` |
| 夹爪相机对齐 | `tools/arm_alignment_work.py`、`pi_vision/arm_alignment.py` |
| 电控通信 | `pi_vision/electrical_link.py`、`electrical_protocol.py`、`arm_protocol.py` |
| 相机识别和预览 | `pi_vision/stream.py` → `pipeline.py` → `detectors/` |
| 物料顶面定位 | `pi_vision/material_region.py`、`top_face.py` |
| 比赛任务框架 | `pi_vision/mission.py`、`vision_backend.py` |
| 桌面调试台 | `tools/robot_desktop.py`、`desktop_remote.py`、`communication_monitor.py` |
| 离线检查 | `tools/hsv_replay.py`、`material_acceptance.py`、`vision_health_check.py` |
| 底盘模拟 | `tools/chassis_route_sim.py`、`chassis_route_benchmark.py` |
| 雷达预览与规划 | `tools/lidar_view.py`、`lidar_navigation.py`、`lidar_mission.py` |

`tools/` 中的现有文件路径同时用于启动命令、部署配置和工具导入，整理后仍可使用原命令。

## 操作与设计文档

| 内容 | 文档 |
| --- | --- |
| 实车按键测试、扫码与对齐开关 | [按键启动](docs/guides/CHASSIS_BUTTON_START.md) |
| 底盘路线测试 | [路线测试](docs/guides/CHASSIS_ROUTE_TEST.md) |
| 树莓派与电控联调 | [电控通信](docs/guides/ELECTRICAL_COMMUNICATION.md) |
| 到站航向 | [作业区航向](docs/guides/WORK_AREA_HEADING.md) |
| 相机安装、HSV、预览和诊断 | [视觉使用说明](docs/guides/VISION.md) |
| 雷达预览 | [雷达说明](docs/guides/LIDAR_VIEW.md) |
| 任务状态和 ROS 2 接口 | [任务框架](docs/design/MISSION_FRAMEWORK.md) |
| 对齐接口 | [对齐设计](docs/design/ALIGNMENT_INTERFACE.md) |
| 后续开发 | [开发计划](docs/design/DEVELOPMENT_PLAN.md) |
| 路线耗时分析 | [时序分析](docs/design/ROUTE_TIMING_REVIEW.md) |

## 开发检查

Mac 终端，在项目根目录运行软件测试。

```bash
python3 -m pytest -q tests --ignore=tests/test_lidar.py
node tests/lidar/test_lidar_display.js
```

按功能验证时，可以把 `tests` 换成 `tests/vision` 或 `tests/electrical` 等子目录。软件测试、历史图回放、相机静止实测和整车测试分别记录结果。

安装方式和树莓派相机命令见 [视觉使用说明](docs/guides/VISION.md)。当前物料识别显式使用 `--material-backend hsv`。
