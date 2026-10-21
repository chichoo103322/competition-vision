# 智能工程竞赛机器人视觉

面向树莓派的机器人视觉代码，包含二维码任务码确认、物料识别与跟踪、浏览器视频预览，以及独立的颜色/任务码/雷达数据处理模块。这里的测试主要验证软件逻辑，不等同于整车比赛验收。

## 代码范围

- `pi_vision/`：树莓派相机、二维码、物料、圆环和预览服务。
- `robot_perception/`：独立的任务码、颜色、雷达数据处理命令行工具。
- `tools/`：静态物料验收、模型诊断和训练资料审计工具。
- `tests/`：不需要真实相机或串口的自动化测试。
- `config/vision_config.example.yaml`：设备配置样例，不含本机网络和标定参数。

## 安装

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
